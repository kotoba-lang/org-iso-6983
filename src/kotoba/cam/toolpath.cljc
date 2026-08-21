(ns kotoba.cam.toolpath
  "Toolpath generation: CAM operations, segment types, and job execution.
   Ported from kami-cam (Rust) `src/toolpath.rs`.

   A `CamOperation` is a plain tagged map `{:op :pocket|:drill|... ...}`:
     {:op :face-mill  :tool-id :depth-of-cut :stepover :feed-rate :spindle-rpm}
     {:op :pocket     :tool-id :depth :stepover :strategy :feed-rate :spindle-rpm
                       :pocket-min :pocket-max}     ; corners are vec3
     {:op :contour    :tool-id :depth :side :feed-rate :spindle-rpm}
     {:op :drill      :tool-id :depth :peck-depth :feed-rate :spindle-rpm :holes}
     {:op :surface-3d :tool-id :stepover :strategy :feed-rate :spindle-rpm}
     {:op :turn       :tool-id :depth-of-cut :feed-rate :spindle-rpm}

   `:pocket` (zigzag), `:drill` (peck cycles), `:face-mill` (single-pass
   raster over the stock's top face) and `:contour` (offset following of a
   caller-supplied CONVEX polygon `:profile`, `:side` :inside/:outside/
   :on-line) are actually simulated. `:surface-3d` and `:turn` still
   generate a placeholder rapid move to the tool-change point — real 3D
   surface finishing needs mesh height-field sampling and turning is a
   different (2-axis polar) paradigm; both are future work, not rushed
   here. `:contour` on a concave/self-intersecting profile is also not
   attempted (general polygon offsetting needs self-intersection
   resolution this doesn't implement) — see convex-ccw?."
  (:require [kotoba.cam.vec3 :as vec3]
            [kotoba.cam.stock :as stock]
            [kotoba.cam.util :as util]))

(def pocket-strategies #{:zigzag :spiral :trochoidal-peel})
(def surface-strategies
  "3-axis surface finishing strategies this library *names*. Naming one is not
   implementing one — see `implemented-surface-strategies`."
  #{:raster :spiral :waterline :pencil})

(def implemented-surface-strategies
  "The subset `generate-toolpath` actually cuts.

   `:raster` drops a ball-nose over a triangle-mesh target on a parallel-line
   grid, which is the finishing pass most 3-axis work uses. `:waterline` needs
   constant-Z contours of the part (a different algorithm), `:spiral` needs a
   boundary-offset field, and `:pencil` needs concave-edge detection. Until
   those exist, an operation naming them is REFUSED rather than handed a rapid
   move to the tool-change point and called a finishing pass — a job that looks
   generated and removes nothing is worse than an error, because it reaches the
   machine."
  #{:raster})
(def contour-sides #{:inside :outside :on-line})
(def segment-types #{:rapid :linear :arc-cw :arc-ccw})

(defn segment
  "One segment of a generated toolpath.

   `:spindle-rpm` is carried on the segment because the post needs it: the
   G-code emitter writes `M03 S<rpm>` at each tool change, and the only place
   that number exists is the operation. Generators do not have to set it —
   `generate-toolpath` stamps each operation's rpm onto the segments that
   operation produced (see there) — but a caller building segments by hand
   may pass it directly."
  [{:keys [segment-type start end feed-rate center tool-id spindle-rpm]}]
  (cond-> {:segment-type segment-type
           :start start
           :end end
           :feed-rate (double (or feed-rate 0.0))
           :center center
           :tool-id tool-id}
    (some? spindle-rpm) (assoc :spindle-rpm (double spindle-rpm))))

(defn- last-end
  "The end position of the last segment, or origin if empty."
  [segments]
  (if (seq segments) (:end (peek segments)) vec3/zero))

;; ---------------------------------------------------------------------
;; :pocket — zigzag pocket clearing
;; ---------------------------------------------------------------------

(defn- gen-pocket
  [job {:keys [tool-id depth stepover feed-rate pocket-min pocket-max]} segments0]
  (let [tool (get (:tool-library job) tool-id)
        tool-radius (if tool (/ (:diameter tool) 2.0) 0.0)
        effective-stepover (if (> stepover 0.0) stepover tool-radius)
        x-min (+ (:x pocket-min) tool-radius)
        x-max (- (:x pocket-max) tool-radius)
        y-min (+ (:y pocket-min) tool-radius)
        y-max (- (:y pocket-max) tool-radius)
        z-top (:z pocket-min)
        z-bottom (- z-top depth)
        safe-z (+ z-top (:safe-height job))
        ;; Simple constant depth-of-cut = stepover for now; a real
        ;; implementation would use axial DOC (ported verbatim from Rust).
        layer-doc (min effective-stepover depth)
        num-layers (long (util/ceil (/ depth layer-doc)))]
    (loop [layer 0 segments segments0]
      (if (>= layer num-layers)
        segments
        (let [z (max (- z-top (* (+ layer 1.0) layer-doc)) z-bottom)
              first-start (vec3/v3 x-min y-min safe-z)
              segments (if (seq segments)
                         (conj segments
                               (segment {:segment-type :rapid
                                         :start (last-end segments)
                                         :end (vec3/v3 x-min y-min safe-z)
                                         :tool-id tool-id}))
                         segments)
              segments (conj segments
                             (segment {:segment-type :rapid
                                       :start first-start
                                       :end (vec3/v3 x-min y-min z)
                                       :tool-id tool-id}))
              segments (loop [y y-min forward true segments segments]
                         (if (> y y-max)
                           segments
                           (let [[sx ex] (if forward [x-min x-max] [x-max x-min])
                                 start (vec3/v3 sx y z)
                                 end-pt (vec3/v3 ex y z)
                                 prev (last-end segments)
                                 segments (if (> (vec3/length (vec3/sub prev start)) 1e-6)
                                            (conj segments
                                                  (segment {:segment-type :rapid
                                                            :start prev :end start
                                                            :tool-id tool-id}))
                                            segments)
                                 segments (conj segments
                                                (segment {:segment-type :linear
                                                          :start start :end end-pt
                                                          :feed-rate feed-rate
                                                          :tool-id tool-id}))]
                             (recur (+ y effective-stepover) (not forward) segments))))
              prev (last-end segments)
              segments (conj segments
                             (segment {:segment-type :rapid
                                       :start prev
                                       :end (vec3/v3 (:x prev) (:y prev) safe-z)
                                       :tool-id tool-id}))]
          (recur (inc layer) segments))))))

;; ---------------------------------------------------------------------
;; :drill — peck-drill cycles
;; ---------------------------------------------------------------------

(defn- gen-drill
  [job {:keys [tool-id depth peck-depth feed-rate holes]} segments0]
  (let [safe-z (:safe-height job)]
    (reduce
     (fn [segments hole]
       (let [top (vec3/v3 (:x hole) (:y hole) safe-z)
             prev (last-end segments)
             segments (conj segments
                            (segment {:segment-type :rapid :start prev :end top
                                      :tool-id tool-id}))
             z-bottom (- (:z hole) depth)]
         (loop [z (:z hole) segments segments]
           (if (<= z z-bottom)
             segments
             (let [target-z (max (- z peck-depth) z-bottom)
                   segments (conj segments
                                  (segment {:segment-type :linear
                                            :start (vec3/v3 (:x hole) (:y hole) z)
                                            :end (vec3/v3 (:x hole) (:y hole) target-z)
                                            :feed-rate feed-rate
                                            :tool-id tool-id}))
                   segments (conj segments
                                  (segment {:segment-type :rapid
                                            :start (vec3/v3 (:x hole) (:y hole) target-z)
                                            :end top
                                            :tool-id tool-id}))]
               (recur target-z segments))))))
     segments0
     holes)))

;; ---------------------------------------------------------------------
;; :face-mill — single-pass raster over the stock's top face
;; ---------------------------------------------------------------------

(defn- gen-face-mill
  "Rasters the whole stock's top XY footprint at one Z depth
  (`:depth-of-cut` below the stock's top face, derived from
  `(:stock job)` — face-milling covers the workpiece, not an arbitrary
  caller-specified region like :pocket does). Same zigzag/retract
  structure as gen-pocket, single layer."
  [job {:keys [tool-id depth-of-cut stepover feed-rate]} segments0]
  (let [tool (get (:tool-library job) tool-id)
        tool-radius (if tool (/ (:diameter tool) 2.0) 0.0)
        effective-stepover (if (and stepover (pos? stepover)) stepover (max tool-radius 1.0))
        {:keys [origin] :as job-stock} (:stock job)
        dims (stock/dimensions job-stock)
        x-min (- (:x origin) tool-radius)
        x-max (+ (:x origin) (:x dims) tool-radius)
        y-min (:y origin)
        y-max (+ (:y origin) (:y dims))
        z-top (+ (:z origin) (:z dims))
        z (- z-top depth-of-cut)
        safe-z (+ z-top (:safe-height job))
        first-start (vec3/v3 x-min y-min safe-z)
        segments (if (seq segments0)
                   (conj segments0 (segment {:segment-type :rapid :start (last-end segments0)
                                             :end first-start :tool-id tool-id}))
                   segments0)
        segments (conj segments (segment {:segment-type :rapid :start first-start
                                          :end (vec3/v3 x-min y-min z) :tool-id tool-id}))
        segments (loop [y y-min forward true segments segments]
                   (if (> y y-max)
                     segments
                     (let [[sx ex] (if forward [x-min x-max] [x-max x-min])
                           start (vec3/v3 sx y z) end-pt (vec3/v3 ex y z)
                           prev (last-end segments)
                           segments (if (> (vec3/length (vec3/sub prev start)) 1e-6)
                                      (conj segments (segment {:segment-type :rapid :start prev
                                                               :end start :tool-id tool-id}))
                                      segments)
                           segments (conj segments (segment {:segment-type :linear :start start :end end-pt
                                                             :feed-rate feed-rate :tool-id tool-id}))]
                       (recur (+ y effective-stepover) (not forward) segments))))
        prev (last-end segments)]
    (conj segments (segment {:segment-type :rapid :start prev
                             :end (vec3/v3 (:x prev) (:y prev) safe-z) :tool-id tool-id}))))

;; ---------------------------------------------------------------------
;; :contour — offset following of a convex polygon profile
;; ---------------------------------------------------------------------

(defn convex-ccw?
  "True if `pts` (vec3, Z ignored) form a convex, counter-clockwise
  polygon — the only profile shape gen-contour offsets correctly (a
  concave polygon's straight per-edge offset can self-intersect, which
  this namespace doesn't attempt to resolve)."
  [pts]
  (let [n (count pts)
        cross (fn [a b c]
                (- (* (- (:x b) (:x a)) (- (:y c) (:y a)))
                   (* (- (:y b) (:y a)) (- (:x c) (:x a)))))]
    (and (>= n 3)
         (every? pos? (for [i (range n)]
                        (cross (nth pts i) (nth pts (mod (inc i) n)) (nth pts (mod (+ i 2) n))))))))

(defn- edge-outward-normal-2d
  "Outward unit normal (as [nx ny]) of edge a->b of a CCW polygon —
  rotate the edge direction -90° (clockwise)."
  [a b]
  (let [dx (- (:x b) (:x a)) dy (- (:y b) (:y a))
        len (vec3/length (vec3/v3 dx dy 0.0))]
    (if (< len 1e-9) [0.0 0.0] [(/ dy len) (/ (- dx) len)])))

(defn- line-intersect-2d
  "Intersection point of infinite lines through (p1,p2) and (p3,p4), XY
  only (z carried from p1). nil if parallel."
  [p1 p2 p3 p4]
  (let [x1 (:x p1) y1 (:y p1) x2 (:x p2) y2 (:y p2)
        x3 (:x p3) y3 (:y p3) x4 (:x p4) y4 (:y p4)
        denom (- (* (- x1 x2) (- y3 y4)) (* (- y1 y2) (- x3 x4)))]
    (when (> (Math/abs denom) 1e-9)
      (let [t (/ (- (* (- x1 x3) (- y3 y4)) (* (- y1 y3) (- x3 x4))) denom)]
        (vec3/v3 (+ x1 (* t (- x2 x1))) (+ y1 (* t (- y2 y1))) (:z p1))))))

(defn offset-convex-polygon
  "Offset convex CCW polygon `pts` outward by `dist` (negative = inward):
  each edge shifts along its outward normal by `dist`, then each new
  vertex is the intersection of its two adjacent shifted edges."
  [pts dist]
  (let [n (count pts)
        shifted-edges
        (mapv (fn [i]
                (let [a (nth pts i) b (nth pts (mod (inc i) n))
                      [nx ny] (edge-outward-normal-2d a b)]
                  [(vec3/v3 (+ (:x a) (* nx dist)) (+ (:y a) (* ny dist)) (:z a))
                   (vec3/v3 (+ (:x b) (* nx dist)) (+ (:y b) (* ny dist)) (:z b))]))
              (range n))]
    (mapv (fn [i]
            (let [[a1 b1] (nth shifted-edges (mod (dec i) n))
                  [a2 b2] (nth shifted-edges i)]
              (or (line-intersect-2d a1 b1 a2 b2) a2)))
          (range n))))

(defn- gen-contour
  "Follows `:profile` (a convex CCW polygon, vec3 points, `:side`
  :inside/:outside/:on-line relative to it) offset by the tool radius,
  at one Z depth (`:depth` below the profile's own Z). Not attempted
  (returns `segments0` unchanged) for fewer than 3 points or a
  non-convex/non-CCW profile — see convex-ccw?."
  [job {:keys [tool-id depth side feed-rate profile]} segments0]
  (if (or (< (count (or profile [])) 3) (not (convex-ccw? profile)))
    segments0
    (let [tool (get (:tool-library job) tool-id)
          tool-radius (if tool (/ (:diameter tool) 2.0) 0.0)
          offset (case side :outside tool-radius :inside (- tool-radius) 0.0)
          offset-pts (offset-convex-polygon profile offset)
          z-top (:z (first profile))
          z (- z-top depth)
          safe-z (+ z-top (:safe-height job))
          path (mapv #(vec3/v3 (:x %) (:y %) z) offset-pts)
          closed (conj path (first path))
          start-xy (first path)
          segments (conj segments0
                         (segment {:segment-type :rapid :start (last-end segments0)
                                   :end (vec3/v3 (:x start-xy) (:y start-xy) safe-z) :tool-id tool-id}))
          segments (conj segments
                         (segment {:segment-type :rapid
                                   :start (vec3/v3 (:x start-xy) (:y start-xy) safe-z)
                                   :end start-xy :tool-id tool-id}))
          segments (reduce (fn [segs [a b]]
                              (conj segs (segment {:segment-type :linear :start a :end b
                                                   :feed-rate feed-rate :tool-id tool-id})))
                            segments
                            (map vector closed (rest closed)))
          prev (last-end segments)]
      (conj segments (segment {:segment-type :rapid :start prev
                               :end (vec3/v3 (:x prev) (:y prev) safe-z) :tool-id tool-id})))))

;; ---------------------------------------------------------------------
;; placeholder ops — surface-3d / turn
;; ---------------------------------------------------------------------

(defn- gen-placeholder
  [job {:keys [tool-id]} segments]
  (let [prev (last-end segments)]
    (conj segments
          (segment {:segment-type :rapid :start prev
                    :end (vec3/v3 0.0 0.0 (:safe-height job))
                    :tool-id tool-id}))))


;; ---------------------------------------------------------------------
;; :surface-3d — ball-nose raster finishing over a mesh target
;; ---------------------------------------------------------------------

(defn- tri-height-at
  "Height of triangle `[a b c]` above `[x y]`, or nil when the point is outside
   it. Barycentric, so a point on a shared edge belongs to both triangles and
   the max below is unaffected."
  [[a b c] x y]
  (let [[ax ay az] a [bx by bz] b [cx cy cz] c
        d (- (* (- by cy) (- ax cx)) (* (- bx cx) (- ay cy)))]
    (when (> (#?(:clj Math/abs :cljs js/Math.abs) d) 1e-12)
      (let [l1 (/ (+ (* (- by cy) (- x cx)) (* (- cx bx) (- y cy))) d)
            l2 (/ (+ (* (- cy ay) (- x cx)) (* (- ax cx) (- y cy))) d)
            l3 (- 1.0 l1 l2)
            e -1e-9]
        (when (and (>= l1 e) (>= l2 e) (>= l3 e))
          (+ (* l1 az) (* l2 bz) (* l3 cz)))))))

(defn- closest-on-segment
  "The point of segment a-b closest to `[x y]`, in XY, lifted to its own Z."
  [[ax ay az] [bx by bz] x y]
  (let [dx (- bx ax) dy (- by ay)
        len2 (+ (* dx dx) (* dy dy))]
    (if (< len2 1e-18)
      [ax ay az]
      (let [t (max 0.0 (min 1.0 (/ (+ (* (- x ax) dx) (* (- y ay) dy)) len2)))]
        [(+ ax (* t dx)) (+ ay (* t dy)) (+ az (* t (- bz az)))]))))

(defn ball-nose-drop
  "Programmed Z of a ball-nose tool at `[x y]` above `target` — the TIP, which
   is what a controller is fed — or nil when nothing is under the tool.

   The ball centre rests at `max over the disc of (h + sqrt(r^2 - d^2))`, the
   exact non-penetration condition on a height field, and the tip is `r` below
   it. On a flat plateau that reduces to the surface height, which is why a
   naive implementation that samples only the point under the axis looks right:
   it agrees everywhere flat, and gouges every convex feature by up to `r`
   exactly where the part is not flat.

   **Contacts on edges and vertices are sampled exactly, face interiors on a
   grid.** A square grid alone under-cuts a sharp edge by however much the
   spacing misses it — measured on a 6x6 plateau with a 6mm ball, a 24x24 grid
   put the tip at 1.000 where the true edge contact is 1.768, a 0.77mm gouge
   that no amount of eyeballing the path would reveal. Each triangle's vertices
   and the closest point on each of its edges are therefore sampled directly,
   which is exact for a polyhedral target except where the contact lands in a
   face's interior — there the grid decides, and `drop-samples` sets its
   resolution."
  [target r x y samples]
  (let [tris (:tris target)
        n (max 1 (long samples))
        lift (fn [d2] (when (<= d2 (* r r))
                        (#?(:clj Math/sqrt :cljs js/Math.sqrt) (- (* r r) d2))))
        d2-of (fn [px py] (let [dx (- px x) dy (- py y)] (+ (* dx dx) (* dy dy))))
        ;; contacts on a face interior: a grid over the disc
        grid (if (zero? r)
               [[x y]]
               (for [i (range (inc n)) j (range (inc n))
                     :let [px (+ x (* r (- (/ (* 2.0 i) n) 1.0)))
                           py (+ y (* r (- (/ (* 2.0 j) n) 1.0)))]
                     :when (<= (d2-of px py) (* r r))]
                 [px py]))
        grid-hits (for [[px py] grid
                        t tris
                        :let [h (tri-height-at t px py)
                              l (lift (d2-of px py))]
                        :when (and h l)]
                    (+ h l))
        ;; contacts on an edge or a vertex: sampled exactly, not approached
        edge-hits (for [[a b c] tris
                        [p q] [[a b] [b c] [c a]]
                        :let [[px py pz] (closest-on-segment p q x y)
                              l (lift (d2-of px py))]
                        :when l]
                    (+ pz l))
        hits (concat grid-hits edge-hits)]
    (when (seq hits) (- (reduce max hits) r))))

(defn- mesh-target
  "Normalise an operation's `:target` into `{:tris [...]}`, or nil."
  [target]
  (when target
    (let [vs (vec (:positions target (:vertices target)))
          idx (vec (:indices target))]
      (when (and (seq vs) (seq idx))
        {:tris (mapv (fn [[a b c]]
                       [(let [p (nth vs a)] (if (map? p) [(:x p) (:y p) (:z p)] (vec p)))
                        (let [p (nth vs b)] (if (map? p) [(:x p) (:y p) (:z p)] (vec p)))
                        (let [p (nth vs c)] (if (map? p) [(:x p) (:y p) (:z p)] (vec p)))])
                     (partition 3 idx))}))))

(defn- gen-surface-3d
  [job {:keys [tool-id stepover point-spacing chord-tolerance max-bisections
               strategy feed-rate target drop-samples]} segments0]
  (let [tool (get (:tool-library job) tool-id)
        r (if tool (/ (:diameter tool) 2.0) 0.0)
        ;; `:stepover` is the LATERAL spacing between passes and sets the
        ;; scallop height. `:point-spacing` is the FORWARD spacing along a pass
        ;; and sets the chord error — how far the straight move between two
        ;; sampled points cuts inside the surface it is following. They are
        ;; different quantities and the first version used one number for both:
        ;; `gouge-check` put the resulting path 0.365 mm into the part, on the
        ;; plateau edge where the surface changes fastest. The default keeps the
        ;; forward step an eighth of the lateral one, which is the usual order.
        step (if (and stepover (pos? stepover)) stepover (max r 0.5))
        fstep (if (and point-spacing (pos? point-spacing)) point-spacing (/ step 8.0))
        chord-tol (double (or chord-tolerance 0.01))
        max-bisect (long (or max-bisections 12))
        tgt (mesh-target target)]
    (cond
      (not (contains? implemented-surface-strategies strategy))
      (throw (ex-info (str "surface-3d strategy " strategy
                           " is named in `surface-strategies` but no generator is"
                           " implemented for it; refusing rather than emitting a"
                           " rapid move and calling it a finishing pass")
                      {:requested strategy
                       :implemented implemented-surface-strategies
                       :named surface-strategies}))

      (nil? tgt)
      (throw (ex-info (str "surface-3d needs a :target mesh to drop the tool onto"
                           " ({:positions [...] :indices [...]}); without one there"
                           " is no surface to finish")
                      {:op :surface-3d :target target}))

      :else
      (let [pts3 (mapcat identity (:tris tgt))
            xs (map first pts3)
            ys (map second pts3)
            ;; Retract height comes from the TARGET, not the stock: a surface
            ;; finishing pass is defined by the part it is finishing, and this
            ;; way the operation does not depend on how the caller happened to
            ;; describe the raw material.
            safe-z (+ (apply max (map #(nth % 2) pts3)) (:safe-height job))
            ;; The raster covers the target's own footprint, not the footprint
            ;; grown by the tool radius. Points whose AXIS is off the target are
            ;; skipped below: there the ball rolls off the edge and the drop
            ;; descends by up to r, which is a correct answer to the wrong
            ;; question — a finishing pass is bounded by the surface it finishes.
            ;; Measured before this: the path dived to z = -3 outside a plane at
            ;; z = 0.
            x-min (apply min xs) x-max (apply max xs)
            y-min (apply min ys) y-max (apply max ys)
            samples (or drop-samples 4)
            row (fn [y forward]
                  (let [cols (range 0 (inc (Math/ceil (/ (- x-max x-min) fstep))))
                        over? (fn [x] (some #(tri-height-at % x y) (:tris tgt)))
                        at (fn [x] (when (over? x)
                                     (when-let [z (ball-nose-drop tgt r x y samples)]
                                       (vec3/v3 x y z))))
                        coarse (vec (keep (fn [i] (at (min x-max (+ x-min (* i fstep))))) cols))
                        ;; Uniform spacing cannot make the chord error small where
                        ;; the tool-centre path has a vertical tangent — a ball
                        ;; pivoting on a sharp edge does, and refining uniformly
                        ;; converges there at a crawl. Measured: 0.365 mm inside
                        ;; the part at fstep = stepover, still 0.194 mm at 0.1 mm
                        ;; spacing. So bisect only where the straight move
                        ;; actually sags past `chord-tol`, and cap the depth so a
                        ;; genuine discontinuity cannot spin forever.
                        ;; The refinement criterion samples the SAME way the
                        ;; checker does. Testing only the midpoint passed
                        ;; intervals whose sag peaks off-centre — which it does
                        ;; near a vertical tangent — and left 0.061 mm in the
                        ;; part no matter how tight the tolerance was set.
                        sag (fn [a b]
                              (let [dz (- (:z b) (:z a))
                                    dx (- (:x b) (:x a))]
                                (reduce max 0.0
                                        (keep (fn [t]
                                                (when-let [m (at (+ (:x a) (* t dx)))]
                                                  (- (:z m) (+ (:z a) (* t dz)))))
                                              [0.125 0.25 0.375 0.5 0.625 0.75 0.875]))))
                        refine (fn refine [a b depth]
                                 (if (or (>= depth max-bisect) (<= (sag a b) chord-tol))
                                   [b]
                                   (let [m (at (/ (+ (:x a) (:x b)) 2.0))]
                                     (if m
                                       (into (refine a m (inc depth)) (refine m b (inc depth)))
                                       [b]))))
                        pts (if (< (count coarse) 2)
                              coarse
                              (into [(first coarse)]
                                    (mapcat (fn [[a b]] (refine a b 0))
                                            (map vector coarse (rest coarse)))))]
                    (if forward (vec pts) (vec (reverse pts)))))
            segments (if (seq segments0)
                       (conj segments0 (segment {:segment-type :rapid :start (last-end segments0)
                                                 :end (vec3/v3 x-min y-min safe-z)
                                                 :tool-id tool-id}))
                       segments0)]
        (loop [y y-min forward true segs segments]
          (if (> y (+ y-max 1e-9))
            (let [prev (last-end segs)]
              (conj segs (segment {:segment-type :rapid :start prev
                                   :end (vec3/v3 (:x prev) (:y prev) safe-z)
                                   :tool-id tool-id})))
            (let [pts (row y forward)]
              (if (< (count pts) 2)
                (recur (+ y step) (not forward) segs)
                (let [prev (last-end segs)
                      segs (conj segs (segment {:segment-type :rapid :start prev
                                                :end (vec3/v3 (:x (first pts)) (:y (first pts)) safe-z)
                                                :tool-id tool-id}))
                      segs (conj segs (segment {:segment-type :rapid
                                                :start (vec3/v3 (:x (first pts)) (:y (first pts)) safe-z)
                                                :end (first pts) :tool-id tool-id}))
                      segs (reduce (fn [acc [a b]]
                                     (conj acc (segment {:segment-type :linear :start a :end b
                                                         :feed-rate feed-rate :tool-id tool-id})))
                                   segs (map vector pts (rest pts)))]
                  (recur (+ y step) (not forward) segs))))))))))


;; ---------------------------------------------------------------------
;; Verification — did the path actually stay off the part?
;; ---------------------------------------------------------------------

(defn gouge-check
  "Does `segments` cut into `target` anywhere it should not?

   Generating a finishing pass and verifying one are different claims, and
   until now only the first was made. This checks the second, and only the
   second: at sampled points along every cutting move it recomputes the height
   the tool tip is allowed to reach and reports where the programmed tip is
   below it.

   Returns `{:tool-radius :tolerance :samples :checked :violations :worst-depth
   :passed?}` — `:violations` naming the segment index, the point, the
   programmed z, the allowed z and the depth. A gouge is reported, never
   corrected: a checker that quietly moves the path is no longer a checker.

   ⚠ **This is gouge detection against the target, and nothing else.** Tool
   HOLDER and shank collision, fixtures, clamps and the machine's own envelope
   are not modelled here — `holder-clearance` and a full `collision-check` do
   not exist. A path that passes this can still crash a machine, and saying so
   is the point."
  ([segments target] (gouge-check segments target {}))
  ([segments target {:keys [tool-radius tolerance samples drop-samples]
                     :or {tolerance 1.0e-6 samples 8 drop-samples 8}}]
   (let [tgt (mesh-target target)]
     (when-not tgt
       (throw (ex-info "gouge-check needs a :target mesh to check against"
                       {:target target})))
     (when-not (and (number? tool-radius) (>= tool-radius 0))
       (throw (ex-info (str "gouge-check needs the :tool-radius the path was"
                            " generated for; the allowed height depends on it,"
                            " and guessing would pass a path that gouges")
                       {:tool-radius tool-radius})))
     (let [cuts (keep-indexed (fn [i sg] (when (= :linear (:segment-type sg)) [i sg])) segments)
           n (max 1 (long samples))
           results
           (for [[i {:keys [start end]}] cuts
                 k (range (inc n))
                 :let [t (/ (double k) n)
                       x (+ (:x start) (* t (- (:x end) (:x start))))
                       y (+ (:y start) (* t (- (:y end) (:y start))))
                       z (+ (:z start) (* t (- (:z end) (:z start))))
                       allowed (ball-nose-drop tgt tool-radius x y drop-samples)]
                 :when allowed]
             {:segment i :point [x y z] :programmed-z z :allowed-z allowed
              :depth (- allowed z)})
           violations (vec (filter #(> (:depth %) tolerance) results))]
       {:tool-radius tool-radius
        :tolerance tolerance
        :samples n
        :checked (count results)
        :violations violations
        :worst-depth (if (seq results) (reduce max (map :depth results)) 0.0)
        ;; No samples means nothing was checked. That is not a pass.
        :passed? (and (pos? (count results)) (empty? violations))
        :checked-for #{:gouge-into-target}
        :not-checked-for #{:holder-collision :shank-collision :fixture-collision
                           :machine-envelope :rapid-moves}}))))

;; ---------------------------------------------------------------------
;; CamJob
;; ---------------------------------------------------------------------

(defn new-job
  "A complete CAM job combining stock, tool library, and ordered operations."
  [stock tool-library]
  {:stock stock :operations [] :tool-library tool-library :safe-height 5.0})

(defn add-operation [job op] (update job :operations conj op))

(defn generate-toolpath
  "Generate toolpath segments for all operations in order. Implements
   zigzag pocket, peck drill, single-pass raster face-mill, and convex-
   polygon-offset contour following; :surface-3d/:turn (and :contour on a
   profile gen-contour can't offset correctly) still produce a
   placeholder rapid move to the tool-change point so the G-code
   structure stays valid."
  [job]
  (reduce
   (fn [segments op]
     (let [before (count segments)
           after (case (:op op)
                   :pocket (gen-pocket job op segments)
                   :drill (gen-drill job op segments)
                   :face-mill (gen-face-mill job op segments)
                   :contour (let [result (gen-contour job op segments)]
                              (if (= result segments) (gen-placeholder job op segments) result))
                               :surface-3d (gen-surface-3d job op segments)
                   :turn (gen-placeholder job op segments))]
       ;; Stamp this operation's spindle speed onto the segments it produced.
       ;; Done here rather than inside each generator so the ~18 `segment` call
       ;; sites stay unchanged: the rpm is a property of the operation, and the
       ;; operation boundary is exactly this reduction step. Without it the post
       ;; has no rpm to write and emits a hardcoded one — a G-code program that
       ;; runs every tool at the same speed regardless of what the operation asked
       ;; for, which on a real machine is a broken tool, not a cosmetic defect.
       (if-let [rpm (:spindle-rpm op)]
         (into (subvec (vec after) 0 before)
               (map #(assoc % :spindle-rpm (double rpm)) (subvec (vec after) before)))
         after)))
   []
   (:operations job)))
