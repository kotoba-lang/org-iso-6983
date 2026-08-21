(ns kotoba.cam-test
  "Parity tests ported 1:1 from kami-cam (Rust) `src/tests.rs`. Test names
   and assertions mirror the originals so this file is directly diffable
   against the recovered Rust source (see README for the recovery command)."
  (:require [clojure.test :refer [deftest is testing]]
            [clojure.string :as str]
            [kotoba.cam.gcode :as gcode]
            [kotoba.cam.stock :as stock]
            [kotoba.cam.tool :as tool]
            [kotoba.cam.toolpath :as toolpath]
            [kotoba.cam.util :as util]
            [kotoba.cam.vec3 :as vec3]))

(defn- sample-endmill []
  {:id 1
   :name "6mm 2-flute carbide"
   :tool-type :end-mill
   :diameter 6.0
   :flute-length 20.0
   :overall-length 50.0
   :flute-count 2
   :corner-radius 0.0
   :material :carbide
   :coating "TiAlN"})

;; -----------------------------------------------------------------------
;; 1. Tool library CRUD (Rust: tool_library_crud)
;; -----------------------------------------------------------------------
(deftest tool-library-crud
  (let [lib (tool/empty-library)]
    (is (tool/empty-lib? lib))

    ;; Add
    (let [t1 (sample-endmill)
          [lib prev] (tool/add lib t1)]
      (is (nil? prev))
      (is (= 1 (tool/lib-count lib)))

      ;; Get
      (let [fetched (tool/get-tool lib 1)]
        (is (= "6mm 2-flute carbide" (:name fetched)))
        (is (= :end-mill (:tool-type fetched))))

      ;; Replace
      (let [t1-v2 (assoc (sample-endmill) :name "6mm 3-flute carbide" :flute-count 3)
            [lib old] (tool/add lib t1-v2)]
        (is (= 2 (:flute-count old)))
        (is (= 3 (:flute-count (tool/get-tool lib 1))))

        ;; Add second tool
        (let [t2 {:id 2 :name "10mm ball nose" :tool-type :ball-nose
                  :diameter 10.0 :flute-length 25.0 :overall-length 75.0
                  :flute-count 2 :corner-radius 5.0 :material :hss :coating nil}
              [lib _] (tool/add lib t2)]
          (is (= 2 (tool/lib-count lib)))

          ;; List sorted by id
          (let [l (tool/list-tools lib)]
            (is (= 1 (:id (first l))))
            (is (= 2 (:id (second l)))))

          ;; Remove
          (let [[lib removed] (tool/remove-tool lib 1)]
            (is (= 1 (:id removed)))
            (is (= 1 (tool/lib-count lib)))
            (is (nil? (tool/get-tool lib 1)))))))))

;; -----------------------------------------------------------------------
;; 2. G-code header and footer validity (Rust: gcode_header_footer_valid)
;; -----------------------------------------------------------------------
(deftest gcode-header-footer-valid
  (let [segments [(toolpath/segment {:segment-type :rapid
                                      :start vec3/zero
                                      :end (vec3/v3 10.0 0.0 5.0)
                                      :feed-rate 0.0
                                      :tool-id 1})
                  (toolpath/segment {:segment-type :linear
                                      :start (vec3/v3 10.0 0.0 5.0)
                                      :end (vec3/v3 10.0 0.0 -2.0)
                                      :feed-rate 500.0
                                      :tool-id 1})]
        config gcode/default-config
        g (gcode/generate-gcode segments config)]

    (testing "header"
      (is (str/starts-with? g "%"))
      (is (str/includes? g "O0001"))
      (is (str/includes? g "G21"))
      (is (str/includes? g "G90"))
      (is (str/includes? g "G54")))

    (testing "tool change"
      (is (str/includes? g "T01 M06")))
    (testing "spindle"
      (is (str/includes? g "M03")))
    (testing "coolant"
      (is (str/includes? g "M08")))

    (testing "motion"
      (is (str/includes? g "G00"))
      (is (str/includes? g "G01"))
      (is (str/includes? g "F500.0")))

    (testing "footer"
      (is (str/includes? g "M05"))
      (is (str/includes? g "M09"))
      (is (str/includes? g "M30"))
      (is (str/ends-with? (str/trim g) "%")))))

;; -----------------------------------------------------------------------
;; 3. G-code arc output (G02/G03) (Rust: gcode_arc_output)
;; -----------------------------------------------------------------------
(deftest gcode-arc-output
  (let [segments [(toolpath/segment {:segment-type :arc-cw
                                      :start (vec3/v3 10.0 0.0 -1.0)
                                      :end (vec3/v3 0.0 10.0 -1.0)
                                      :feed-rate 300.0
                                      :center (vec3/v3 0.0 0.0 -1.0)
                                      :tool-id 1})
                  (toolpath/segment {:segment-type :arc-ccw
                                      :start (vec3/v3 0.0 10.0 -1.0)
                                      :end (vec3/v3 10.0 0.0 -1.0)
                                      :feed-rate 300.0
                                      :center (vec3/v3 0.0 0.0 -1.0)
                                      :tool-id 1})]
        g (gcode/generate-gcode segments gcode/default-config)]
    (is (str/includes? g "G02"))
    (is (str/includes? g "G03"))
    ;; I/J values: center - start
    (is (str/includes? g "I-10.0000"))
    (is (str/includes? g "J0.0000"))))

;; -----------------------------------------------------------------------
;; 4. Pocket toolpath generates zigzag segments (Rust: pocket_toolpath_generates_segments)
;; -----------------------------------------------------------------------
(deftest pocket-toolpath-generates-segments
  (let [[lib _] (tool/add (tool/empty-library) (sample-endmill))
        s (stock/stock (stock/block 100.0 100.0 20.0) (stock/aluminum-6061))
        job (-> (toolpath/new-job s lib)
                (toolpath/add-operation
                 {:op :pocket
                  :tool-id 1
                  :depth 3.0
                  :stepover 3.0
                  :strategy :zigzag
                  :feed-rate 800.0
                  :spindle-rpm 12000.0
                  :pocket-min (vec3/v3 10.0 10.0 0.0)
                  :pocket-max (vec3/v3 50.0 50.0 0.0)}))
        segments (toolpath/generate-toolpath job)]

    (testing "non-trivial segment count"
      (is (> (count segments) 5)
          (str "expected many segments, got " (count segments))))

    (testing "has both rapids and linear cuts"
      (is (some #(= :rapid (:segment-type %)) segments))
      (is (some #(= :linear (:segment-type %)) segments)))

    (testing "all segments reference tool 1"
      (is (every? #(= 1 (:tool-id %)) segments)))

    (testing "linear segments have the correct feed rate"
      (doseq [seg segments]
        (when (= :linear (:segment-type seg))
          (let [d (- (:feed-rate seg) 800.0)]
            (is (< (if (neg? d) (- d) d) 1e-6))))))

    (testing "G-code round-trip: segments produce valid G-code"
      (let [g (gcode/generate-gcode segments gcode/default-config)]
        (is (str/includes? g "G01"))
        (is (str/includes? g "M30"))))))

;; -----------------------------------------------------------------------
;; 4b. Face-mill: single-pass raster over the stock's top face (new --
;;     the original Rust crate only had a placeholder for this op)
;; -----------------------------------------------------------------------
(deftest face-mill-rasters-the-stock-top-face
  (let [[lib _] (tool/add (tool/empty-library) (sample-endmill))
        s (stock/stock (stock/block 100.0 60.0 20.0) (stock/aluminum-6061))
        job (-> (toolpath/new-job s lib)
                (toolpath/add-operation
                 {:op :face-mill :tool-id 1 :depth-of-cut 1.0
                  :stepover 4.0 :feed-rate 900.0 :spindle-rpm 10000.0}))
        segments (toolpath/generate-toolpath job)
        linear (filter #(= :linear (:segment-type %)) segments)]
    (testing "non-trivial linear cut coverage across the stock's Y extent"
      (is (> (count linear) 5))
      (is (every? #(= 1 (:tool-id %)) segments)))
    (testing "all cuts are at stock-top minus depth-of-cut"
      (doseq [seg linear]
        (is (< (Math/abs (- (get-in seg [:start :z]) 19.0)) 1e-6))))
    (testing "cuts span (approximately) the stock's full X width"
      (is (some #(<= (get-in % [:start :x]) 3.0) linear))
      (is (some #(>= (get-in % [:end :x]) 97.0) linear)))
    (testing "G-code round-trip"
      (let [g (gcode/generate-gcode segments gcode/default-config)]
        (is (str/includes? g "G01"))
        (is (str/includes? g "M30"))))))

;; -----------------------------------------------------------------------
;; 4c. Contour: convex-polygon offset following (new -- the original Rust
;;     crate only had a placeholder for this op)
;; -----------------------------------------------------------------------
(deftest contour-offsets-a-square-outward
  (testing "offsetting a unit square outward by the tool radius (3mm) grows
            each side by 2x the offset -- verified against the analytic result"
    (let [square [(vec3/v3 0.0 0.0 0.0) (vec3/v3 10.0 0.0 0.0)
                  (vec3/v3 10.0 10.0 0.0) (vec3/v3 0.0 10.0 0.0)]]
      (is (toolpath/convex-ccw? square))
      (let [offset (toolpath/offset-convex-polygon square 3.0)]
        (is (every? #(< (Math/abs (- (:x %) -3.0)) 1e-9)
                    [(nth offset 0) (nth offset 3)]))
        (is (every? #(< (Math/abs (- (:x %) 13.0)) 1e-9)
                    [(nth offset 1) (nth offset 2)]))))))

(deftest contour-toolpath-follows-outside-offset
  (let [[lib _] (tool/add (tool/empty-library) (sample-endmill))
        s (stock/stock (stock/block 100.0 100.0 20.0) (stock/aluminum-6061))
        square [(vec3/v3 10.0 10.0 20.0) (vec3/v3 40.0 10.0 20.0)
                (vec3/v3 40.0 40.0 20.0) (vec3/v3 10.0 40.0 20.0)]
        job (-> (toolpath/new-job s lib)
                (toolpath/add-operation
                 {:op :contour :tool-id 1 :depth 2.0 :side :outside
                  :feed-rate 700.0 :spindle-rpm 9000.0 :profile square}))
        segments (toolpath/generate-toolpath job)
        linear (filter #(= :linear (:segment-type %)) segments)]
    (testing "closed loop: 4 linear edges for a 4-point square profile"
      (is (= 4 (count linear))))
    (testing "cuts at stock-top (20.0) minus depth (2.0)"
      (is (every? #(< (Math/abs (- (get-in % [:start :z]) 18.0)) 1e-6) linear)))
    (testing ":outside offsets past the profile by the 3mm tool radius"
      (is (some #(< (get-in % [:start :x]) 10.0) linear)))
    (testing "G-code round-trip"
      (let [g (gcode/generate-gcode segments gcode/default-config)]
        (is (str/includes? g "G01"))
        (is (str/includes? g "M30"))))))

(deftest contour-rejects-concave-profile-falls-back-to-placeholder
  (testing "a concave/non-CCW profile isn't offset (would self-intersect) --
            gen-contour declines and generate-toolpath falls back to the
            same placeholder rapid the crate uses for unimplemented ops,
            not a silently wrong toolpath"
    (let [[lib _] (tool/add (tool/empty-library) (sample-endmill))
          s (stock/stock (stock/block 100.0 100.0 20.0) (stock/aluminum-6061))
          concave [(vec3/v3 0.0 0.0 20.0) (vec3/v3 10.0 0.0 20.0)
                   (vec3/v3 5.0 5.0 20.0) (vec3/v3 10.0 10.0 20.0)
                   (vec3/v3 0.0 10.0 20.0)]
          job (-> (toolpath/new-job s lib)
                  (toolpath/add-operation
                   {:op :contour :tool-id 1 :depth 2.0 :side :outside
                    :feed-rate 700.0 :profile concave}))
          segments (toolpath/generate-toolpath job)]
      (is (not (toolpath/convex-ccw? concave)))
      (is (empty? (filter #(= :linear (:segment-type %)) segments)))
      (is (some #(= :rapid (:segment-type %)) segments)))))

;; -----------------------------------------------------------------------
;; 5. Material presets (Rust: material_presets)
;; -----------------------------------------------------------------------
(deftest material-presets
  (let [al (stock/aluminum-6061)]
    (is (< 2.0 (:density al) 3.0))
    (is (> (:hardness al) 50.0)))

  (let [ti (stock/titanium-ti6al4v)]
    (is (> (:density ti) 4.0))
    (is (> (:hardness ti) 300.0)))

  (let [wood (stock/wood-oak)]
    (is (< (:density wood) 1.0))))

;; ---------------------------------------------------------------------
;; Regression: the post used to write `TNaN M06` and a hardcoded spindle
;; speed. Neither showed up in the Rust-parity tests above, because the
;; Rust original keyed its tool library by integer and never carried a
;; per-operation rpm into the post. Both were found by a behaviour probe
;; that asserted properties of the emitted program rather than the shape
;; of the call (com-junkawasaki/root, 90-docs/maturity/probes/gcode.cljs).
;; ---------------------------------------------------------------------

(defn- two-op-job []
  (let [[lib _] (tool/add (tool/empty-library)
                          {:id :em6 :name "6mm end mill" :tool-type :end-mill
                           :diameter 6.0 :flute-length 20.0 :overall-length 60.0
                           :flute-count 4 :corner-radius 0.0 :material :carbide})
        [lib _] (tool/add lib
                          {:id :d5 :name "5mm drill" :tool-type :drill
                           :diameter 5.0 :flute-length 30.0 :overall-length 70.0
                           :flute-count 2 :corner-radius 0.0 :material :carbide})]
    (-> (toolpath/new-job (stock/block 100 100 20) lib)
        (toolpath/add-operation
         {:op :pocket :tool-id :em6 :depth 3.0 :stepover 2.0 :strategy :zigzag
          :feed-rate 400.0 :spindle-rpm 8000
          :pocket-min (vec3/v3 10.0 10.0 0.0) :pocket-max (vec3/v3 60.0 40.0 0.0)})
        (toolpath/add-operation
         {:op :drill :tool-id :d5 :depth 10.0 :peck-depth 3.0
          :feed-rate 120.0 :spindle-rpm 2500 :holes [(vec3/v3 20.0 20.0 0.0)]}))))

(deftest non-integer-tool-ids-get-real-tool-numbers
  (testing "a keyword tool id becomes T01/T02 in first-appearance order"
    (let [segments (toolpath/generate-toolpath (two-op-job))
          text (gcode/generate-gcode segments)]
      (is (not (str/includes? text "NaN"))
          "a controller rejects TNaN; the generator must not report success while writing it")
      (is (str/includes? text "T01 M06 (tool change: :em6)"))
      (is (str/includes? text "T02 M06 (tool change: :d5)"))))

  (testing "integer tool ids keep their own numbers"
    (is (= {3 3, 7 7} (gcode/tool-numbers [{:tool-id 3} {:tool-id 7} {:tool-id 3}]))))

  (testing "assignment is first-appearance order and stable"
    (is (= {:b 1, :a 2} (gcode/tool-numbers [{:tool-id :b} {:tool-id :a} {:tool-id :b}]))))

  (testing "pad-int refuses a non-number instead of formatting NaN"
    (is (= "07" (util/pad-int 7 2)))
    (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                 (util/pad-int :em6 2)))))

(deftest spindle-speed-comes-from-the-operation
  (let [segments (toolpath/generate-toolpath (two-op-job))
        text (gcode/generate-gcode segments)]
    (testing "each operation's own rpm reaches the post"
      (is (str/includes? text "M03 S8000 (spindle CW)"))
      (is (str/includes? text "M03 S2500 (spindle CW)")))
    (testing "segments carry the rpm of the operation that produced them"
      (is (= #{8000.0 2500.0} (set (map :spindle-rpm segments)))))
    (testing "the fallback is named in the config and announced in the program"
      (let [bare (gcode/generate-gcode [{:segment-type :rapid :tool-id 1
                                         :start (vec3/v3 0.0 0.0 0.0)
                                         :end (vec3/v3 1.0 0.0 0.0) :feed-rate 0.0}])]
        (is (str/includes? bare (str "M03 S" (:default-spindle-rpm gcode/default-config))))
        (is (str/includes? bare "no spindle speed on this operation"))))))

(deftest unimplemented-post-processors-are-refused-not-faked
  (testing "the named set is larger than the implemented set, and says so"
    (is (contains? gcode/post-processors :heidenhain))
    (is (not (contains? gcode/implemented-post-processors :heidenhain)))
    (is (contains? gcode/implemented-post-processors :fanuc)))

  (let [segments (toolpath/generate-toolpath (two-op-job))]
    (testing "the implemented target emits"
      (is (str/includes? (gcode/generate-gcode segments gcode/default-config) "M30")))
    (testing "an unimplemented target throws instead of returning Fanuc output under its name"
      (doseq [post (disj gcode/post-processors :fanuc)]
        (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                     (gcode/generate-gcode segments (assoc gcode/default-config
                                                           :post-processor post)))
            (str post " must be refused while its dialect is unimplemented"))))))

;; ---------------------------------------------------------------------
;; :surface-3d — ball-nose raster finishing (2026-08-22)
;;
;; Before this, `:surface-3d` returned a rapid move to the tool-change point
;; for every one of the four strategies `surface-strategies` names. A job that
;; looks generated and removes nothing is worse than an error: it reaches the
;; machine.
;; ---------------------------------------------------------------------

(def ^:private plateau-target
  ;; a 6x6 plateau at z=4 sitting on a plane at z=0
  {:positions [[-20 -20 0] [20 -20 0] [20 20 0] [-20 20 0]
               [-3 -3 4] [3 -3 4] [3 3 4] [-3 3 4]]
   :indices [0 1 2 0 2 3 4 5 6 4 6 7]})

(defn- tris-of [{:keys [positions indices]}]
  {:tris (mapv (fn [[a b c]] [(nth positions a) (nth positions b) (nth positions c)])
               (partition 3 indices))})

(deftest ball-nose-drop-is-exact-on-edges
  ;; The ball centre rests at max over the disc of (h + sqrt(r^2 - d^2)); the
  ;; tip is r below it. On flat ground that reduces to the surface height, which
  ;; is why sampling only the point under the axis LOOKS right — it agrees
  ;; everywhere flat and gouges every convex feature by up to r.
  ;;
  ;; A grid alone is not enough either. Measured with a 24x24 grid, x=5.9 came
  ;; out at 1.000 against a true 1.768: a 0.77mm gouge into the plateau edge
  ;; that no amount of looking at the path would show. Edges and vertices are
  ;; now sampled directly.
  (let [t (tris-of plateau-target) r 3.0]
    (doseq [[x expected] [[0.0 4.0]        ; on the plateau
                          [2.9 4.0]        ; still on it
                          [4.0 3.828]      ; riding the edge, d=1
                          [5.0 3.236]      ; d=2
                          [5.9 1.768]      ; d=2.9 — the case a grid misses
                          [6.5 0.0]        ; out of reach, back on the plane
                          [10.0 0.0]]]
      (testing (str "x=" x)
        (is (< (Math/abs (- (toolpath/ball-nose-drop t r x 0.0 12) expected)) 0.002))))

    (testing "nothing under the tool is nil, not zero"
      (is (nil? (toolpath/ball-nose-drop {:tris []} r 0.0 0.0 12))))))

(deftest surface-3d-cuts-and-follows-the-target
  (let [[lib _] (tool/add (tool/empty-library)
                          {:id :bn6 :name "6mm ball" :tool-type :ball-nose :diameter 6.0
                           :flute-length 20.0 :overall-length 60.0 :flute-count 2
                           :corner-radius 3.0 :material :carbide})
        job (fn [strategy] (-> (toolpath/new-job (stock/stock (stock/block 60 60 20)
                                                              (stock/aluminum-6061)) lib)
                               (toolpath/add-operation
                                {:op :surface-3d :tool-id :bn6 :stepover 4.0
                                 :strategy strategy :feed-rate 1200.0 :spindle-rpm 12000
                                 :target plateau-target})))
        segments (toolpath/generate-toolpath (job :raster))
        cuts (filter #(= :linear (:segment-type %)) segments)]

    (testing "it cuts — this is the whole point"
      (is (> (count cuts) 20)))

    (testing "every cutting move carries the operation's feed rate"
      (is (every? #(= 1200.0 (:feed-rate %)) cuts)))

    (testing "the path follows the target: it reaches the plateau and the floor"
      (let [zs (map #(:z (:end %)) cuts)]
        (is (< (Math/abs (- (apply max zs) 4.0)) 1.0e-6))
        (is (< (Math/abs (apply min zs)) 1.0e-6))))

    (testing "and never dips below the floor it is finishing"
      (is (every? #(> (:z (:end %)) -1.0e-9) cuts)))))

(deftest unimplemented-surface-strategies-are-refused-not-faked
  (testing "the named set is larger than the implemented set, and says so"
    (is (contains? toolpath/surface-strategies :waterline))
    (is (not (contains? toolpath/implemented-surface-strategies :waterline)))
    (is (contains? toolpath/implemented-surface-strategies :raster)))

  (let [[lib _] (tool/add (tool/empty-library)
                          {:id :bn6 :name "6mm ball" :tool-type :ball-nose :diameter 6.0
                           :flute-length 20.0 :overall-length 60.0 :flute-count 2
                           :corner-radius 3.0 :material :carbide})
        job (fn [op] (-> (toolpath/new-job (stock/stock (stock/block 60 60 20)
                                                        (stock/aluminum-6061)) lib)
                         (toolpath/add-operation op)))]
    (testing "a strategy with no generator throws instead of emitting a rapid"
      (doseq [s (disj toolpath/surface-strategies :raster)]
        (is (thrown? #?(:clj clojure.lang.ExceptionInfo :cljs js/Error)
                     (toolpath/generate-toolpath
                      (job {:op :surface-3d :tool-id :bn6 :stepover 4.0 :strategy s
                            :feed-rate 1200.0 :target plateau-target})))
            (str s " must be refused while it has no generator"))))

    (testing "and a surface pass with no target is refused too"
      (is (thrown-with-msg?
           #?(:clj clojure.lang.ExceptionInfo :cljs js/Error) #":target"
           (toolpath/generate-toolpath
            (job {:op :surface-3d :tool-id :bn6 :stepover 4.0 :strategy :raster
                  :feed-rate 1200.0})))))))
