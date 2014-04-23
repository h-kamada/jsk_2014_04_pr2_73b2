(load "models/room73b2-scene.l")
(load "package://pr2eus/pr2-interface.l")
(load "package://jsk_demo_common/euslisp/pr2-move.l")
(setq *room* (room73b2))

(pr2)
(pr2-init)
(objects (list *pr2* *room*))
(send *irtviewer* :look-all 
      (geo::make-bounding-box
       (flatten (send-all (send *pr2* :bodies) :vertices))))


(send *pr2* :reset-pose)
;(pr2-reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *pr2* :move-to (send *room* :spot "cook-spot") :world)
;(send *ri* :go-pos 0 -0.04 0) 
;(send *pr2* :translate (float-vector -300 0 0))
;(send *pr2* :translate #f(200 0 0))
;(move-to-spot "/eng2/7f/room73B2-trashbox-front" :clear-costmap nil)
;(send *pr2* :arms :shoulder-p :joint-angle 30)
;(send *pr2* :rarm :move-end-pos #f(0 170 40) :world :rotation-axis nil)
;(send *ri* :wait-interpolation)


(send *ri* :angle-vector  #f(50.0207 33.1095 74.2513 59.9813 -67.2523 -27.8703 -5.72958 377.546 -22.9158 57.9269 0.113247 -116.905 0.001316 -5.74938 359.997 1.49389 -9.96) 2000)
(send *pr2* :angle-vector (send *ri* :state :potentio-vector) )
(send *ri* :wait-interpolation)
;(send *pr2* :larm :shoulder-p :joint-angle 74)
;(send *pr2* :larm :shoulder-r :joint-angle 70)
;(send *pr2* :larm :elbow-p :joint-angle -60)
;(send *pr2* :larm :elbow-r :joint-angle 20)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)

(send *pr2* :larm :move-end-pos #f(100 0 0) *pr2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)
;(send *ri* :state :potentio-vector)
;(send *pr2* :state :angle-vector)
;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))

(send *ri* :start-grasp :larm)

(send *pr2* :larm :move-end-pos #f(0 0 100) *pr2*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :go-pos -0.4 0 0) 
