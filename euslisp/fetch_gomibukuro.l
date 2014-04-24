#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; about::pr2 fetches gomibukuro
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-scene.l")
(ros::roseus "pr2_send_joints")
(ros::load-ros-manifest "fetch_trashbag_demo")

(pr2-init)
(setq *room* (room73b2))

(objects (list *pr2* *room*))
(send *pr2* :locate #f(0 0 550))
;;(send *irtviewer* :look-all (geo::make-bounding-box (flatten (send-all (send *pr2* :bodies) :vertices))))

(when (not (boundp '*irtviewer*))
  (make-irtviewer)
  (objects (list *pr2*))
  (send *irtviewer* :draw-objects))

(setq *ri* (instance pr2-interface :init))

(send *pr2* :reset-pose)
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))

(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :stop-grasp)
(send *ri* :wait-interpolation)
 

(send *pr2* :torso :waist-z :joint-angle 50.096)
(send *pr2* :larm :collar-y :joint-angle 17.5336)
(send *pr2* :larm :shoulder-p :joint-angle -20.1088)
(send *pr2* :larm :shoulder-r :joint-angle 36.3231)
(send *pr2* :larm :elbow-p :joint-angle -29.3121)
(send *pr2* :larm :elbow-r :joint-angle 171.85)
(send *pr2* :larm :wrist-p :joint-angle -46.7167)
(send *pr2* :larm :wrist-r :joint-angle 237.294)

(send *pr2* :head :neck-p :joint-angle 15)


(send *ri* :angle-vector (send *pr2* :angle-vector ) 3000)
(send *ri* :wait-interpolation)
