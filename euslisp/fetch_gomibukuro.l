#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; about::pr2 fetches gomibukuro                                               ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(ros::roseus "fetch_gomibukuro")
(ros::load-ros-manifest "fetch_trashbag_demo")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "package://jsk_common_demo/pr2-move.l")

;;????????これでうまく移動するのか????????;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(send *ri* :move-to (send *scene* :spot "eng2/7f/room73B2-trashbox-front"))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(send *pr2* :reset-pose)
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

(send *ri* :start-grasp)
(send *ri* :wait-interpolation)