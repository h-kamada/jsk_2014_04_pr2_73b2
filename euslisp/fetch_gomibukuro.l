#!/usr/bin/env roseus

;;今はここにやりたい動作を全部書いている
;;ゴミ袋のある棚まで移動　ゴミ袋取る　お盆のある机の上まで移動　ゴミ袋をお盆に載せる　お盆をつかむ　顔認識で誰かを見つける　話しかける　人がゴミ袋入れてPR2の左肩を内向きに押す　ありがとうと言ってお盆を離す　　

;;わからないところ
;;移動の関数　お盆の認識（ゴミ袋を落とすところ）ありがとうと言うのを一回だけにしたい

;;動作確認できなかったところ
;;pick-tray 顔認識　（カメラが写らなかったため）
;;それにより全体の通し動作確認できず

;;時間があればやりたいこと
;;ゴミ袋の認識（はじめにつかむところ）PR2に絵（線でもいい）を描かせたい　NAO
(load "package://pr2eus/pr2-interface.l")
(load "models/room73b2-scene.l")
(ros::roseus "pr2_send_joints")
(ros::load-ros-manifest "fetch_trashbag_demo")

(pr2-init)
(setq *room* (room73b2))

(objects (list *pr2* *room*))
(send *pr2* :locate #f(0 0 550))
;;(send *irtviewer* :look-all (geo::make-bounding-box (flatten (send-all (send *pr2* :bodies) :vertices))))

(ros::roseus "pr2_send_joints")

(pr2)
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
 


;;棚に移動 ゴミ袋をつまむ
;;(send *pr2* :move-to (send *room* :spot "cook-spot" ) :world)
;;(send *pr2* :locate #f(50 2700 0))



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

;;ちょっと近づいてつかむ


(send *ri* :start-grasp :arms )


;;机に移動 お盆をとる
;;(send *pr2* :locate #f(-3800 -2500 0))



;;お盆にゴミ袋を置く
(send *pr2* :torso :waist-z :joint-angle 50.0175)
(send *pr2* :larm :collar-y :joint-angle -9.88456)
(send *pr2* :larm :shoulder-p :joint-angle 23.9643)
(send *pr2* :larm :shoulder-r  :joint-angle  0.583074)
(send *pr2* :larm :elbow-p :joint-angle -75.348)
(send *pr2* :larm :elbow-r :joint-angle 195.616)
(send *pr2* :larm :wrist-p :joint-angle -113.611)
(send *pr2* :larm :wrist-r :joint-angle -2.01249)
(send *ri* :stop-grasp :arms)
(send *ri* :angle-vector (send *pr2* :angle-vector ) 3000)

(send *ri* :wait-interpolation)

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)


;;お盆をつかむ

;;(load "package://jsk_demo_common/euslisp/pr2-action.l")
;;(pick-tray)

;;お盆を持って移動する
;;(send *ri* :move-pos-unsafe     )

;;人に渡す(顔認識を使う)
(load "package://face_detect_demo/euslisp/face_detect_listener.l")


;;人がゴミ袋を入れてくれる pr2の左肩をひねって合図をする
;;(load "package://interaction_demo/interaction_demo.l)

(ros::load-ros-manifest "roseus")
(ros::roseus "listener2")

;;(setq *ri* (instance pr2-interface :init))

(defclass string-cb-class
  :super propertied-object
  :slots ())

(defmethod string-cb-class
  (:init () (ros::subscribe "joint_states" sensor_msgs::JointState #'send self :string-cb))
  (:string-cb (msg) 
	      (progn (print (setq effort (elt (send msg :effort ) 32)))
		     (if (> effort 1.0 ) 
;;(progn
;;(send *ri* :stop-grasp :larm)
;;(send *pr2* :larm :collar-y :joint-angle 60)
;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 3000)
;;)))))
			(progn (speak-jp "ありがとう")
			       (send *ri* :stop-grasp :arms))
))))
(setq m (instance string-cb-class :init))
(do-until-key
(ros::spin-once)) 
