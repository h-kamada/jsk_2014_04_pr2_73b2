#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; about: pr2 interacting with people

;; If someone twist pr2's left shoulder,

;; pr2 says "Thank you" and release his tray
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "roseus")
(ros::roseus "intearaction_demo")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defclass string-cb-class
  :super propertied-object
  :slots ())

(defmethod string-cb-class
  (:init () (ros::subscribe "joint_states" sensor_msgs::JointState #'send self :string-cb))
  (:string-cb (msg) 
	            (progn (print (setq effort (elt (send msg :effort ) 32)))
			   (if (> effort 1.5)
			       (progn 
				 (speak-jp "ありがとう")     				                        (send *ri* :stop-grasp :arms)
				 )))))
			     
			   
(setq m (instance string-cb-class :init))
(do-until-key
(ros::spin-once)) 
