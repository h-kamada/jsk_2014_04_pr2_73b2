#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; about: pr2 interacting with people

;; If someone twist pr2's left shoulder,

;; pr2 says "Thank you" and takes a bow
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
				 (send *pr2* :reset-pose)
				 (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
				 (send *ri* :wait-interpolation)
				
				 (send *pr2* :larm :collar-y :joint-angle 70)
				 (send *pr2* :rarm :collar-y :joint-angle -70)
				 (send *pr2* :larm :shoulder-r :joint-angle 100)
				 (send *pr2* :rarm :shoulder-r :joint-angle -100)
				 (send *pr2* :head :neck-p :joint-angle 30)
				 (send *ri* :angle-vector (send *pr2* :angle-vector))
				 (send *ri* :wait-interpolation)
				 (speak-jp "ありがとう")     				                        
				 )))))		     
			   
(setq m (instance string-cb-class :init))
(do-until-key
(ros::spin-once)) 
