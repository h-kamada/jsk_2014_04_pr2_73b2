#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;about: pr2 giving someone trashbag                                                                   
;;when pr2 feels her left shoulder is pushed,                                                                              
;;she opens her left hand and says "yorosiku" 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;                                                                                                                ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "give_someone_something")
(ros::roseus "give_someone_something")
(require :pr2-interface "package://pr2eus/pr2-interface.l")

(defclass string-cb-class
  :super propertied-object
  :slots ())

(defmethod string-cb-class
  (:init () (ros::subscribe "joint_states" sensor_msgs::JointState #'send self 
:string-cb))
  (:string-cb (msg)
	      (progn (print (setq effort (elt (send msg :effort) 32)))
;;左肩の値


                     (if (> effort 2.0)                                      
                         (progn                                               
                           (send *ri* :stop-grasp :arms)                      
			   (send *ri* :wait-interpolation)                    
			   (send *pr2* :torso :waist-z :joint-angle 50)
			   (send *pr2* :larm :collar-y :joint-angle 46.6521)
			   (send *pr2* :larm :shoulder-p :joint-angle 28.2833)
			   (send *pr2* :larm :shoulder-r :joint-angle 39.6189)
			   (send *pr2* :larm :elbow-p :joint-angle -103.725)
			   (send *pr2* :larm :elbow-r :joint-angle 6.83664)
			   (send *pr2* :larm :wrist-p :joint-angle -22.2449)
			   (send *pr2* :larm :wrist-r :joint-angle 178.785)
			   (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
			   (send *ri* :wait-interpolation)
			   (speak-jp "よろしくね")
			   (send *pr2* :reset-pose)
			   (send *ri* :angle-vector (send *pr2* :angle-vector))
                           )))))                                              

(setq m (instance string-cb-class :init))
(do-until-key
 (ros::spin-once))
