
(ros::load-ros-manifest "roseus")
(ros::roseus "grasp_object")

;;prepare to receive object
(send *pr2* :larm :collar-y :joint-angle -24)
(send *pr2* :larm :shoulder-p :joint-angle 25)
(send *pr2* :larm :shoulder-r :joint-angle 0)
(send *pr2* :larm :elbow-p :joint-angle -110)
(send *pr2* :larm :elbow-r :joint-angle 200)
(send *pr2* :larm :wrist-p :joint-angle -80)
(send *pr2* :larm :wrist-r :joint-angle 0)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
(send *ri* :stop-grasp)
;(speak-jp "ゴミがを捨ててあげるよxo")

(defun get_garbage_cb (msg)
  (setq *effort* (send msg :effort))
  (setq *name* (send msg :name))
  ;(print (elt *name* 39))  ;;35:l_elbow_flex_joint 39:l_gripper_finger
  (setq elbow-force (elt *effort* 35))
  (setq grasp-force (elt *effort* 39))
  (print grasp-force)
  (print grasped)
  (cond ((and (> elbow-force 1.5f) (eq  grasped nil)) ;;grasp if objects being handed
	 (progn (send *ri* :start-grasp :larm :gain 0.06) 
		(send *ri* :wait-interpolation)
		(setq grasped t)
		;(unix:sleep 2)
		))
	 ((and (eq grasped t) (> grasp-force -10.0f)) ;;open clip if failed to grasp firmly
	  (progn (speak-jp "やり直し")
	 	 (unix:sleep 8)
	 	 (send *ri* :stop-grasp :larm)
	 	 (send *ri* :wait-interpolation)
	 	 (setq grasped nil)
	 	 ))
	 ((and (eq grasped t) (< elbow-force -2.0f)) ;;release if objects being taken
	  (progn (send *ri* :stop-grasp :larm)
		 (send *ri* :wait-interpolation)
		 (setq grasped nil)
		 ))
	 )
  )

(ros::subscribe "/joint_states"
 sensor_msgs::JointState
 #'get_garbage_cb)


(progn
  (ros::ok)
  (do-until-key
   (ros::spin-once))
   (ros::sleep))
