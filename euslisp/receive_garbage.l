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
  (if (eq (boundp 'grasped) nil) (setq grasped nil))
  (print grasped)
  (cond ((and (> elbow-force 1.5f) (eq  grasped nil)) ;;grasp if objects being handed
	 (progn (send *ri* :start-grasp :larm :gain 0.06) 
		;; (send *ri* :wait-interpolation)
		(setq grasped t)
		(setq time_g (unix:runtime))
		))
	((and (eq grasped t) (> grasp-force -10.0f)) ;;open clip if failed to grasp firmly
	 (progn (print "やり直し")
					;(unix:sleep 8)
		(send *ri* :stop-grasp :larm :wait t)
		;; (send *ri* :wait-interpolation)
		(setq grasped nil)
		))
	((and (eq grasped t) (< elbow-force -2.0f)) ;;release if objects being taken
	 (progn (send *ri* :stop-grasp :larm :wait t)
		;; (send *ri* :wait-interpolation)
		(setq grasped nil)
		))
	((and (eq grasped t) (> (- (unix:runtime) time_g) 600))
	 (print "移動します！")
	 (setq receive-coord (send *ri* :state :worldcoords))
					;(move-to-spot "eng2/7f/room73b2-trashbox-front")
	 (send *ri* :go-pos-unsafe 0.4 0 0)
	 (throw_garbage)
	 (send *ri* :move-to receive-coord)
	 (print "今度は自分でしてください！")
	 (setq grasped nil)
	 ))
  )

(defun throw_garbage()
  (setq throw-pose-a 
	#f(50.0981 -2.65476 74.2725 9.07247 -113.869 172.78 -42.1726 6.74921 -30.9009 74.2725 -19.1349 -112.309 -163.76 -114.592 0.031832 1.46389 -21.2682))
  (setq throw-pose-b 
	#f(50.098 -5.86114 29.6401 33.1717 -36.0143 172.339 -8.16723 -15.2604 -30.1076 73.6891 -19.8699 -112.724 -163.415 -114.592 1.62229 1.75789 -21.2682))
  
  (send *ri* :angle-vector-sequence (list throw-pose-a throw-pose-b))
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  (send *ri* :go-pos -0.2 0.0 0)
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  )


(ros::subscribe "/joint_states"
		sensor_msgs::JointState
 #'get_garbage_cb)


(progn
  (ros::ok)
  (do-until-key
   (ros::spin-once))
   (ros::sleep))
