(ros::load-ros-manifest "roseus")
(ros::roseus "throw_garbage")
(load "package://jsk_perception/euslisp/detection_interface.l")

(setq throw-pose-a
      #f(50.0 4.41354 52.7406 55.3782 -103.724 -11.1028 -97.7521 -14.088 -5.20232 70.2236 -105.26 -89.532 69.9914 -5.72958 19.9806 0.0 30.0))

(setq throw-pose-b 
      #f(50.0 4.15228 74.0 9.07247 -113.869 172.0 -42.1726 6.74921 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))
(setq throw-pose-c1 
      #f(50.0 4.15228 24.8319 42.7085 -35.4088 172.0 -12.1733 0.0 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))
(setq throw-pose-c2
      #f(50.0 20.15228 24.8319 42.7085 -35.4088 172.0 -12.1733 0.0 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))



(setq good-to-move nil)

(setq cbox_x 0)
(setq cbox_y 0)
(setq cbox_z 0.7)


(defun checkerbox_cb(msg)
  (setq *objects* (car (send msg :objects)))
  (if (not (eq *objects* nil)) 
      (progn
        (setq cbox_x (send *objects* :pose :position :x))
	(setq cbox_y (send *objects* :pose :position :y))
	(setq cbox_z (send *objects* :pose :position :z))
	)
    )
  )

(defun get_garbage_cb (msg)
  (setq *effort* (send msg :effort))
  (setq *name* (send msg :name))
					;(print (elt *name* 39))  ;;35:l_elbow_flex_joint 39:l_gripper_finger
  (setq elbow-force (elt *effort* 35))
  (setq grasp-force (elt *effort* 39))
  (print grasp-force)
  (if (eq (boundp 'grasped) nil) (setq grasped nil))
  (if (eq (boundp 'time_g) nil) (setq time_g 0))
  (print grasped)
  (cond ((and (> elbow-force 1.5f) (eq grasped nil)) ;;grasp if objects being handed
	 (send *ri* :start-grasp :larm :gain 0.06) 
	 (setq grasped t)
	 (setq time_g (car (unix:ptimes)))
	 )	
	
	((and (eq grasped t) (> grasp-force -10.0f)) ;;open clip if failed to grasp firmly
;	 (speak-jp "やり直してください")
					;(unix:sleep 8)
	 (send *ri* :stop-grasp :larm)
	 (setq grasped nil)
	 )
	((and (eq grasped t) (< elbow-force -3.0f)) ;;release if objects being taken
	 (send *ri* :stop-grasp :larm)
	 (setq grasped nil)
	 ;(speak-jp "あ")
	 )
	((and (eq good-to-move nil) (eq grasped t) (> (- (car (unix:ptimes)) time_g) 800));; go to throw garbage
	 (setq good-to-move t)
	 (speak-jp "移動します")
	 (setq receive-coord (send *ri* :state :worldcoords))
					;(pr2-tuckarm-pose :larm)
	 (send *ri* :angle-vector throw-pose-a 2000)
	 (send *ri* :wait-interpolation)
	 )
	
	((eq good-to-move t) 
					;(send *ri* :go-pos 0.1 0 0)
	 (move-to-spot "/eng2/7f/room73B2-trashbox-front")
	 (adjust_pose)
	 (throw_garbage)
					;(send *ri* :go-pos -0.4 0.0 0.0)
	 (pr2-tuckarm-pose :larm)
	 ;; (pr2-tuckarm-pose :larm)
	 (clear-costmap)
	 (send *ri* :move-to receive-coord)
	 (speak-jp "今度は自分で捨ててください")
	 (get_garbage_pose)
	 (setq grasped nil)
	 (setq good-to-move nil)
	 ))
  (print (- (car (unix:ptimes)) time_g))
  )


(defun adjust_pose()
  (setq diff_z (- cbox_y 0.7))
  (send *ri* :go-pos (if (> diff_z 0) diff_z (- diff_z))
	(- cbox_x) 
	0.0
	)
  )


(defun throw_garbage()
  (send *ri* :angle-vector throw-pose-b 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector throw-pose-c1 4000)
  (send *ri* :wait-interpolation)
  
  (send *ri* :stop-grasp :larm)
  (unix:sleep 3)
  (send *ri* :angle-vector throw-pose-a 6000)
  (send *ri* :wait-interpolation)
  )

(defun main()
  (get_garbage_pose)
  (ros::rate 10)
  (ros::subscribe "/joint_states" sensor_msgs::JointState #'get_garbage_cb)			
  (ros::subscribe "/checkerdetector/ObjectDetection" posedetection_msgs::ObjectDetection  #'checkerbox_cb)
  (ros::ros-info "Waiting for message...")  
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))

(defun get_garbage_pose()

  (send *ri* :start-grasp)
  (pr2-tuckarm-pose :larm)
  ;; (send *pr2* :rarm :wrist-p :joint-angle -90)
  ;; (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  ;; (send *ri* :wait-interpolation)
  
  (send *ri* :angle-vector throw-pose-a 1000)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :collar-y :joint-angle -24)
  (send *pr2* :larm :shoulder-p :joint-angle 25)
  (send *pr2* :larm :shoulder-r :joint-angle 0)
  (send *pr2* :larm :elbow-p :joint-angle -110)
  (send *pr2* :larm :elbow-r :joint-angle 200)
  (send *pr2* :larm :wrist-p :joint-angle -80)
  (send *pr2* :larm :wrist-r :joint-angle 0)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
  (send *ri* :wait-interpolation)
  (send *ri* :stop-grasp :larm)
  )


(main)