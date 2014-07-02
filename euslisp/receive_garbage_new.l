#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_gui_msgs")
(ros::roseus "throw_garbage")

(setq cbox_x 0)
(setq cbox_y 0)
(setq cbox_z 0.7)
(setq elbow-force 0)
(setq grasp-force 0)

(setq update_data t)

(setq saw_checkerbox nil)


(setq receive-coord nil)

(setq moerugomi nil)

(setq throw-pose-a
      #f(50.0 4.41354 52.7406 55.3782 -103.724 -11.1028 -97.7521 -14.088 -5.20232 70.2236 -105.26 -89.532 69.9914 -5.72958 19.9806 0.0 30.0))

(setq throw-pose-b 
      #f(50.0 4.15228 74.0 9.07247 -113.869 172.0 -42.1726 6.74921 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))
(setq throw-pose-c1 
      #f(50.0 4.15228 24.8319 42.7085 -35.4088 172.0 -12.1733 0.0 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))
(setq throw-pose-c2
      #f(50.0 20.15228 24.8319 42.7085 -35.4088 172.0 -12.1733 0.0 -5.00281 69.9958 -105.003 -89.9965 70.0013 -6.00365 20.0005 0.0 50.0))


(defun main()
  (while (ros::ok)
    (get_garbage)
    ;(ask_question)
    (go_trashbox)
    (throw_garbage)
    )
  )

(defun get_garbage_cb (msg)
  (setq *effort* (send msg :effort))
  (setq *name* (send msg :name))
  (setq elbow-force (elt *effort* 35))
  (setq grasp-force (elt *effort* 39))
  )

(defun checkerbox_cb(msg)
  (setq *objects* (car (send msg :objects)))
  (if (not (eq *objects* nil)) 
      (progn
	(setq cbox_x (send *objects* :pose :position :x))
	(setq cbox_y (send *objects* :pose :position :y))
	(setq cbox_z (send *objects* :pose :position :z))
	(setq saw_checkerbox t)
	(print 'checkerbox_cb_called_inside)
	)
    )
  (print 'checkerbox_cb_called)
  )

(defun ask_question_cb (msg)
  (let ((candidate-list (send msg :texts)))
    (format t "received message: ~A~%" candidate-list)
    (when (find-if #'(lambda (candidate) (substringp "はい" candidate)) candidate-list)
      (setq moerugomi t)
      (speak-jp "わかりました")))
  (print moerugomi))

(defun ask_question()
  (speak-jp "燃えるゴミですか?")
  (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'ask_question_cb)
  (ros::ros-info "Waiting for tablet...")
  (setq time_a 0)
  (while (< (- (car (unix:ptimes)) time_a) 1000)
    (ros::spin-once))
  )
    

    
(defun get_garbage()
  (ros::subscribe "/joint_states" sensor_msgs::JointState #'get_garbage_cb)
  (setq get_finished nil)
  (setq grasped nil)
  (setq time_g 0)
  (get_garbage_pose)
  (while (eq get_finished nil)
    (ros::spin-once)
    (cond ((and (> elbow-force 1.5f) (eq grasped nil)) ;;grasp if objects being handed
	   (send *ri* :start-grasp :larm :gain 0.06) 
	   (setq grasped t)
	   (setq time_g (car (unix:ptimes)))
	   )
	  ((and (eq grasped t) (> grasp-force -10.0f)) ;;open clip if failed to grasp firmly
	   (speak-jp "やり直してください")
	   (send *ri* :stop-grasp :larm)
	   (setq grasped nil)
	   )
	  ((and (eq grasped t) (< elbow-force -3.0f)) ;;release if objects being taken
	   (send *ri* :stop-grasp :larm)
	   (setq grasped nil)
	   )
	  ((and (eq grasped t) (> (- (car (unix:ptimes)) time_g) 600));; good to move
	   (setq get_finished t)
	   )))
  )

(defun go_trashbox()
  (speak-jp "移動します")
  (setq receive-coord (send *ri* :state :worldcoords))
  (send *ri* :angle-vector throw-pose-a 2000)
  (send *ri* :wait-interpolation)
  (move-to-spot "/eng2/7f/room73B2-trashbox-front")
  (send *ri* :angle-vector throw-pose-a 2000)
  (send *ri* :wait-interpolation)
  )

(defun throw_garbage()
  (ros::subscribe "/checkerdetector/ObjectDetection" posedetection_msgs::ObjectDetection  #'checkerbox_cb)
  (speak-jp "ゴミ箱を探します")
  (while (eq saw_checkerbox nil)
    (ros::spin-once)
    )
  (adjust_pose)
  (throw_garbage_pose)
  (send *ri* :move-to receive-coord)
  (setq saw_checkerbox nil)
)


(defun throw_garbage_pose()
  (send *ri* :angle-vector throw-pose-b 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector throw-pose-c1 4000)
  (send *ri* :wait-interpolation)
  
  (send *ri* :stop-grasp :larm)
  (unix:sleep 3)
  (send *ri* :angle-vector throw-pose-b 4000)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector throw-pose-a 3000)
  (send *ri* :wait-interpolation)
  )


(defun adjust_pose()
  (setq diff_z (- cbox_z 0.7))
  (format t "~a ~10,3f ~a ~10,3f ~%" "diff_z=" diff_z "-cbox_x=" (- cbox_x))
  (send *ri* :go-pos-unsafe (if (> diff_z 0) diff_z (- diff_z))
	(- cbox_x) 
	0.0)
  )


(defun get_garbage_pose()

  (send *ri* :start-grasp)
  (pr2-tuckarm-pose :larm)
  (send *ri* :angle-vector throw-pose-a 1500)
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