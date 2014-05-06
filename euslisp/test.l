(ros::load-ros-manifest "roseus")
(ros::roseus "test_checkerbox")
(load "package://jsk_perception/euslisp/detection_interface.l")



(defun main()
  
  (ros::rate 10)
  (ros::subscribe "/checkerdetector/ObjectDetection" posedetection_msgs::ObjectDetection  #'checkerbox_cb)
  (ros::ros-info "Waiting for message...")  
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)))


(defun checkerbox_cb(msg)
  (setq *objects* (send msg :objects))
  (print *objects*.position.x))
  )


(main)