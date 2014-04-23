#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")
(ros::roseus "speech_recognition_sample")

(defun callback (msg)
  (let ((candidate-list (send msg :texts)))
    (format t "received message: ~A~%" candidatelist)
    (when (find-if #'(lambda (candidate) (substringp "こんにちは" candidate)) candidate-list)
      (ros::ros-info "Hello, JSK!"))))

(defun main ()
  (ros::rate 10)
  (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'callback)
  (ros::ros-info "Waiting for tablet...")
  (while (ros::ok)
   (ros::spin-once)
   (ros::sleep)))

(main)