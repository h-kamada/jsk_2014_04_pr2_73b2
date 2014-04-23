#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")
(ros::roseus "speech_recognition_sample")

(defun callback (msg)
  (let ((candidate-list (send msg :texts)))
    (format t "received message: ~A~%" candidate-list)
    (when (find-if #'(lambda (candidate) (substringp "こんにちは" candidate)) candidate-list)
      (ros::ros-info "Hello, JSK!"))))

(defun main ()
  (ros::rate 10)
  (ros::subscribe "/Tablet/voice" jsk_gui_msgs::VoiceMessage #'callback)
  (ros::ros-info "Waiting for tablet...")
  (while (ros::ok)
   (ros::spin-once)
   (ros::sleep)))

(defun usage ()
  (ros::ros-info "1. open chrome web browser.")
  (ros::ros-info "2. access http://pr1040.jsk.imi.i.u-tokyo.ac.jp:8000/rwt_voice_message/")
  (ros::ros-info "3. Enjoy speaking!!!"))

(usage)
(main)