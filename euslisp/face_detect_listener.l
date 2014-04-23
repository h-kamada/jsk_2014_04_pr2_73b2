#!/usr/bin/env roseus
;;(ros::load-ros-manifest "roseus_tutorials")
(ros::load-ros-manifest "face_detector_mono")
;;;

;;;
;;;
(ros::roseus "listener")
;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;; callback function
;(defun string-cb (msg) (print (list 'cb (sys::thread-self) (send msg :data))))
;(ros::subscribe "chatter" std_msgs::string #'string-cb)

; lambda function
;(ros::subscribe "chatter" std_msgs::string
;                #'(lambda (msg) (ros::rosinfo 
;                                 (format nil "I heard ~A" (send msg :data)))))

;; method call
(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "/openni/rgb/face_detector_mono/faces" face_detector_mono::RectArray #'send self :string-cb))
  (:string-cb (msg) (if (atom (send msg :rects)) (speak-jp "ありがとう")))) 
(setq m (instance string-cb-class :init))

(do-until-key
 (ros::spin-once)
 ;;(sys::gc)
)
;(ros::spin)
