#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; about: pr2 detecting someone's face 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "face_detector_mono")
(ros::roseus "face_detect")

(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "/openni/rgb/face_detector_mono/faces" face_detector_mono::RectArray #'send self :string-cb))
  (:string-cb (msg) 
(progn
(setq evaluation_value nil)	     
(if (eql (atom (send msg :rects)) nil) (setq evaluation_value t))
(if (eql evaluation_value t) (progn (speak-jp "ごみぶくろをいれてくれ　あとはまかせた")
				    (setq evaluation_value nil)))

)))

	      

(setq m (instance string-cb-class :init))

(do-until-key
 (ros::spin-once)
)


;;(注)　顔認識されると　atom (send msg :rects) はnilを返す　それ以外はt