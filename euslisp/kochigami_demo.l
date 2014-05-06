#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")
(ros::load-ros-manifest "face_detector_mono")
(ros::roseus "pr2_gets_trashbag_and_bring_it_someone")

(load "package://pr2eus/pr2-interface.l")
(pr2-init)

(objects (list *pr2*))
(when (not (boundp '*irtviewer*))
  (make-irtviewer)
  (objects (list *pr2*))
  (send *irtviewer* :draw-objects))

(setq *ri* (instance pr2-interface :init))

(send *pr2* :reset-pose)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;(send *ri* :wait-interpolation)

(load "package://jsk_2014_04_pr2_73b2/euslisp/find_trashbag.l")
(find_gomibukuro)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;全部無限ループっぽいのだけど、face_detect_listener.lでとまりそう・・・;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;(load "package://jsk_2014_04_pr2_73b2/euslisp/face_detect_listener.l")
;(load "package://jsk_2014_04_pr2_73b2/euslisp/give_someone_trashbag.l")
;(load "package://jsk_2014_04_pr2_73b2/euslisp/interaction_demo.l")
(defclass string-cb-class
  :super propertied-object
  :slots())


(defmethod string-cb-class
  (:init() (progn
	     (ros::subscribe "joint_states" sensor_msgs::JointState #'send self :joint-cb)
	     (ros::subscribe "/openni/rgb/face_detector_mono/faces" face_detector_mono::RectArray #'send self :face-cb)
	     
	     )
	)



  (:joint-cb (msg)
	       (progn 
		 
		 (setq effort (elt (send msg :effort) 32))
	       		;左肩の値
		  (setq effort2 (elt (send msg :effort) 18))                                                                   


                     (if  (> (abs effort) 2.0)
                         (progn                                              ;;               


                           (send *pr2* :torso :waist-z :joint-angle 50)
                           (send *pr2* :larm :collar-y :joint-angle 46.6521)
                           (send *pr2* :larm :shoulder-p :joint-angle 28.2833)
                           (send *pr2* :larm :shoulder-r :joint-angle 39.6189)
                           (send *pr2* :larm :elbow-p :joint-angle -103.725)
                           (send *pr2* :larm :elbow-r :joint-angle 6.83664)
                           (send *pr2* :larm :wrist-p :joint-angle -22.2449)
                           (send *pr2* :larm :wrist-r :joint-angle 178.785)
                           (send *pr2* :head :neck-p :joint-angle -15)
			   (send *pr2* :head :neck-y :joint-angle 0)
                           (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
                           ;;(send *ri* :wait-interpolation)                                  
                           ;;(unix:sleep 2)                                                   

                           (send *ri* :stop-grasp :arms)
                           ;;(unix:sleep 4)                                  
			   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
			   (send *ri* :wait-interpolation)
                           (speak-jp "よろしくね")

                           (send *pr2* :reset-pose)
                           (send *ri* :angle-vector (send *pr2* :angle-vector)3000)
                           ))      
		 
	       (if (> (abs effort2) 2.0)
		   (progn
		     (send *pr2* :reset-pose)
		     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
		     (send *ri* :wait-interpolation)
		     
		     (send *pr2* :larm :collar-y :joint-angle 70)
		     (send *pr2* :rarm :collar-y :joint-angle -70)
		     (send *pr2* :larm :shoulder-r :joint-angle 100)
		     (send *pr2* :rarm :shoulder-r :joint-angle -100)
		     (send *pr2* :head :neck-p :joint-angle 30)
		     (send *ri* :angle-vector (send *pr2* :angle-vector))
		     (send *ri* :wait-interpolation)
		     (speak-jp "ありがとう") 
		     (send *ri* :angle-vector (send *pr2* :reset-pose))
		     (send *ri* :wait-interpolation)
		     )))) 
  (:face-cb (msg)
	    (progn
	      (setq evaluation_value nil)
	      (when (send msg :rects )
		(if (eql (atom (send msg :rects)) nil)
		    (progn (setq evaluation_value t)
			   (setq count (+ count 1))
			   (print count)
			   (if (eql evaluation_value t)
			       (if (equal count 1)
				   (progn
				    ;; (print "ok")
				     (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
				     (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
				     (objects (list *pr2*))
				     (send *pr2* :head :neck-p :joint-angle 30)
				     (send *pr2* :larm :shoulder-p :joint-angle 0)
				     (send *pr2* :larm :elbow-p :joint-angle -50)
				     (send *pr2* :larm :elbow-r :joint-angle 10)
				     (send *ri* :angle-vector (send *pr2* :angle-vector))
				     (send *ri* :wait-interpolation)
				     (speak-jp "ごみぶくろをいれてくれ　あとはまかせた")
				     (setq evaluation_value nil)
				     (send *ri* :angle-vector (send *pr2* :reset-pose))
				     )
				 )
			     )
			   )
		  )
		)
	      )
	    ))
	

(setq m (instance string-cb-class :init))
(setq count 0)
(do-until-key
 (ros::spin-once))