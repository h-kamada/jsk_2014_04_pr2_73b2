
(ros::roseus "nao-test")
(load "package://naoeus/euslisp/nao-interface.l")

(setq *nao* (nao))
(objects (list *nao*))
(setq *ni* (instance nao-interface :init))

(unix:sleep 1)
(send *ni* :servo-on)
(send *nao* :reset-pose)
(send *ni* :angle-vector (send *nao* :angle-vector) 1000)
(send *ni* :wait-interpolation)


(ros::load-ros-manifest "semi_nao")

(defun callback (msg)
  (setq *point* (send msg :point))
  (setq *pointX* (send *point* :x))
  (setq *pointY* (send *point* :y))

  (setq necky-current (send *nao* :head :neck-y :joint-angle))
  (setq neckp-current (send *nao* :head :neck-p :joint-angle))
  ;(print necky-current)
  (setq y-k 0.08)
  (setq p-k 0.05)
  (setq radius 100)

;;  (print (+ (* (- *pointX* 320) (- *pointX* 320)) (* (- *pointY* 240) (- *pointY* 240))))
  (if (> (+ (* (- *pointX* 320) (- *pointX* 320)) (* (- *pointY* 240) (- *pointY* 240))) (* radius radius))
      (progn
	(if (> *pointX* 320) 
	    (send *nao* :head :neck-y :joint-angle (- necky-current (* y-k (- *pointX* 320))))
	  (send *nao* :head :neck-y :joint-angle (+ necky-current (* y-k (- 320 *pointX*)))))
	(if (> *pointY* 240) 
	    (send *nao* :head :neck-p :joint-angle (+ neckp-current (* p-k (- *pointY* 240))))
	  (send *nao* :head :neck-p :joint-angle (- neckp-current (* p-k (- 240 *pointY*)))))
	(if (and (> necky-current 0) (< (- 119 necky-current) 70))
	    (progn 
	      (send *ni* :go-pos 0 0 45)
	      (send *ni* :wait-interpolation)
	      (send *nao* :head :neck-y :joint-angle 10)))
	  ;;else if the angle of neck is not that large, turns head only
	(if (and (< necky-current 0) (< (+ 119 necky-current) 70))
	    (progn
	      (send *ni* :go-pos 0 0 -45)
	      (send *ni* :wait-interpolation)	      
	      (send *nao* :head :neck-y :joint-angle -10)))
	;; (if (and (> *pointX* 320)) 
	;;     (send *nao* :head :neck-y :joint-angle (- necky-current (* y-k (- *pointX* 320))))
	;;   (send *nao* :head :neck-y :joint-angle (+ necky-current (* y-k (- 320 *pointX*)))))
	
	;;for head up-and-down
	(send *ni* :angle-vector (send *nao* :angle-vector) 500)
	(send *ni* :wait-interpolation)
	)
    ;;if object at the center, nao waves left hand
    (progn	
      (dotimes (i 3)
	(send *nao* :larm :shoulder-p :joint-angle -20)
	(send *nao* :larm :elbow-y :joint-angle -110)
	(send *nao* :larm :wrist-y :joint-angle 60)
	(send *ni* :angle-vector (send *nao* :angle-vector) 500)
	(send *ni* :wait-interpolation)
	(send *nao* :larm :elbow-y :joint-angle -50)
	(send *ni* :angle-vector (send *nao* :angle-vector) 300)
	(send *ni* :wait-interpolation))
      (send *nao* :larm :shoulder-p :joint-angle 110)
      (send *nao* :larm :elbow-y :joint-angle -90)
      (send *nao* :larm :wrist-y :joint-angle 0)
      (send *ni* :angle-vector (send *nao* :angle-vector) 500) 
      (send *ni* :wait-interpolation)
      ))
)

(ros::roseus "nao_seeing")
(ros::subscribe "/image_tracked/trackedpoint"
		geometry_msgs::PointStamped
		#'callback)
(progn
  (ros::ok)
  (do-until-key
   (ros::spin-once)
   (ros::sleep)
   )
  ;; (setq look-lr 0.0)
  ;; (setq look-ud 0.0)
  

;; (do-until-key
;;  (
;;   (if (> (send *nao* :state :head) head-yrange)
;;       movebody)
;;   (if (< (send *nao* :state :head) head-yrange)
;;       movebody)
;;   (if (> (send *nao* :state :head-p) head-prange)
;;       movehandup)


  
;;   (if (> X Xrange)
;;       (prog  (setq look-lr (+ look-lr 1.0))
;; 	     (send *nao* :head :neck-y :joint-angle look-lr)))
;;   (if (< X -Xrange)
;;        (prog (setq look-lr (- look-lr 1.0))
;; 	     (send *nao* :head :neck-y :joint-angle look-lr)))
;;   (if (> Y Yrange)
;;       (prog  (setq look-ud (+ look-ud 1.0))
;; 	     (send *nao* :head :neck-y :joint-angle look-ud)))
;;   (if (< Y -Yrange)
;;       (prog  (setq look-lr (+ look-lr 1.0))
;; 	     (send *nao* :head :neck-y :joint-angle look-lr)))

;;   ))