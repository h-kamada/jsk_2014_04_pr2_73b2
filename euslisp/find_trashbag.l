#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;about: pr2 finding trash bag                      
;;                 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "find_trashbag")
(ros::roseus "find_trashbag")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(pr2-init)
(load "package://jsk_perception/euslisp/detection_interface.l")

(defun find_gomibukuro (&optional (root-frame-id "/base_footprint") 
				  (desired-marker-cds 
				   (make-coords :pos #f(1000 -800 1000) 
						;;(550 -1910 950)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
						;;desired-marker-coordsは/base_footprintから見た理想のマーカーの座標
						;;適宜直す
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

						:rot (list 0 0 0)))
				  (distance-threshold 25.0)
				  (rotation-threshold 0.09))
  (let ((speakp t)
	(prev-pr2-av (send *ri* :state :potentio-vector)))
    (if (not (boundp '*tfl*)) 
	
;;boundpは*tfl* がグローバルな値を持っているかをチェック

	(setq *tfl* (instance ros::transform-listener :init)))
    
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (send *pr2* :reset-pose)
    (send *pr2* :head :neck-p :joint-angle 40)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)

    ;(dotimes  (i 10)
    (let ((cds (check-detection 
		:speak-name "ごみぶくろ" 
		:timeout 30   
		:speak speakp
		:detection-topic "/checkerdetector/ObjectDetection"))  

;;cdsをcheck-detectionの結果として定義

	  diff-pos
	  diff-rot)
      (setq cds (send cds :copy-worldcoords)) ;;/home/kochigami/ros/groovy/jsk-ros-pkg/jsk_roseus/euslisp/jskeus/eus/Linux64/bin/irteusgl 0 error: cannot find method :objects in (send cds :objects)

      (print cds)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;send cds :difference-position desired-marker-cds
;;send cds :difference-rotation desired-marker-cds
;;
;;このメソッドは、cdsとdesired-marker-cdsの差をとっている
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

     ; (setq diff-pos (send cds :difference-position desired-marker-cds))  
     ; (setq diff-rot (send cds :difference-rotation desired-marker-cds))  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;difference-position というmethodはないとエラーが出た
;;角度もクォータニオンなのでこの方法でできるのか？

	;(setq *pose* nil)
	;(setq *orientation* nil)

	;(dolist (a cds)
	 ; (setq *x* (send a :pose :position :x))
	  ;(setq *y* (send a :pose :position :y))
	  ;(setq *z* (send a :pose :position :z))
	  ;(setq *pose* (float-vector *x* *y* *z*))
	  ;(setq *a_x* (send a :pose :orientation :x))
	  ;(setq *a_y* (send a :pose :orientation :y))
	  ;(setq *a_z* (send a :pose :orientation :z))
	  ;(setq *a_w* (send a :pose :orientation :w))
	  ;(setq *orientation* (float-vector *a_x* *a_y* *a_z* *a_w*))
	  ;)

;	(setq diff-pos (v- #f(1000 -800 1000) *pose*))
;	(setq diff-rot (v- *orientation* #f(0 0 0 0)))
	
      ;(ros::ros-info "~A ~A" diff-pos diff-rot)
      ;(when (and (< (norm diff-pos) distance-threshold)
       ;          (< (abs (elt diff-rot 2)) rotation-threshold))

      ;  (return)) ;;diff-pos diff-rotがpr2が調べている方の座標

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;check-detectionで何を求めているの??　多分格子の原点
;;それに応じてpr2の手先の位置を動かす
;;これでいいのか...???
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      (send *ri* :stop-grasp :arms)
      (send *ri* :wait-interpolation)
      

      (send *pr2* :inverse-kinematics 
	    (send (copy-object cds) :translate #f(-400 -100 -330) :world) ;;cdsのx y zの値はどう取り出すのか
	    ;;(copy-object cds)
	    :rotation-axis nil
	    :use-torso t
	    :move-arm :larm 
	    :debug-view t)
      
      (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
      
      (send *ri* :wait-interpolation)
	
      
      (send *ri* :start-grasp :arms :gain 0.05)
	


      (send *ri* :wait-interpolation)

      (send *pr2* :reset-pose)

      (send *ri* :angle-vector (send *pr2* :angle-vector))

      (load "models/arrow-object.l")

      (setq ar (arrow))

      (send ar :move-to (send cds :copy-worldcoords) :world)
      
      ;;(send ar :move-to cds :world) 
     
      (objects (list ar *pr2*))

      
)))  
  




