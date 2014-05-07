#!/usr/bin/env roseus
;;;
;;; euslisp version of ros_tutorials/rospy_tutorials/001_talker_listener
;;;

(ros::load-ros-manifest "roseus")
;;(ros::load-ros-manifest "leap_control")
(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")

(ros::roseus "leap_listener")

(defclass string-cb-class
    :super propertied-object
  :slots (prev-time update-freq av-queue time-queue))
(defmethod string-cb-class
    (:init ()
     (ros::subscribe "leapmotion/data" jsk_2014_04_pr2_73b2::leapros #'send self :string-cb)
     (setq prev-time (ros::time-now))
     (setq update-freq 500) ;; 1000/500 Hz
     (setq av-queue nil)
     (setq time-queue (list (ros::time-now)))
     )
                                        ; kokomade
                                        ;date ->direction nanikak
  (:string-cb (msg) 
              (setq hand_poss (send msg :points))
              (setq hand_points3D nil)
              (setq hand_points3D_r nil)
              (setq hand_points3D_t nil)
              (let ((x 0))
                (dolist (pos hand_poss)
                  (setq hand_points3D (append hand_points3D (list (float-vector (* (* -1  robot_scaler 5) (send pos :x)) (*  robot_scaler 5 (send pos :z)) (+ (*  robot_scaler 5 (+ (send pos :y) -190)) robot_h)))))
                  (send (elt cube_hands x) :locate (float-vector (*  -5 robot_scaler (send pos :x)) (* 5 robot_scaler (send pos :z)) (+ (* 5 robot_scaler (+ (send pos :y) -190)) robot_h)) :worldpos)    

                  (setq hand_points3D_r (append hand_points3D_r (list (float-vector (* -1.4 (- (send pos :x) 200)) (* 1.4 (send pos :z)) (* 1.4 (+ (send pos :y) -300))))))
                  (send (elt cube_hands_r x) :locate (float-vector (* -1.4 (- (send pos :x) 200)) (* 1.4 (send pos :z)) (* 1.4 (+ (send pos :y) -300))) :worldpos)

                  (setq hand_points3D_t (append hand_points3D_t (list (float-vector (* -1.4  (send pos :x)) (* 1.4 (send pos :z)) (* 1.4 (+ (send pos :y) -370))))))
                  (send (elt cube_hands_t x) :locate (float-vector (* -1.4 (send pos :x)) (* 1.4 (send pos :z)) (* 1.4 (+ (send pos :y) -370))) :worldpos)
                  (setq x (+ x 1))
                  )
                (while (> 10 x)
                       (send (elt cube_hands x) :locate #f(0 0 0) :worldpos)
                       (send (elt cube_hands_r x) :locate #f(1000 0 0) :worldpos)
                       (send (elt cube_hands_t x) :locate #f(1000 0 0) :worldpos)
                       (setq x (+ x 1))
                       )
                )

              (format t "get data~%")
              (setq avoid_point nil)


              (setq det_pos1 x_t1)
              (dolist (hand_pos hand_points3D_t)
                (setq dif (v- hand_pos x_t1))
                (if (< (v. dif dif) 3000)     
                    (progn
                      (setq det_pos1 (v+ det_pos1 (scale 0.1 dif))) 
                      )
                    )
                )
              (setq det_pos1 (scale 0.99 det_pos1))
              (setq x_t1 det_pos1) 
              (setq x_t1 (float-vector (elt x_t1 0) (elt x_t1 1) 0))
              (send cube_t1 :locate x_t1 :worldpos)        

              (setq det_pos1 x_r1)
              (dolist (hand_pos hand_points3D_r)
                (setq dif (v- hand_pos x_r1))
                (if (< (v. dif dif) 3000)     
                    (progn
                      (setq det_pos1 (v+ det_pos1 (scale 0.1 dif))) ;kokode minus mo ireru  
                      )
                    )
                )
              (setq det_pos2 x_r2)
              (dolist (hand_pos hand_points3D_r)
                (setq dif (v- hand_pos x_r2))
                (if (< (v. dif dif) 3000)     
                    (progn
                      (setq det_pos2 (v+ det_pos2 (scale 0.2 dif))) ;kokode minus mo ireru  
                      )
                    )
                )
              (setq dif (v- det_pos1 det_pos2))
              (setq dif_n (v. dif dif))
              (if grip_r 
                  (if (> dif_n 10000)
                      (progn
                        (setq grip_r nil)
                        (send *ri* :stop-grasp)
                        (setq scaler (* 0.000005 (- dif_n 10000)))      
                        )
                      (progn 
                        (setq scaler (* 0.000005 (- dif_n 6000)))      
                        )
                      )
                  (if (< dif_n 4000)
                      (progn
                        (setq grip_r t)
                        (send *ri* :start-grasp)
                        (setq scaler (* 0.000005 (- dif_n 6000)))      
                        )
                      (progn 
                        (setq scaler (* 0.000005 (- dif_n 10000)))      
                        )
                      )
                  )
              (print grip_r)
              (setq det_pos2 (v+ det_pos2 (scale scaler dif)))
                                        ;        (setq x_r1 det_pos1)
              (setq x_r1 (scale 100 (normalize-vector x_r1)))
              (send cube_r1 :locate  x_r1 :world_pos)
              (setq x_r2 det_pos2)
              (setq x_r2 (float-vector (elt x_r2 0) (elt x_r2 1) 0)) ;;this rotate
              (setq x_r2 (scale 140 (normalize-vector x_r2)))
              (send cube_r2 :locate  x_r2 :world_pos)
              
              
              (dolist (arm '(:rarm :larm))  
                                        ;(setq arm :larm) 
                (setq temp_pos (send (send *pr2* arm :end-coords) :worldpos))
                (setq det_pos temp_pos)
                (setq avoid_point (append avoid_point (list temp_pos)))
                (dolist (hand_pos hand_points3D)
                  (setq dif (v- hand_pos temp_pos))
                  (if (< (v. dif dif) (* robot_scaler robot_scaler 100000))     
                      (progn
                        (setq det_pos (v+ det_pos (scale (* robot_scaler 0.1) dif))) ;kokode minus mo ireru  
                        )
                      )
                  )
                (dolist (hand_pos avoid_point)
                  (setq dif (v- hand_pos temp_pos))
                  (if (< (v. dif dif) (* robot_scaler robot_scaler  100000))     
                      (progn
                        (setq det_pos (v+ det_pos (scale (* robot_scaler -0.2) dif))) ;kokode minus mo ireru  
                        )
                      )
                  )
                (if (equal arm :rarm)
                    (send *pr2* arm :inverse-kinematics 
                          (make-cascoords :pos det_pos :rot 
                                          (inverse-matrix
                                           (make-matrix 3 3 (list    
                                                             #f(1 0 0) #f(0 1 0) #f(0 0 1)
                                        ;            (normalize-vector  (scale 1 x_r1)) 
                                        ;            (normalize-vector (v- x_r1 x_r2))
                                        ;            (normalize-vector (v* x_r1 (v- x_r1 x_r2)))
                                        ;#f(0 -1 0) #f(0 0 -1)
                                                             ))
                                           )
                                          )
                                    :revert-if-fail nil :stop 10 :rotation-axis axis_pose)  
                    )
                (if (equal arm :larm)
                    (send *pr2* arm :inverse-kinematics (make-cascoords :pos det_pos) :revert-if-fail nil :stop 10 :rotation-axis axis_pose) 
                    )
                )
              (push (send *pr2* :angle-vector) av-queue)
              (push (ros::time-now) time-queue)
              (when (> (send (ros::time- (ros::time-now) prev-time) :sec) (/ update-freq 1000))
                (nreverse av-queue)
                (nreverse time-queue)
                (let (tms)
                  (dotimes (i (1- (length time-queue)))
                    (push (* (send (ros::time- (elt time-queue (1+ i)) (elt time-queue i)) :sec) 1000) tms))
                  (send *ri* :angle-vector-sequence av-queue tms))
                (setq av-queue (list (send *pr2* :angle-vector)))
                (setq time-queue (list (ros::time-now)))
                (setq prev-time (ros::time-now))
                (if (> (v. x_t1 x_t1) 3000) 
                    (progn
                      (send *pr2* :reset-pose)
                      (send *ri* :go-pos (* robot_scaler 0.005 (elt x_t1 0)) 0 (* 0.5 (elt x_t1 1)))
                      (print (* robot_scaler 0.01 (elt x_t1 0))) 
                      (send cube_t1 :set-color #f(0.8 0.4 0))
                      )
                    (send cube_t1 :set-color :green)
                    )
                (format t "after send *ri* ")
                )
              
              (send viewer :draw-objects)
              (send viewer_r :draw-objects)
              (send viewer_t :draw-objects)
              (x::window-main-one)

              ;;       (setq counter (1+ counter))


              )
  )


(defun nao-start ()
                                        ;for nao
  (load "package://naoeus/euslisp/nao-interface.l")
  (nao)
  (setq *pr2* *nao*)
  (setq *ri* (instance nao-interface :init))
  (send *ri* :servo-on)
  (setq robot_scaler 0.3)
  (setq robot_h 0)
  (setq axis_pose nil)
  (robot-start)
  )
                                        ;
                                        ;for pr2
(defun pr2-start ()
  (load "package://pr2eus/pr2-interface.l")
  (pr2)
  (pr2-init)
  (setq robot_scaler 0.8)
  (setq robot_h 550)
  (setq axis_pose t)     ;
  (robot-start) 
  )

(defun robot-start ()
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;  (setq counter 0)
  (setq cube_hands nil)
  (setq cube_hands_r nil)
  (setq cube_hands_t nil)


  (dotimes (x 10) (setq cube_hands (append cube_hands (list (make-cube (* robot_scaler 100)  (* robot_scaler 100) (* robot_scaler 100))))))
  (dotimes (x 10) (send (elt cube_hands x) :set-color :red))
  (dotimes (x 10) (setq cube_hands_r (append cube_hands_r (list (make-cube 20 20 20)))))
  (dotimes (x 10) (send (elt cube_hands_r x) :set-color :red))
  (dotimes (x 10) (setq cube_hands_t (append cube_hands_t (list (make-cube 20 20 20)))))
  (dotimes (x 10) (send (elt cube_hands_t x) :set-color :red))


  (setq grip_r nil)
  (setq cube_r1 (make-cube 20 20 20))
  (send cube_r1 :set-color :green)
  (setq x_r1 #f(100.0 0 0))
  (send cube_r1 :locate  x_r1 :world_pos)

  (setq cube_t1 (make-cube 20 20 20))
  (send cube_t1 :set-color :green)
  (setq x_t1 #f(0 0 0))
  (send cube_t1 :locate  x_t1 :world_pos)


  (setq cube_r2 (make-cube 20 20 20))
  (send cube_r2 :set-color :yellow)
  (setq x_r2 #f(80.0 -55 0))
  (send cube_r2 :locate  x_r2 :world_pos)

  (setq viewer_t (make-irtviewer))
  (setq viewer_r (make-irtviewer))
  (setq viewer (make-irtviewer))
  (send viewer :resize 1000 800)


  (setq l nil)
  (dotimes (x 20) (setq l (append  l (list (make-cube (* robot_scaler 2000) (* robot_scaler 1) (* robot_scaler 1))))))
  (dotimes (x 20) 
    (progn (send (elt l x) :locate (float-vector 0 0 (+ (* (- x 10) (* robot_scaler 100))   robot_h))) :worldpos)
    (send (elt l x) :set-color :green))

  (setq l2 nil)
  (dotimes (x 20) (setq l2 (append  l2 (list (make-cube (* robot_scaler 1)  (* robot_scaler 2000) (* robot_scaler 1))))))
  (dotimes (x 20) 
    (progn (send (elt l2 x) :locate (float-vector 0 0 (+ (* (- x 10 ) (* robot_scaler 100)) robot_h)) :worldpos)
           (send (elt l2 x) :set-color :yellow))
    )

  (send viewer_t :objects (cons cube_t1 cube_hands_t)) ;(send viewer_t :look-all)
  (send viewer :objects (append l  l2 (cons *pr2*  cube_hands)))
  (send viewer_r :objects (cons cube_r1 (cons cube_r2 cube_hands_r)))
  (send viewer :look-all)
  (setq m (instance string-cb-class :init))


  (do-until-key
      (ros::spin-once)
    (ros::sleep)
    )
  )

