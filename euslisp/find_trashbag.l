#!/usr/bin/env roseus

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;about: pr2 finding trash bag                      
                 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(ros::load-ros-manifest "find_trashbag")
(ros::roseus "find_trashbag")


(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "package://jsk_perception/euslisp/detection_interface.l")

(defun find_gomibukuro (&optional (root-frame-id "/base_footprint") 
                                       (desired-marker-cds 
                                        (make-coords :pos #f(1000 -800 1000) 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;desired-marker-coordsは理想のマーカーの座標??
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                                     :rot (list 0 0 0)))
                                       (distance-threshold 25.0)
                                       (rotation-threshold 0.09))
(let ((speakp t)
      (prev-pr2-av (send *ri* :state :potentio-vector)))
  (if (not (boundp '*tfl*)) 

;;boundpは*tfl* がグローバルな値を持っているかをチェック

      (setq *tfl* (instance ros::transform-listener :init)))
  (pr2-tuckarm-pose)

  (dotimes  (i 10)
    (let ((cds (check-detection :type "object"
                                :object "trashbag" 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;:objectをcheck-detectionのどこで使っているのかわからない
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                                :timeout "30"
                                :speak speakp))  

;;cdsをcheck-detectionの結果として定義

          diff-pos
          diff-rot)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;これは何を代入しているの??２つ違うの??
;;
;;send cds :difference-position desired-marker-cds
;;send cds :difference-rotation desired-marker-cds
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

      (setq diff-pos (send cds :difference-position desired-marker-cds))  
      (setq diff-rot (send cds :difference-rotation desired-marker-cds))  

      (ros::ros-info "~A ~A" diff-pos diff-rot)
      (when (and (< (norm diff-pos) distance-threshold)
                 (< (abs (elt diff-rot 2)) rotation-threshold))

        (return)) ;;diff-pos diff-rotがpr2が調べている方の座標

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;check-detectionで何を求めているの??　格子の中心座標??
;;それに応じてpr2の手先の位置を動かす
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;;    #f()を変える
      ;;(setq c (make-cascoords :pos #f() :rpy (float-vector 0 0 0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;これでいいのか...???
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      (send *ri* :inverse-kinematics c
	    :link-list (send *ri* :link-list (send *ri* :link-list (send *ri* :end-coords :parent)))
	    :move-target (send *ri* :end-coords) :translation-axis t :rotation-axis t)
)

      (send *ri* :start-grasp :arms :gain 0.05)))))




