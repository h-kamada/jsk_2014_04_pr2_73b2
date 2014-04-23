(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")
(ros::roseus "pr2_go_through_door")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "package://jsk_perception/euslisp/detection_interface.l")

(print ";; go-through-73b2-door~%")
(defun go-through-73b2-door (&optional (root-frame-id "/base_footprint")
                                       (desired-marker-cds
                                        (make-coords :pos #f(1000 -800 1000)
                                                     :rot (list 0 0 0)))
                                       (distance-threshold 25.0)
                                       (rotation-threshold 0.09))
  (let ((speakp t)
        (prev-pr2-av (send *ri* :state :potentio-vector)))
    (if (not (boundp '*tfl*))
        (setq *tfl* (instance ros::transform-listener :init)))
    (pr2-tuckarm-pose)
    (dotimes (i 10)
      (let ((cds (check-detection :type "door_marker_object"
                                  :object "door"
                                  :timeout 30
                                  :speak speakp))
            diff-pos
            diff-rot)
        (setq diff-pos (send cds :difference-position desired-marker-cds))
        (setq diff-rot (send cds :difference-rotation desired-marker-cds))
        (ros::ros-info "~A ~A" diff-pos diff-rot)
        (when (and (< (norm diff-pos) distance-threshold)
                   (< (abs (elt diff-rot 2)) rotation-threshold))
          (return))
        (send *ri* :go-pos-unsafe
              (/ (elt diff-pos 0) 1100.0)
              (/ (elt diff-pos 1) 1100.0)
              (* 0.9 (rad2deg (elt diff-rot 2)))))))

  (speak-jp "前へ進みます")
  (send *ri* :go-pos-unsafe 0.8 0 0)


  (send *ri* :angle-vector prev-pr2-av)
  (send *ri* :wait-interpolation)
  )