(ros::load-ros-manifest "jsk_2014_04_pr2_73b2")
(ros::roseus "pr2_go_through_door")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(load "package://jsk_perception/euslisp/detection_interface.l")

(defparameter *detection-topic* "/openni/rgb/ObjectDetection")

(warn ";; go-through-73b2-door")
;; relative pose of marker on "/eng2/7f/room73B2-door-front" assumed that #<coordinates #Xcfa9190  499.571 -223.306 1245.997 / 2.559 -1.539 -2.643>
(defun go-through-73b2-door (&key (root-frame-id "/base_footprint")
                                  (tuckarm t)
                                  (door-front-spot "/eng2/7f/room73B2-door-front")
                                  (desired-marker-cds
                                   (make-coords :pos #f(500 -577 1246)
                                                :rpy (list 2.659 -1.539 -2.643))))

  (move-to-spot door-front-spot
                :clear-costmap t
                :tilt-enable (not tuckarm))
  (let ((speakp t)
        (prev-pr2-av (send *ri* :state :potentio-vector)))
    (if tuckarm
        (pr2-tuckarm-pose)
      (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))
    (send *pr2* :head :neck-p :joint-angle 20)
    (send *ri* :angle-vector (send *pr2* :angle-vector))
    (let ((cds (check-detection :type "73b2_door_checkerboard"
                                :speak-name "door"
                                :timeout 5
                                :speak speakp))
          diff-pos
          diff-rot)
      (setq diff-pos (send desired-marker-cds :difference-position cds))
      (setq diff-rot (send desired-marker-cds :difference-rotation cds))
      (ros::ros-info "~A ~A" diff-pos diff-rot)
      (send *ri* :go-pos-unsafe
            (/ (elt diff-pos 0) 1100.0)
            (/ (elt diff-pos 1) 1100.0)
            (* 0.9 (rad2deg (elt diff-rot 2)))))

    (speak-jp "前へ進みます")
    (send *ri* :go-pos-unsafe 0.8 0 0)
    (send *ri* :angle-vector prev-pr2-av)
    (send *ri* :wait-interpolation)
    ))

(provide :go-throuth-door)