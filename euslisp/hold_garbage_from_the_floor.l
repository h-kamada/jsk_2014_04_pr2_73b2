(when (boundp '*irtviewer*)
  (objects (list *pr2* *room*))
  (send *irtviewer* :look-all 
        (geo::make-bounding-box
         (flatten (send-all (send *pr2* :bodies) :vertices)))))

(defun hold-garbage-from-floor ()
  (warn "hold-garbage-from-floor~%")
  ;;(pr2-reset-pose)

  (defparameter *pr2-hold-garbage-pose* #f(50.0 26.8 73.8 97.6 -101.0 257.7 -68.5 -85.6 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0))
  (defparameter *pulling_up_garbage*    #f(50.0 -4.6 28.1 26.1 -118.6 27.90 -83.8 -27.5 5.930 73.0 -93.6 -75.92 4.588 -101  164.9 0.0 -2.0))

  (send *pr2* :reset-pose)

  (send *pr2* :angle-vector *pr2-hold-garbage-pose*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(100 0 0) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)

  (send *pr2* :larm :move-end-pos #f(0 0 300) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pulling_up_garbage*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  ;;(send *ri* :go-pos -0.04 0 0)

  ;;(send *pr2* :reset-pose)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  ;;(send *ri* :wait-interpolation)
)


;;(send-message *ri* (send *ri* :super) :angle-vector avs (make-sequence 'list (length avs) :initial-element 500))