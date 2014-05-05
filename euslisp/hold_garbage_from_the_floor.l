(when (boundp '*irtviewer*)
  (objects (list *pr2* *room*))
  (send *irtviewer* :look-all 
        (geo::make-bounding-box
         (flatten (send-all (send *pr2* :bodies) :vertices)))))

(defun hold-garbage-from-floor ()
  (warn "hold-garbage-from-floor~%")
  (pr2-reset-pose)

  (defparameter *pr2-hold-garbage-pose* #f(50.0207 33.1095 74.2513 59.9813 -67.2523 -27.8703 -5.72958 377.546 -22.9158 57.9269 0.113247 -116.905 0.001316 -5.74938 359.997 1.49389 -9.96))
  (send *ri* :angle-vector *pr2-hold-garbage-pose*  2000)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(100 0 0) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)

  (send *pr2* :larm :move-end-pos #f(0 0 100) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :go-pos -0.04 0 0)

  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))
