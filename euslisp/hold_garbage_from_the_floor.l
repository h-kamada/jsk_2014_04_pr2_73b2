(when (boundp '*irtviewer*)
  (objects (list *pr2* *room*))
  (send *irtviewer* :look-all 
        (geo::make-bounding-box
         (flatten (send-all (send *pr2* :bodies) :vertices)))))

(defun hold-garbage-from-floor ()
  (warn "hold-garbage-from-floor~%")
  (pr2-reset-pose)

  (defparameter *pr2-hold-garbage-pose* #f(50.0 26.8 73.8 97.6 -101.0 257.7 -68.5 -85.6 -60.0 74.0 -70.0 -120.0 -20.0 -30.0 180.0 0.0 0.0))
  (send *ri* :angle-vector *pr2-hold-garbage-pose*  2000)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :wait-interpolation)

  (send *pr2* :larm :move-end-pos #f(300 0 0) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)

  (send *pr2* :larm :move-end-pos #f(0 0 300) *pr2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (defparameter *pulling_up_garbage #f(50.0936 -4.63559 28.1569 26.1707 -118.605 27.901 -83.8011 -27.5549 5.93215 73 -93.6835 -75.9203 4.58844 -101.501 164.922 0.005887 -2.07))
  (send *ri* :angle-vector *pr2-hold-garbage-pose*  2000)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :wait-interpolation)

  (send *ri* :go-pos -0.04 0 0)

  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))
