(load "package://pr2eus/pr2-interface.l")
(load "package://pr2eus/speak.l")
(load "package://jsk_demo_common/euslisp/pr2-move.l")


(pr2)
(when (not (boundp '*irtviewer*))
  (make-irtviewer)
  (objects (list *pr2*))
  (send *irtviewer* :draw-objects))

(setq *ri* (instance pr2-interface :init))

(send *pr2* :reset-pose)
(if (boundp '*irtviewer*)
    (send *irtviewer* :draw-objects))
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)
