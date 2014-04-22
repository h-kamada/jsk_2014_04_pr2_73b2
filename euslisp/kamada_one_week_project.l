(load "models/room73b2-scene.l")
(setq *room* (room73b2))
(load "pr2-interface.l")
(pr2)
;(pr2-init)
(objects (list *pr2* *room*))
(send *pr2* :move-to (send *room* :spot "cook-spot") :world)
(send *irtviewer* :look-all 
      (geo::make-bounding-box
	   (flatten (send-all (send *pr2* :bodies) :vertices))))
(send *pr2* :arms :shoulder-p :joint-angle 30)
(send *pr2* :rarm :move-end-pos #f(0 170 40) :world :rotation-axis nil)
;(send *ri* :wait-interpolation)
;(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)