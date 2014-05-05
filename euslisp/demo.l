(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(require :go-through-door "go-through-door.l")

;; load h-kamada
(load "hold_garbage_from_the_floor.l")
(load "throw_away_the_garbage.l")

(defun pr2-init-for-elevator ()
  (pr2)
  (setq *ri* (instance pr2-interface :init :move-base-action-base "elevator_move_base_simple")))

(defun setup ()
  (pr2-init-for-elevator) ;; use instead for (pr2-init)
)

(defun demo ()
  ;; au

  ;; kochigami

  ;; h-kamada
  (move-to-spot "/eng2/7f/room73B2-trashbox-front" :clear-costmap nil)
  (hold-garbage-from-floor)
  (go-through-73b2-door)
  (move-to-spot "/eng2/1f/garbage_front" :clear-costmap nil)
  (throw-away-garbage)
  (move-to-spot "/eng2/7f/room73A3-front")
  (go-through-73b2-door :door-front-spot "/eng2/7f/room73A3-front"
                        :desired-marker-cds (make-coords :pos #f(500 577 1246)
                                                         :rpy (list 2.659 1.539 -2.643)))
  (move-to-spot "/eng2/7f/room73B2-fridge-front")
  (speak-jp "ただいま")
  )

;;(setup)
;;(demo)