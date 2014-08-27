(in-package :pr2-example)

(defstruct gmmseds
  segment
  id)

(def-fact-group gmm-actions (action-desig)

  ;;; Reaching
  (<- (action-desig ?desig (movement ?act))
    (desig-prop ?desig (type gmm))
    (desig-prop ?desig (movement reach))
    (lisp-fun make-gmmseds :segment "reach" :id 1  ?act)
  )

  ;;; Swipping
  (<- (action-desig ?desig (movement ?act))
    (desig-prop ?desig (type gmm))
    (desig-prop ?desig (movement swipe))
    (lisp-fun make-gmmseds :segment "swipe" :id 2  ?act)
  )

  ;;; Flipping
  (<- (action-desig ?desig (movement ?act))
    (desig-prop ?desig (type gmm))
    (desig-prop ?desig (movement flip))
    (lisp-fun make-gmmseds :segment "flip" :id 3  ?act)
  )

  ;;; Back
  (<- (action-desig ?desig (movement ?act))
    (desig-prop ?desig (type gmm))
    (desig-prop ?desig (movement back))
    (lisp-fun make-gmmseds :segment "back" :id 4  ?act)
  )
)
