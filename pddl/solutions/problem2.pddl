(define (problem Problem2) (:domain Skiros)

; Goal: all waste in dumpster and dumpster closed
;
; Properties on objects can also be expressed like so:
;
;   (= (<property> <object>) <value>)
;
; in this particular problem we have the `skiros:Open` property
; which tells us if something is open or not.
;
; Hint: the possible values for the `skiros:Open` property are `True` and `False`
;
; To combine two or more conditions you can use `and`:
;
;   (and <condition1> <condition2> ...)
;
; Additionally it might be intractable to write down that _every_ piece
; of waste should be in the dumpster. For this one can use `forall`
;
;   (forall (?x - <type>) <condition/goal>)
;
; where `?x` can then be used as an arbitrary object of type <type> in
; <condition>. Since there are only two pieces of waste this is not
; strictly required.
(:goal (and
    ; All waste in dumpster
    (forall (?x - skiros:Waste) (skiros:contain skiros:Location-20 ?x))
    ; (skiros:contain skiros:Location-20 skiros:Waste-26)
    ; (skiros:contain skiros:Location-20 skiros:Waste-27)

    ; Dumpster closed
    (= (skiros:Open skiros:Location-20) False)
))

)
