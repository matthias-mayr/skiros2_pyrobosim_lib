(define (problem Problem4) (:domain Skiros)

; Goal: all waste in dumpster, dumpster closed. Bread and butter on dining
; room table, fridge and pantry closed
(:goal (and
    (forall (?x - skiros:Waste) (skiros:contain skiros:Location-20 ?x)) ; All waste in dumpster
    (= (skiros:Open skiros:Location-20) False)                          ; Dumpster closed
    (skiros:contain skiros:Table-15 skiros:Bread-22)                    ; Bread on table
    (skiros:contain skiros:Table-15 skiros:Butter-25)                   ; Butter on table
    (= (skiros:Open skiros:Fridge-17) False)                            ; Fridge closed
    (= (skiros:Open skiros:Pantry-18) False)                            ; Pantry closed
))

)
