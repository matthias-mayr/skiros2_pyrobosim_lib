(define (problem Problem3) (:domain Skiros)

; Goal: bread and butter on dining room table, fridge and pantry closed
;
; This exercise should be an opportunity to reuse the existing
; concepts from the previous two exercises
(:goal (and
    (skiros:contain skiros:Table-15 skiros:Bread-22)    ; Bread on table
    (skiros:contain skiros:Table-15 skiros:Butter-25)   ; Butter on table
    (= (skiros:Open skiros:Fridge-17) False)            ; Fridge closed
    (= (skiros:Open skiros:Pantry-18) False)            ; Pantry closed
))

)
