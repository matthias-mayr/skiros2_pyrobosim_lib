(define (problem Problem4) (:domain Skiros)

(:goal (and
    (forall (?x - skiros:Waste) (skiros:contain skiros:Location-20 ?x))
    (skiros:PROPERTY skiros:Location-20 skiros:CLOSED)
    (skiros:contain skiros:Table-15 skiros:Bread-)
    (skiros:contain skiros:Table-15 skiros:Butter-)
    (skiros:PROPERTY skiros:Fridge-17 skiros:CLOSED)
    (skiros:PROPERTY skiros:Pantry-18 skiros:CLOSED)
))

)
