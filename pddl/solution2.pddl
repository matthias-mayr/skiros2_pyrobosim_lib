(define (problem Problem2) (:domain Skiros)

(:goal (and
    (forall (?x - skiros:Waste) (skiros:contain skiros:Location-20 ?x))
    (skiros:PROPERTY skiros:Location-20 skiros:CLOSED)
))

)
