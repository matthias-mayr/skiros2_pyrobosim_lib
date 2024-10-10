(define (problem Problem1) (:domain Skiros)

; Goal: the snacks should be on the table in the dining room
;
; To specify a relation between two elements you write the following:
;
;   (<relation> <from_object> <to_object>)
;
; In SkiROS2 the `skiros:contain` relation is used to specify this property
(:goal
    ; Snacks are on the table
    (skiros:contain skiros:Table-15 skiros:Snacks-23)
)

)
