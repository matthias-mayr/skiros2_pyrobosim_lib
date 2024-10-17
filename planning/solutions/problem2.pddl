(define (problem Problem2) (:domain Skiros)

(:objects 
	skiros:Location-16 skiros:Location-19 skiros:Location-20 skiros:Pantry-18 skiros:Fridge-17 skiros:Door-10 skiros:Door-11 skiros:Door-12 skiros:Door-13 skiros:Door-6 skiros:Door-7 skiros:Door-8 skiros:Door-9 skiros:Table-15 skiros:Charger-21 skiros:Room-2 skiros:Room-3 skiros:Room-4 skiros:Room-5 skiros:Room-1 - skiros:Location
	cora:Robot-14 - sumo:Agent
	skiros:Butter-25 skiros:Waste-26 skiros:Waste-27 skiros:Snacks-23 skiros:Soda-24 skiros:Bread-22 - skiros:Part
	rparts:GripperEffector-15 - rparts:GripperEffector
	skiros:Pantry-18 skiros:Fridge-17 skiros:Door-10 skiros:Door-11 skiros:Door-12 skiros:Door-13 skiros:Door-6 skiros:Door-7 skiros:Door-8 skiros:Door-9 - skiros:OpenableLocation
	skiros:Door-10 skiros:Door-11 skiros:Door-12 skiros:Door-13 skiros:Door-6 skiros:Door-7 skiros:Door-8 skiros:Door-9 - skiros:Door
)

(:init 
	(skiros:at cora:Robot-14 skiros:Room-1)
	(skiros:at robi:robot skiros:Room-1)
	(skiros:hasA cora:Robot-14 rparts:GripperEffector-15)
	(skiros:hasA robi:robot rparts:GripperEffector-15)
	(Empty rparts:GripperEffector-15)
	(skiros:contain skiros:Location-16 skiros:Waste-26)
	(skiros:contain skiros:Location-19 skiros:Waste-27)
	(skiros:contain skiros:Pantry-18 skiros:Snacks-23)
	(skiros:contain skiros:Pantry-18 skiros:Bread-22)
	(skiros:contain skiros:Fridge-17 skiros:Butter-25)
	(skiros:contain skiros:Fridge-17 skiros:Soda-24)
	(= (skiros:Open skiros:Door-6) True)
	(= (skiros:Open skiros:Door-7) False)
	(= (skiros:Open skiros:Door-8) False)
	(= (skiros:Open skiros:Door-9) True)
	(= (skiros:Open skiros:Door-10) False)
	(= (skiros:Open skiros:Door-11) False)
	(= (skiros:Open skiros:Door-12) False)
	(= (skiros:Open skiros:Door-13) False)
)

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
(:goal
	(skiros:contain skiros:Dumpster-20 skiros:Waste-26),(skiros:contain skiros:Dumpster-20 skiros:Waste-27),(not (skiros:Open skiros:Dumpster-20))
)

)
