(define (problem Problem4) (:domain Skiros)

(:objects 
	skiros:Location-16 skiros:Location-19 skiros:Dumpster-20 skiros:Pantry-18 skiros:Fridge-17 skiros:Door-10 skiros:Door-11 skiros:Door-12 skiros:Door-13 skiros:Door-6 skiros:Door-7 skiros:Door-8 skiros:Door-9 skiros:Table-15 skiros:Charger-21 skiros:Room-2 skiros:Room-3 skiros:Room-4 skiros:Room-5 skiros:Room-1 - skiros:Location
	rparts:GripperEffector-15 - rparts:GripperEffector
	cora:Robot-14 - sumo:Agent
	skiros:Charger-21 - skiros:Charger
	skiros:Butter-25 skiros:Waste-26 skiros:Waste-27 skiros:Snacks-23 skiros:Soda-24 skiros:Bread-22 - skiros:Part
	skiros:Dumpster-20 skiros:Pantry-18 skiros:Fridge-17 skiros:Door-10 skiros:Door-11 skiros:Door-12 skiros:Door-13 skiros:Door-6 skiros:Door-7 skiros:Door-8 skiros:Door-9 - skiros:OpenableLocation
)

(:init 
	(skiros:hasA cora:Robot-14 rparts:GripperEffector-15)
	(skiros:hasA robi:robot rparts:GripperEffector-15)
	(ContainerState_Empty rparts:GripperEffector-15)
	(skiros:at cora:Robot-14 skiros:Room-1)
	(skiros:at robi:robot skiros:Room-1)
	(skiros:Reachable skiros:Location-16)
	(skiros:Reachable skiros:Location-19)
	(skiros:Reachable skiros:Pantry-18)
	(skiros:Reachable skiros:Fridge-17)
	(skiros:Reachable skiros:Door-10)
	(skiros:Reachable skiros:Door-11)
	(skiros:Reachable skiros:Door-12)
	(skiros:Reachable skiros:Door-13)
	(skiros:Reachable skiros:Door-6)
	(skiros:Reachable skiros:Door-7)
	(skiros:Reachable skiros:Door-8)
	(skiros:Reachable skiros:Door-9)
	(skiros:Reachable skiros:Table-15)
	(skiros:Reachable skiros:Room-2)
	(skiros:Reachable skiros:Room-5)
	(skiros:Reachable skiros:Room-1)
	(skiros:contain skiros:Location-16 skiros:Waste-26)
	(skiros:contain skiros:Location-19 skiros:Waste-27)
	(skiros:contain skiros:Pantry-18 skiros:Snacks-23)
	(skiros:contain skiros:Pantry-18 skiros:Bread-22)
	(skiros:contain skiros:Fridge-17 skiros:Butter-25)
	(skiros:contain skiros:Fridge-17 skiros:Soda-24)
	(skiros:Open skiros:Location-16)
	(skiros:Open skiros:Location-19)
	(skiros:Open skiros:Door-6)
	(skiros:Open skiros:Door-9)
	(skiros:Open skiros:Table-15)
)

; Goal: Charger reachable
(:goal (and
	; Charger reachable, i.e. robot can navigate to charger directly without opening doors
    (skiros:Reachable skiros:Charger-21)
))

)
