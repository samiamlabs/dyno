(define (domain quadrotor_deliver_parcels)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint
	robot
	parcel
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(parcel_at ?p - parcel ?wp - waypoint)
)

;; Move parcel to waypoint, avoiding terrain
(:durative-action move_parcel
	:parameters (?v - robot ?p - parcel ?from ?to - waypoint)
	:duration ( = ?duration 10)
	:condition (and
		(at start (robot_at ?v ?from)))
	:effect (and
		(at end (parcel_at ?p ?to)))
)
)
