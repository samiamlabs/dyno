(define (problem task)
(:domain quadrotor_deliver_parcels)
(:objects
    start right up left - waypoint
    dyno_client - robot
    red_box blue_box yellow_box green_box - parcel
)
(:init
    (robot_at dyno_client start)


)
(:goal (and
    (parcel_at red_box start)
    (parcel_at blue_box right)
    (parcel_at yellow_box up)
    (parcel_at green_box left)
))
)
