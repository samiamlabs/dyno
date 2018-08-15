(define (problem task)
(:domain quadrotor_visit_all)
(:objects
    start right up left - waypoint
    dyno_client - robot
)
(:init
    (robot_at dyno_client up)



)
(:goal (and
    (visited start)
    (visited right)
    (visited up)
    (visited left)
))
)
