(define (problem task)
    (:domain turtlebot)
    (:objects
        wp0 wp1 wp2 wp3 wp4 - waypoint
        kenny - robot
        count - counter
    )
    (:init
        (= (counter_value count) 0)
        (robot_at kenny wp0)
        (visited wp0)
        (can_move kenny)
         
    )
    (:goal (and
        
        (visited wp1)
        (visited wp2)
        (visited wp3)
        (visited wp4)
        (back_to wp0)
 
        )
    )
)
