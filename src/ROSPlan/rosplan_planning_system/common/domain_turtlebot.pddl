(define (domain turtlebot)
(:requirements :strips :typing :fluents)
    
(:types
    waypoint
    robot
    counter
)

;; functions to count the number of visited waypoints
(:functions
    (counter_value ?c - counter)
)
    
(:predicates
    (robot_at ?v - robot ?wp - waypoint)
    (visited ?wp - waypoint)
    (can_rotate ?v - robot)
    (back_to ?wp - waypoint)
    (can_move ?v - robot)
)

;; Robot rotates untill it finds the waypoint
(:action rotate
    :parameters (?v - robot ?wp - waypoint ?c - counter)
    :precondition (and 
    (can_rotate ?v)(robot_at ?v ?wp))
    :effect (and
    (visited ?wp)
    (increase (counter_value ?c) 1)
    
    (can_move ?v)
    (not(can_rotate ?v)))   
)
  
;; Move to any waypoint, avoiding terrain
(:action goto_waypoint
    :parameters (?v - robot ?from ?to - waypoint)
    :precondition (and 
    (robot_at ?v ?from)(can_move ?v))
    :effect (and
    (robot_at ?v ?to)(can_rotate ?v)(not(can_move ?v))(not(robot_at ?v ?from)))
    
)   

;; Come back to the initial position
(:action come_back
    :parameters (?v - robot ?from ?to - waypoint ?c - counter)
    :precondition (and (= (counter_value ?c) 4)(robot_at ?v ?from))
    :effect (and (back_to ?to)(visited ?to)(not(robot_at ?v ?from)))
    )
)


