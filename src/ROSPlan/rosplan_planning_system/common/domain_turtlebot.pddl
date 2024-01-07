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
    (robot_at ?v - robot ?wp - waypoint) ; Predicate to determine the current location of the robot
    (visited ?wp - waypoint) ; Predicate to check if a waypoint has been visited
    (can_rotate ?v - robot) ; Predicate to check if the robot can rotate
    (back_to ?wp - waypoint) ; Predicate to check if the robot can back to the initial position
    (can_move ?v - robot) ; Predicate to check if the robot can move
)

;; Robot rotates until it finds the waypoint
(:action rotate
    :parameters (?v - robot ?wp - waypoint ?c - counter)
    :precondition (and 
    (can_rotate ?v)(robot_at ?v ?wp))
    :effect (and
    (visited ?wp) ; Mark the waypoint as visited
    (increase (counter_value ?c) 1) ; Increase the waypoint visit counter
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
    :precondition (and (= (counter_value ?c) 4)(robot_at ?v ?from)) ;  All waypoints have been visited (counter is 4)
    :effect (and (back_to ?to)(visited ?to)(not(robot_at ?v ?from))) ; Robot goes back to a specific waypoint [in our case wp0: initial position]
    )
)


