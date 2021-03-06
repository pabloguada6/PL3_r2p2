(define (problem pb1)
    (:domain rover_mars_extended)
    (:objects P0006 P2129 P1430 P1105 P2507 - position
              curiosity - rover)
    (:init (in_position curiosity P0006) (occupied P0006)
           (= (battery_percentage curiosity) 30)
           (= (distance P0006 P2129) 44)
           (= (distance P0006 P1430) 38)
           (= (distance P0006 P1105) 12)
           (= (distance P0006 P2507) 26)
           (= (distance P2129 P0006) 44)
           (= (distance P2129 P1430) 8)
           (= (distance P2129 P1105) 35)
           (= (distance P2129 P2507) 26)
           (= (distance P1430 P0006) 38)
           (= (distance P1430 P2129) 8)
           (= (distance P1430 P1105) 28)
           (= (distance P1430 P2507) 34)
           (= (distance P1105 P0006) 12)
           (= (distance P1105 P2129) 35)
           (= (distance P1105 P1430) 28)
           (= (distance P1105 P2507) 16)
           (= (distance P2507 P0006) 26)
           (= (distance P2507 P2129) 26)
           (= (distance P2507 P1430) 34)
           (= (distance P2507 P1105) 16)
           (= (take_picture_duration) 6)  
           (= (take_picture_consumption) 12)  
           (= (drill_duration) 10)  
           (= (drill_consumption) 20)  
           (= (earth_communication_duration) 18)  
           (= (earth_communication_consumption) 36)  
           (= (analyse_samples_duration) 14)  
           (= (analyse_samples_consumption) 28)  
    )
    (:goal (and (have_image curiosity P2507)
               	(drilled curiosity P1105)
             	(drilled curiosity P2129)
             	(have_earth_communication curiosity P1430)
             	(have_samples_analysed curiosity P2129))
    )
)