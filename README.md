# Autonomous-vehicle-candidates
Code and relevant material for the 2026 Candidates competition.

Small brain storm:
 * For Pista B, omnidirectional wheels would the most efficient. We would have to think over the first section of the map, where the ball hides behind four different configurations of walls. Regardless, the second part with the white lines would be significantly easier, as would the last section of the colored tiles that represent directions of movement.
  
 * Omnidirectional wheels would never work on Pista A. They are really unreliable on textured terrain and would be completely unable to cross the "túmulos", even less so the stairs. From an open perspective, the best way would be to include those tank-like wheels that can run over anything, but we would then be unable to turn. We will likely have to use normal, rubber wheels, which means figuring out movement in a cramped space. This is a point we will have to explore and definitely improve on.
   
 * The ArUco is extremely important and we'll see what limitations come with integrating more components to account for the bonus.
  
 * It might be worthwhile to include both the ESP-32 and the available RaspBerry Pi 4. ESP for the more immediate needs, like spotting colors and obstacles, and the Rasp to integrate computer vision. OpenCV and all. This is specially necessary with the ArUco challenge.
  
 * We'll have to get a little golf ball to test for grab, for sure.

 * For the colors, HSV as always. For Pista B, a state machine for sure. We'll need to agree on the maze algorithm. The points for the colors detected are not generous, but Chiva would have to explore regardless to find the ArUco. Another state machine. The amount of sensors that would be ideally needed is a little worrisome. At least three distance sensors on the chassis, facing different directions. Preferably a screen to display color and ArUco number. The puente H. Whatever color sensor we'll use, which is as I see it, the biggest challenge of all. Detecting colors, avoiding the white lines in Pista B. Maybe the TCS34725, but that will need some type of cover to ensure proper reading and I'm not sure how that will work on the second part of Pista B. Again, the white lines.

 ## The last part of Pista B:
 * Cyan: move right
 * Yellow: move left
 * Orange: upward
 * Magenta: downward.

  Many challenges incoming. Agree on wheel approach soon.
