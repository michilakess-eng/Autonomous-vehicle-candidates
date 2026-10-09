Host tests for Pista A (no ESP32 needed; fake Arduino headers in tests/stubs). Run from the project root:
  g++ -std=gnu++11 -O1 -Itests/stubs -Isrc tests/sim_pista_a.cpp src/PistaA.cpp src/Display.cpp -o sim_a && ./sim_a 1900
  g++ -std=gnu++11 -Itests/stubs -Isrc tests/test_obstacles.cpp -o t_obs && ./t_obs
  g++ -std=gnu++11 -Itests/stubs -Isrc tests/test_colors.cpp src/Sensors.cpp -o t_col && ./t_col
Not covered: motors, IMU, ToF/RGB timing, the Navigator control loop (needs the robot).
