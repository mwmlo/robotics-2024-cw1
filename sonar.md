# Sonar Calibration

1. `Min = 28cm`, `Max = 205cm`
2. `Max angular deviation = 35°`
3. - `20cm: 21, 21, 21 => 1cm err`
   - `40cm: 40, 40 ,40 => 0cm err`
   - `60cm: 60, 59, 59 => 0.66cm err`
   - `80cm: 79, 79, 79 => 1cm err`
   - `100cm: 99, 99, 99 => 1cm err`
<!---(Measurements in rk1121 Keep notes)--->
4. - `40cm:`
       1. 40
       2. 40
       3. 40
       4. 40
       5. 41
       6. 41
       7. 40
       8. 40
       9. 40
       10. 39
    - `100cm:`
       1. 100
       2. 99
       3. 99
       4. 99
       5. 100
       6. 100
       7. 99
       8. 100
       9. 99
       10. 99
5. The sonar sensor's measurements are accurate within a limited distance range (up to 200 cm) and within a limited range of angular deviation. These are very specific conditions that will not always appear in general conditions for robot navigation, particularly when the state space is large or has many corners. Therefore, we estimate the sonar will give inaccurate readings 60% of the time, and garbage readings 30% of the time.
