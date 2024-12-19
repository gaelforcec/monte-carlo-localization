# VEX REAL TIME MONTE CARLO LOCALIZATION

Team [5327C] VEX Monte Carlo Impl.

## Features

  - Advanced Monte Carlo Localization with Kalman Filter fusion
  - Supports custom odometry tracking through `chassis.odom_tracking_set()`
  - Flexible integration with any tracking system (tracking wheels, distance sensors, imus)
  - Seamless fusion of multiple sensor inputs for robust position estimation
  - Real-time particle filtering with dynamic resampling
  - Kalman filter smoothing for precise state estimation

## Project Structure

- `src/`: Source files
- `include/`: Header files
- `docs/`: Documentation /* TODO: */

## Resources
https://www.ri.cmu.edu/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf
https://www.youtube.com/watch?v=htE5cClSy4Y
https://www.youtube.com/watch?v=BkxSYHpfnic
