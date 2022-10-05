# EKF-SLAM
The current version implements EKF-SLAM with known correspondence.

To use it in the simplest manner, run FullSim(Number of landmarks, Number of iterations, Size of square map, Measuring range). The specified number of landmarks will be randomly positioned in a square with the specified size, and the robot will move randomly with a fixed linear velocity (the fact that it's fixed is easily alterable) for the chosen number of iterations, measuring in each iterations the distance and angle to every landmark in its measuring range.

Coming soon: Importing maps, variable number of arguments (for choosing velocity vectors and such), and more!



Landmark initialization from http://www.joansola.eu/JoanSola/eng/course.html

Main source: S. Thrun, W.Burgard, D. Fox, “Probabilistic Robotics”, MIT Press, 2005 – SLAM
