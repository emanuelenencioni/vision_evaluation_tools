#!/bin/bash
g++ -O3 -DNDEBUG -o evaluate_odometry evaluate_odometry.cpp matrix.cpp

g++ -O3 -DNDEBUG -o evaluate_odometry_avg evaluate_odometry_avg.cpp matrix.cpp