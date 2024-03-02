/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#pragma once

#include <array>

/**
 * Performs one iteration of the follow-the-gap algorithm.
 *
 * @param scan The LIDAR scan from -90 to 90 degrees relative to the robot's
 * current heading. It will be copied when passed.
 * @param safety_radius The radius of a circle centered at the LIDAR sensor that
 * circumscribes the robot.
 *
 * @pre `safety_radius` and each distance in `scan` are all positive.
 * @pre `N >= 2`.
 *
 * @returns The requested change in heading relative to the current heading. The
 * furthest distance in the gap is aimed for.
 *
 * @par Time complexity:
 * `O(N)`, where `N` is the size of the scan.
 */
template<size_t N>
double follow_the_gap(std::array<double, N> scan, double safety_radius);
