/**
 * @copyright Copyright (C) 2023 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <cmath>
#include <algorithm>
#include "follow_the_gap.h"

// Ethan's implementation of follow-the-gap, based only on the F1Tenth lecture.

template<size_t N>
double follow_the_gap(std::array<double, N> scan, double safety_radius,
    double range) {
    static_assert(N >= 2, "The LIDAR scan must provide at least two points.");

    const double angle_increment = range / N;
    const long offset = (long)N / 2;

    // step 1: find nearest LIDAR point and put a safety bubble around it
    long closest_index = std::min_element(scan.begin(), scan.end())
                         - scan.begin();
    double safety_angle_delta =
        (std::atan(safety_radius / scan[closest_index]) / 2) * (180.0 / M_PI);
    int safety_delta = (int)(safety_angle_delta / angle_increment);
    long low = std::max(closest_index - safety_delta, (long)0);
    long high = std::min(closest_index + safety_delta + 1, (long)N);
    for (long i = low; i < high; i++) {
        scan[i] = 0;
    }

    // step 2: find max thresholded subarray
    long start = 0;
    long length = 0;
    long best_start = 0;
    long max_length = 0;
    for (long i = 0; i < (long)N; i++) {
        if (scan[i] > safety_radius) {
            length++;
            if (length > max_length) {
                max_length = length;
                best_start = start;
            }
        } else {
            length = 0;
            start = i + 1;
        }
    }

    // step 3: choose furthest point in gap
    long furthest_index = std::max_element(scan.begin() + best_start,
                              scan.begin() + best_start + max_length)
                          - scan.begin();
    return (furthest_index - offset) * angle_increment;
}
