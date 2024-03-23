/**
 * @copyright Copyright (C) 2024 Ethan Uppal. All rights reserved.
 * @author Ethan Uppal
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <icp/icp.h>
#include "aux/tui.h"

#define NODE_NAME "scan_matching_node"
#define LIDAR_TOPIC "/scan"
#define N_LIDAR_POINTS 1147
#define TUI_N 23

class ScanMatchingNode {
    ros::NodeHandle n;
    ros::Subscriber lidar_subscriber;
    bool looking_for_scan;
    std::vector<sensor_msgs::LaserScan> scans;
    std::unique_ptr<icp::ICP> point_to_point;
    std::vector<icp::Point> a;
    std::vector<icp::Point> b;
    ethan_tui::tui tui;

    void on_lidar_scan(sensor_msgs::LaserScan scan) {
        ROS_INFO("on_lidar_scan\n");
        if (looking_for_scan) {
            visualize_point_cloud(scan);
            if (scans.empty()) {
                scans.push_back(scan);
            } else if (scans.size() == 1) {
                scans.push_back(scan);
            } else {
                scans[0] = scans[1];
                scans[1] = scan;
            }
            looking_for_scan = false;
        }
    }

    void visualize_point_cloud(sensor_msgs::LaserScan& scan) {
        memset(tui.b, ' ', sizeof(tui.w * tui.h));

        float angle =
            M_PI_2;          // scan.angle_min is -pi but starts scans at front
        double scale = 0.3;  // meters
        for (int i = 0; i < N_LIDAR_POINTS; i++) {
            double r = scan.ranges[i];
            double a = angle;
            angle += scan.angle_increment;
            if (r >= scale || r < scan.range_min) {
                continue;
            }
            double d = (scan.ranges[i] / scale) * (TUI_N / 2);
            double x = d * std::cos(a);
            double y = d * std::sin(a);
            tui.b[(-(int)y + TUI_N / 2) * TUI_N + (int)x + TUI_N / 2] = '#';
        }

        tui.b[TUI_N / 2 * TUI_N + TUI_N / 2] = '@';

        // assumes characters are height:width = 2:1
        std::cout << "┌";
        for (int r = 0; r < TUI_N; r++) std::cout << "──";
        std::cout << "┐\n";
        for (int r = 0; r < TUI_N; r++) {
            std::cout << "│";
            for (int c = 0; c < TUI_N; c++) {
                std::cout << tui.b[r * TUI_N + c] << tui.b[r * TUI_N + c];
            }
            std::cout << "│\n";
        }
        std::cout << "└";
        for (int r = 0; r < TUI_N; r++) std::cout << "──";
        std::cout << "┘\n";
        std::cout << "SCALE: side = " << (2 * scale) << " meters\n";
        std::cout.flush();
    }

    void request_single_lidar_scan() {
        looking_for_scan = true;
    }

    void scan_to_point_cloud(std::vector<icp::Point>& cloud,
        sensor_msgs::LaserScan& scan) {
        while (cloud.size() < N_LIDAR_POINTS) {
            cloud.push_back(icp::Point());
        }
        float angle = scan.angle_min;
        int j = 0;
        for (int i = 0; i < N_LIDAR_POINTS; i++) {
            double r = scan.ranges[i];
            if (r == INFINITY) {
                a.pop_back();
                continue;
            }
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);
            cloud[j++] = icp::Point(x, y);
            angle += scan.angle_increment;
        }
    }

    void scan_match() {
        ROS_INFO("invoke scan_match\n");
        scan_to_point_cloud(a, scans[0]);
        scan_to_point_cloud(b, scans[1]);
        ROS_INFO("got point clouds\n");
        icp::Transform guess{};  // e.g., from odometry
        point_to_point->set_initial(guess);
        point_to_point->converge(a, b, 100);
        ROS_INFO("finished icp\n");
        auto result = point_to_point->transform();
        std::cout << "dx=" << result.dx << ", dy=" << result.dy
                  << ", deg=" << (result.theta / M_PI * 180) << '\n';
        std::cout.flush();
    }

public:
    ScanMatchingNode()
        : n("~"),
          looking_for_scan(false),
          point_to_point(icp::ICP::from_method("point_to_point", N_LIDAR_POINTS,
              0.01)),
          a(N_LIDAR_POINTS),
          b(N_LIDAR_POINTS) {
        lidar_subscriber = n.subscribe(LIDAR_TOPIC, 1000,
            &ScanMatchingNode::on_lidar_scan, this);
        if (!lidar_subscriber) {
            ROS_ERROR("ScanMatchingNode failed to subscribe to " LIDAR_TOPIC
                      "\n");
            std::exit(1);
        }
    }

    void run() {
        enum class RunState {
            WaitFirst,
            WaitSecond,
            StartAgain,
            NUM_STATES
        } state = RunState::WaitFirst;
        bool state_changed = true;
        tui_begin(tui, TUI_N, TUI_N);
        while (ros::ok()) {
            if (state_changed) {
                switch (state) {
                    case RunState::WaitFirst: {
                        std::cout
                            << "\033[H\033[2JPRESS ENTER FOR FIRST SCAN\n";
                        std::cout.flush();
                        break;
                    }
                    case RunState::WaitSecond: {
                        std::cout << "\nPRESS ENTER FOR SECOND SCAN\n";
                        std::cout.flush();
                        break;
                    }
                    case RunState::StartAgain: {
                        std::cout << "\nPRESS ENTER TO START AGAIN\n";
                        std::cout.flush();
                        break;
                    }
                }
                state_changed = false;
            }
            tui_keys(tui);
            if (tui.k == '\n') {
                switch (state) {
                    case RunState::WaitFirst: {
                        request_single_lidar_scan();
                        break;
                    }
                    case RunState::WaitSecond: {
                        request_single_lidar_scan();
                        scan_match();
                        break;
                    }
                }

                state = static_cast<RunState>(
                    (static_cast<int>(state) + 1)
                    % static_cast<int>(RunState::NUM_STATES));
                state_changed = true;
            }
        }
        tui_end(tui);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME);
    ScanMatchingNode node{};
    node.run();
    return 0;
}
