#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <unordered_set>
#include <queue>
#include <chrono>

using namespace std::chrono;

struct Position {
    int x, y;
    bool operator==(const Position &other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template <> struct hash<Position> {
        size_t operator()(const Position &p) const {
            return hash<int>()(p.x) ^ hash<int>()(p.y);
        }
    };
}

class MazeSolver : public rclcpp::Node {
public:
    MazeSolver() : Node("maze_solver"), total_distance(0.0), start_time(steady_clock::now()), last_position_time(steady_clock::now()) {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeSolver::scan_callback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::unordered_set<Position> visited_positions;
    std::queue<Position> path_queue;
    float total_distance;
    steady_clock::time_point start_time;
    steady_clock::time_point last_position_time;
    Position last_position = {0, 0};
    bool maze_completed = false;
    bool reversing = false;

    bool is_corner(float front, float left, float right) {
        return (front < 0.6 && left < 0.6 && right < 0.6); // Increased threshold
    }

    bool is_stuck(Position current_position) {
        if (current_position == last_position) {
            if (duration_cast<seconds>(steady_clock::now() - last_position_time).count() > 5) {
                return true;
            }
        } else {
            last_position = current_position;
            last_position_time = steady_clock::now();
        }
        return false;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (maze_completed) return;

        float front_distance = msg->ranges[msg->ranges.size() / 2];
        float left_distance = msg->ranges[msg->ranges.size() * 3 / 4];
        float right_distance = msg->ranges[msg->ranges.size() / 4];
        auto cmd_msg = geometry_msgs::msg::Twist();

        Position current_position = {static_cast<int>(total_distance * 10), 0};
        if (visited_positions.find(current_position) == visited_positions.end()) {
            visited_positions.insert(current_position);
            path_queue.push(current_position);
        }

        if (is_stuck(current_position)) {
            cmd_msg.linear.x = -0.5; // Reverse a good distance
            cmd_msg.angular.z = 1.57; // Rotate 90 degrees
            reversing = true;
        }
        else if (is_corner(front_distance, left_distance, right_distance)) {
            if (!reversing) {
                cmd_msg.linear.x = -0.5;  // Reverse first
                reversing = true;
            } else {
                if (left_distance > right_distance) {
                    cmd_msg.angular.z = 1.57; // Take a right turn first
                } else {
                    cmd_msg.angular.z = -1.57; // If that fails, take a left turn
                }
                reversing = false;
            }
        }
        else if (front_distance > 0.7) {
            cmd_msg.linear.x = 1.5;  // Move fast
            total_distance += 1.5 * 0.1;
        }
        else if (front_distance < 0.5 && left_distance < 0.5 && right_distance > 0.7) {
            cmd_msg.angular.z = -1.0; // Turn right to escape
        }
        else if (front_distance < 0.5 && right_distance < 0.5 && left_distance > 0.7) {
            cmd_msg.angular.z = 1.0; // Turn left to escape
        }
        else if (left_distance > 0.5) {
            cmd_msg.angular.z = 0.8;
        }
        else {
            cmd_msg.angular.z = -0.8;
        }

        cmd_vel_publisher_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MazeSolver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
