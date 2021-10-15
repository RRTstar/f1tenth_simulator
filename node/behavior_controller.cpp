#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/package.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <fstream>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace racecar_simulator;

class BehaviorController : public rclcpp::Node {
private:
    // Clock
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // Publisher for mux controller
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr mux_pub;

    // Mux indices
    int joy_mux_idx_int;
    int key_mux_idx_int;
    int random_walker_mux_idx_int;
    int brake_mux_idx_int;
    int nav_mux_idx_int;
    // ***Add mux index for new planner here***
    // int new_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size_int;

    // Button indices
    int joy_button_idx_int;
    int key_button_idx_int;
    int random_walk_button_idx_int;
    int brake_button_idx_int;
    int nav_button_idx_int;
    // ***Add button index for new planner here***
    // int new_button_idx;

    // Key indices
    std::string joy_key_char_str;
    std::string keyboard_key_char_str;
    std::string brake_key_char_str;
    std::string random_walk_key_char_str;
    std::string nav_key_char_str;
    // ***Add key char for new planner here***
    // int new_key_char;

    // Is ebrake on? (not engaged, but on)
    bool safety_on;

    // To roughly keep track of vehicle state
    racecar_simulator::CarState state;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // for collision detection
    double ttc_threshold;
    bool in_collision = false;

    // for collision logging
    std::ofstream collision_file_os;
    double beginning_seconds;
    int collision_count = 0;


public:
    BehaviorController() : Node("behavior_controller") {
        // get topic names
        this->declare_parameter("scan_topic");
        const std::string scan_topic_str = this->get_parameter("scan_topic").as_string();
        this->declare_parameter("odom_topic");
        const std::string odom_topic_str = this->get_parameter("odom_topic").as_string();
        this->declare_parameter("imu_topic");
        const std::string imu_topic_str = this->get_parameter("imu_topic").as_string();
        this->declare_parameter("joy_topic");
        const std::string joy_topic_str = this->get_parameter("joy_topic").as_string();
        this->declare_parameter("keyboard_topic");
        const std::string keyboard_topic_str = this->get_parameter("keyboard_topic").as_string();
        this->declare_parameter("brake_bool_topic");
        const std::string brake_bool_topic_str = this->get_parameter("brake_bool_topic").as_string();
        this->declare_parameter("mux_topic");
        const std::string mux_topic_str = this->get_parameter("mux_topic").as_string();

        // Make a publisher for mux messages
        mux_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>(mux_topic_str, 10);

        // Start subscribers to listen to laser scan, joy, IMU, and odom messages
        this->create_subscription<sensor_msgs::msg::LaserScan>(scan_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::laser_callback, this, std::placeholders::_1));
        this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::joy_callback, this, std::placeholders::_1));
        this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::imu_callback, this, std::placeholders::_1));
        this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::odom_callback, this, std::placeholders::_1));
        this->create_subscription<std_msgs::msg::String>(keyboard_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::key_callback, this, std::placeholders::_1));
        this->create_subscription<std_msgs::msg::Bool>(brake_bool_topic_str, rclcpp::SensorDataQoS(), std::bind(&BehaviorController::brake_callback, this, std::placeholders::_1));

        // Get mux indices
        this->declare_parameter("joy_mux_idx");
        joy_mux_idx_int = this->get_parameter("joy_mux_idx").as_int();
        this->declare_parameter("key_mux_idx");
        key_mux_idx_int = this->get_parameter("key_mux_idx").as_int();
        this->declare_parameter("random_walker_mux_idx");
        random_walker_mux_idx_int = this->get_parameter("random_walker_mux_idx").as_int();
        this->declare_parameter("brake_mux_idx");
        brake_mux_idx_int = this->get_parameter("brake_mux_idx").as_int();
        this->declare_parameter("nav_mux_idx");
        nav_mux_idx_int = this->get_parameter("nav_mux_idx").as_int();
        // ***Add mux index for new planner here***
        // n.getParam("new_mux_idx", new_mux_idx);

        // Get button indices
        this->declare_parameter("joy_button_idx");
        joy_button_idx_int = this->get_parameter("joy_button_idx").as_int();
        this->declare_parameter("key_button_idx");
        key_button_idx_int = this->get_parameter("key_button_idx").as_int();
        this->declare_parameter("random_walk_button_idx");
        random_walk_button_idx_int = this->get_parameter("random_walk_button_idx").as_int();
        this->declare_parameter("brake_button_idx");
        brake_button_idx_int = this->get_parameter("brake_button_idx").as_int();
        this->declare_parameter("nav_button_idx");
        nav_button_idx_int = this->get_parameter("nav_button_idx").as_int();
        // ***Add button index for new planner here***
        // n.getParam("new_button_idx", new_button_idx);

        // Get key indices
        this->declare_parameter("joy_key_char");
        joy_key_char_str = this->get_parameter("joy_key_char").as_string();
        this->declare_parameter("keyboard_key_char");
        keyboard_key_char_str = this->get_parameter("keyboard_key_char").as_string();
        this->declare_parameter("random_walk_key_char");
        random_walk_key_char_str = this->get_parameter("random_walk_key_char").as_string();
        this->declare_parameter("brake_key_char");
        brake_key_char_str = this->get_parameter("brake_key_char").as_string();
        this->declare_parameter("nav_key_char");
        nav_key_char_str = this->get_parameter("nav_key_char").as_string();
        // ***Add key char for new planner here***
        // n.getParam("new_key_char", new_key_char);

        // Initialize the mux controller
        this->declare_parameter("mux_size");
        mux_size_int = this->get_parameter("mux_size").as_int();
        mux_controller.reserve(mux_size_int);
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = false;
        }

        // Start with ebrake off
        safety_on = false;

        // Initialize state
        state.x = 0.0;
        state.y=0.0;
        state.theta=0.0;
        state.velocity=0.0;
        state.steer_angle=0.0;
        state.angular_velocity=0.0;
        state.slip_angle=0.0;
        state.st_dyn=false;

        // Get params for precomputation and collision detection
        this->declare_parameter("scan_beams");
        int scan_beams_int = this->get_parameter("scan_beams").as_int();

        // double ttc_threshold_double = this->get_parameter("ttc_threshold").as_double();
        this->declare_parameter("scan_distance_to_base_link");
        double scan_distance_to_base_link_double = this->get_parameter("scan_distance_to_base_link").as_double();
        this->declare_parameter("width");
        double width_double = this->get_parameter("width").as_double();
        this->declare_parameter("wheelbase");
        double wheelbase_double = this->get_parameter("wheelbase").as_double();
        this->declare_parameter("scan_field_of_view");
        double scan_field_of_view_double = this->get_parameter("scan_field_of_view").as_double();

        double scan_ang_incr = scan_field_of_view_double / scan_beams_int;

        // Precompute cosine and distance to car at each angle of the laser scan
        cosines = Precompute::get_cosines(scan_beams_int, -scan_field_of_view_double/2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(scan_beams_int, wheelbase_double, width_double,
                scan_distance_to_base_link_double, -scan_field_of_view_double/2.0, scan_ang_incr);

        // Create collision file to be written to
        this->declare_parameter("collision_file");
        std::string collision_file_str = this->get_parameter("collision_file").as_string();
        collision_file_os.open(ament_index_cpp::get_package_share_directory("f1tenth_simulator") + "/logs/" + collision_file_str + ".txt");
        // rclcpp::Time time;
        // beginning_seconds = time.seconds();
        beginning_seconds = clock->now().seconds();
    }

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    void publish_mux() {
        // make mux message
        std_msgs::msg::Int32MultiArray mux_msg;
        mux_msg.data.clear();
        // push data onto message
        for (int i = 0; i < mux_size_int; i++) {
            mux_msg.data.push_back(int(mux_controller[i]));
        }

        // publish mux message
        mux_pub->publish(mux_msg);
    }

    void change_controller(int controller_idx_int) {
        // This changes the controller to the input index and publishes it

        // turn everything off
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[controller_idx_int] = true;

        publish_mux();
    }

    void collision_checker(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // This function calculates TTC to see if there's a collision
        if (state.velocity != 0) {
            for (size_t i = 0; i < msg->ranges.size(); i++) {
                double angle = msg->angle_min + i * msg->angle_increment;

                // calculate projected velocity
                double proj_velocity = state.velocity * cosines[i];
                double ttc = (msg->ranges[i] - car_distances[i]) / proj_velocity;

                // if it's small, there's a collision
                if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                    // Send a blank mux and write to file
                    collision_helper();

                    in_collision = true;

                    collision_count++;
                    collision_file_os << "Collision #" << collision_count << " detected:\n";
                    collision_file_os << "TTC: " << ttc << " seconds\n";
                    collision_file_os << "Angle to obstacle: " << angle << " radians\n";
                    collision_file_os << "Time since start of sim: " << (clock->now().seconds() - beginning_seconds) << " seconds\n";
                    collision_file_os << "\n";
                    return;
                }
            }
            // if it's gone through all beams without detecting a collision, reset in_collision
            in_collision = false;
        }
    }

    void collision_helper() {
        // This function will turn off ebrake, clear the mux and publish it

        safety_on = false;

        // turn everything off
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = false;
        }

        publish_mux();
    }

    void toggle_mux(int mux_idx_int, std::string driver_name_str) {
        // This takes in an index and the name of the planner/driver and
        // toggles the mux appropriately
        if (mux_controller[mux_idx_int]) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), driver_name_str + " turned off");
            mux_controller[mux_idx_int] = false;
            publish_mux();
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), driver_name_str + " turned on");
            change_controller(mux_idx_int);
        }
    }

    void toggle_brake_mux() {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency brake engaged");
        // turn everything off
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = false;
        }
        // turn on desired controller
        mux_controller[brake_mux_idx_int] = true;

        publish_mux();
    }


    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void brake_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && safety_on) {
            toggle_brake_mux();
        } else if (!msg->data && mux_controller[brake_mux_idx_int]) {
            mux_controller[brake_mux_idx_int] = false;
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Changing mux_controller:
        if (msg->buttons[joy_button_idx_int]) {
            // joystick
            toggle_mux(joy_mux_idx_int, "Joystick");
        }
        if (msg->buttons[key_button_idx_int]) {
            // keyboard
            toggle_mux(key_mux_idx_int, "Keyboard");
        }
        else if (msg->buttons[brake_button_idx_int]) {
            // emergency brake
            if (safety_on) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency brake turned off");
                safety_on = false;
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency brake turned on");
                safety_on = true;
            }
        }
        else if (msg->buttons[random_walk_button_idx_int]) {
            // random walker
            toggle_mux(random_walker_mux_idx_int, "Random Walker");
        } else if (msg->buttons[nav_button_idx_int]) {
            // nav
            toggle_mux(nav_mux_idx_int, "Navigation");
        }
        // ***Add new else if statement here for new planning method***
        // if (msg.buttons[new_button_idx_int]) {
        //  // new planner
        //  toggle_mux(new_mux_idx_int, "New Planner");
        // }
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Changing mux controller:
        if (msg->data == joy_key_char_str) {
            // joystick
            toggle_mux(joy_mux_idx_int, "Joystick");
        } else if (msg->data == keyboard_key_char_str) {
            // keyboard
            toggle_mux(key_mux_idx_int, "Keyboard");
        } else if (msg->data == brake_key_char_str) {
            // emergency brake
            if (safety_on) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency brake turned off");
                safety_on = false;
            }
            else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency brake turned on");
                safety_on = true;
            }
        } else if (msg->data == random_walk_key_char_str) {
            // random walker
            toggle_mux(random_walker_mux_idx_int, "Random Walker");
        } else if (msg->data == nav_key_char_str) {
            // nav
            toggle_mux(nav_mux_idx_int, "Navigation");
        }
        // ***Add new else if statement here for new planning method***
        // if (msg.data == new_key_char_str) {
        //  // new planner
        //  toggle_mux(new_mux_idx_int, "New Planner");
        // }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // check for a collision
        collision_checker(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Keep track of state to be used elsewhere
        state.velocity = msg->twist.twist.linear.x;
        state.angular_velocity = msg->twist.twist.angular.z;
        state.x = msg->pose.pose.position.x;
        state.y = msg->pose.pose.position.y;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr /* msg */ ) {
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorController>());
    rclcpp::shutdown();
    return 0;
}
