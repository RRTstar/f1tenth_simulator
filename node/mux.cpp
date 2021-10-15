#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "f1tenth_simulator/channel.h"

class Mux : public rclcpp::Node {
private:
    // Clock
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // Publish drive data to simulator/car
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    // Mux indices
    int joy_mux_idx_int;
    int key_mux_idx_int;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size_int;

    // Channel array
    std::vector<Channel*> channels;

    // Make Channel class have access to these private variables
    friend class Channel;

    // For printing
    std::vector<bool> prev_mux;

    // Params for joystick calculations
    int joy_speed_axis_int, joy_angle_axis_int;
    double max_speed_double, max_steering_angle_double;
    // For keyboard driving
    double prev_key_velocity=0.0;
    double keyboard_speed_double;
    double keyboard_steer_ang_double;


public:
    Mux() : Node("mux_controller") {
        // get topic names
        this->declare_parameter("drive_topic");
        std::string drive_topic_str = this->get_parameter("drive_topic").as_string();
        this->declare_parameter("mux_topic");
        std::string mux_topic_str = this->get_parameter("mux_topic").as_string();
        this->declare_parameter("joy_topic");
        std::string joy_topic_str = this->get_parameter("joy_topic").as_string();
        this->declare_parameter("keyboard_topic");
        std::string keyboard_topic_str = this->get_parameter("keyboard_topic").as_string();

        // Make a publisher for drive messages
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_str, 10);

        // Start a subscriber to listen to mux messages
        this->create_subscription<std_msgs::msg::Int32MultiArray>(mux_topic_str, 10, std::bind(&Mux::mux_callback, this, std::placeholders::_1));

        // Start subscribers to listen to joy and keyboard messages
        this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_str, 10, std::bind(&Mux::joy_callback, this, std::placeholders::_1));
        this->create_subscription<std_msgs::msg::String>(keyboard_topic_str, 10, std::bind(&Mux::key_callback, this, std::placeholders::_1));

        // get mux indices
        this->declare_parameter("joy_mux_idx");
        joy_mux_idx_int = this->get_parameter("joy_mux_idx").as_int();
        this->declare_parameter("key_mux_idx");
        key_mux_idx_int = this->get_parameter("key_mux_idx").as_int();

        // get params for joystick calculations
        this->declare_parameter("joy_speed_axis");
        joy_speed_axis_int = this->get_parameter("joy_speed_axis").as_int();
        this->declare_parameter("joy_angle_axis");
        joy_angle_axis_int = this->get_parameter("joy_angle_axis").as_int();

        this->declare_parameter("max_steering_angle");
        max_steering_angle_double = this->get_parameter("max_steering_angle").as_double();
        this->declare_parameter("max_speed");
        max_speed_double = this->get_parameter("max_speed").as_double();

        // get params for keyboard driving
        this->declare_parameter("keyboard_speed");
        keyboard_speed_double = this->get_parameter("keyboard_speed").as_double();
        this->declare_parameter("keyboard_steer_ang");
        keyboard_steer_ang_double = this->get_parameter("keyboard_steer_ang").as_double();

        // get size of mux
        this->declare_parameter("mux_size");
        mux_size_int = this->get_parameter("mux_size").as_int();

        // initialize mux controller
        mux_controller.reserve(mux_size_int);
        prev_mux.reserve(mux_size_int);
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        // A channel contains a subscriber to the given drive topic and a publisher to the main drive topic
        channels = std::vector<Channel*>();

        /// Add new channels here:
        // Random driver example
        this->declare_parameter("random_walker_mux_idx");
        int random_walker_mux_idx_int = this->get_parameter("random_walker_mux_idx").as_int();
        this->declare_parameter("rand_drive_topic");
        std::string rand_drive_topic_str = this->get_parameter("rand_drive_topic").as_string();
        add_channel(rand_drive_topic_str, drive_topic_str, random_walker_mux_idx_int);

        // Channel for emergency braking
        this->declare_parameter("brake_mux_idx");
        int brake_mux_idx_int = this->get_parameter("brake_mux_idx").as_int();
        this->declare_parameter("brake_drive_topic");
        std::string brake_drive_topic_str = this->get_parameter("brake_drive_topic").as_string();
        add_channel(brake_drive_topic_str, drive_topic_str, brake_mux_idx_int);

        // General navigation channel
        this->declare_parameter("nav_mux_idx");
        int nav_mux_idx_int = this->get_parameter("nav_mux_idx").as_int();
        this->declare_parameter("nav_drive_topic");
        std::string nav_drive_topic_str = this->get_parameter("nav_drive_topic").as_string();
        add_channel(nav_drive_topic_str, drive_topic_str, nav_mux_idx_int);

        // ***Add a channel for a new planner here**
        // int new_mux_idx;
        // std::string new_drive_topic;
        // n.getParam("new_drive_topic", new_drive_topic);
        // n.getParam("new_mux_idx", new_mux_idx);
        // add_channel(new_drive_topic, drive_topic, new_mux_idx);
    }

    void add_channel(std::string channel_name, std::string drive_topic, int mux_idx_) {
        Channel* new_channel = new Channel(channel_name, drive_topic, mux_idx_, this);
        channels.push_back(new_channel);
    }

    void publish_to_drive(double desired_velocity, double desired_steer) {
        // This will take in a desired velocity and steering angle and make and publish an
        // Ackermann_drive_stamped message to the /drive topic

        // Make and publish message
        ackermann_msgs::msg::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::msg::AckermannDrive drive_msg;
        std_msgs::msg::Header header;
        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;
        header.stamp = clock->now();

        drive_st_msg.header = header;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub->publish(drive_st_msg);
    }

    void mux_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        // reset mux member variable every time it's published
        for (int i = 0; i < mux_size_int; i++) {
            mux_controller[i] = bool(msg->data[i]);
        }

        // Prints the mux whenever it is changed
        bool changed = false;
        // checks if nothing is on
        bool anything_on = false;
        for (int i = 0; i < mux_size_int; i++) {
            changed = changed || (mux_controller[i] != prev_mux[i]);
            anything_on = anything_on || mux_controller[i];
        }
        if (changed) {
            std::cout << "MUX: " << std::endl;
            for (int i = 0; i < mux_size_int; i++) {
                std::cout << mux_controller[i] << std::endl;
                prev_mux[i] = mux_controller[i];
            }
            std::cout << std::endl;
        }
        if (!anything_on) {
            // if no mux channel is active, halt the car
            publish_to_drive(0.0, 0.0);
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // make drive message from joystick if turned on
        if (mux_controller[joy_mux_idx_int]) {
            // Calculate desired velocity and steering angle
            double desired_velocity = max_speed_double * msg->axes[joy_speed_axis_int];
            double desired_steer = max_steering_angle_double * msg->axes[joy_angle_axis_int];

            publish_to_drive(desired_velocity, desired_steer);
        }
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        // make drive message from keyboard if turned on
        if (mux_controller[key_mux_idx_int]) {
            // Determine desired velocity and steering angle
            double desired_velocity = 0.0;
            double desired_steer = 0.0;

            bool publish = true;

            if (msg->data == "w") {
                // Forward
                desired_velocity = keyboard_speed_double; // a good speed for keyboard control
            } else if (msg->data == "s") {
                // Backwards
                desired_velocity = -keyboard_speed_double;
            } else if (msg->data == "a") {
                // Steer left and keep speed
                desired_steer = keyboard_steer_ang_double;
                desired_velocity = prev_key_velocity;
            } else if (msg->data == "d") {
                // Steer right and keep speed
                desired_steer = -keyboard_steer_ang_double;
                desired_velocity = prev_key_velocity;
            } else if (msg->data == " ") {
                // publish zeros to slow down/straighten out car
            } else {
                // so that it doesn't constantly publish zeros when you press other keys
                publish = false;
            }

            if (publish) {
                publish_to_drive(desired_velocity, desired_steer);
                prev_key_velocity = desired_velocity;
            }
        }
    }


};


/// Channel class method implementations

Channel::Channel() : Node("channel") {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Channel intialized without proper information");
    Channel("", "", -1, nullptr);
}

Channel::Channel(std::string channel_name_str, std::string drive_topic_str, int mux_idx_, Mux* mux)
: Node("channel"), mux_idx(mux_idx_), mp_mux(mux) {
    drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_str, 10);
    this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(channel_name_str, 10, std::bind(&Channel::drive_callback, this, std::placeholders::_1));
}

void Channel::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    if (mp_mux->mux_controller[this->mux_idx]) {
        drive_pub->publish(*msg.get());
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mux>());
    rclcpp::shutdown();
    return 0;
}
