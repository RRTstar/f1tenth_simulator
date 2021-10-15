#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <termios.h>

#include <stdio.h>
#include <signal.h>
#include <unistd.h>

// for printing
#include <iostream>

static volatile sig_atomic_t keep_running = 1;


void sigHandler(int /* not_used */ ) {
    keep_running = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("keyboard");

    // Initialize publisher
    node->declare_parameter("keyboard_topic");
    std::string keyboard_topic_str = node->get_parameter("keyboard_topic").as_string();

    auto key_pub = node->create_publisher<std_msgs::msg::String>(keyboard_topic_str, 10);


    static struct termios oldT, newT;
    tcgetattr( STDIN_FILENO, &oldT);
    newT = oldT;
    newT.c_lflag &= ~(ICANON);
    tcsetattr( STDIN_FILENO, 0, &newT);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigaction(SIGINT, &act, NULL);


    auto msg = std_msgs::msg::String();
    int c;
    while ((rclcpp::ok()) && (keep_running)) {
        // get the character pressed
        c = getchar();

        // Publish the character
        msg.data = c;
        key_pub->publish(msg);
    }

    tcsetattr( STDIN_FILENO, 0, &oldT);

    return 0;
}
