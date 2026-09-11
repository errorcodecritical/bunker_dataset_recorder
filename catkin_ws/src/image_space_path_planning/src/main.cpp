#include "image_space_path_planning/node_interface.hpp"

#include <chrono>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_space_path_planning_node");
    ros::NodeHandle nh;

    NodeInterface node(nh);

    ros::AsyncSpinner spinner(1);  // or 2+ threads
    spinner.start();

    // int counter = 0;

    ros::Rate loop_rate(6);
    while (ros::ok()) {
        // auto loop_start = std::chrono::high_resolution_clock::now();

        node.run();

        // auto loop_run = std::chrono::high_resolution_clock::now();

        loop_rate.sleep();

        // auto loop_end = std::chrono::high_resolution_clock::now();

        // std::chrono::duration<double> loop_duration = loop_end - loop_start;
        // std::chrono::duration<double> run_duration = loop_run - loop_start;

        // std::cout << "[Timing] node.run() took " << run_duration.count() * 1000.0 << " ms"
        //           << " (" << 1.0 / loop_duration.count() << " Hz with sleep)" << std::endl;
    }

    return 0;
}