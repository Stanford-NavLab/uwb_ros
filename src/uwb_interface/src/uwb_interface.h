#ifndef UWB_INTERFACE_H
#define UWB_INTERFACE_H

#include <list>

#include <typeinfo>
#include <iostream>
#include <algorithm>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>            // enables ROS logging
#include <std_msgs/Float64.h>
#include <uwb_interface/UWBRange.h> // custom UWB message

// C library headers
#include <stdio.h> // printf, NULL
#include <string>
#include <sstream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/file.h>


#include <bits/stdc++.h> //split serial input, strok()
#include <stdlib.h>     // strtod, strtol
#include <chrono>

#include <sys/stat.h>

namespace uwb_interface{

    void ParseOptions(int argc, char **argv);
    void Initialize(int argc, char **argv);
    void Update();
    void Cleanup();
    bool check_setup_serial();
    std::string get_topic_name(std::string anchor_address, std::string tag_address);
    void publish_serial_data(char data[], int buffer_length);
}

#endif
