#include "kernel_msg.h"
#include <typeinfo>
#include <iostream>
#include <algorithm>
#include <sys/ioctl.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <uwb_interface/serial_event.h>

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

#include <sys/types.h>
#include <sys/socket.h>
#include <linux/netlink.h>



namespace uwb_interface{

    
    double ros_rate = 100.0;
    ros::Publisher serial_event_pub;
    uwb_interface::serial_event serial_event_msg;

    std::string uwb_port = "";
    
    int nl_socket;
    char msg[NL_MAX_PAYLOAD];
    
    
    
    void ParseOptions(int argc, char **argv){

        int c;
        bool uwb_port_set = false;
        
        while((c = getopt(argc, argv, "p:")) != -1){
            switch(c){
                case 'p':
                    uwb_port = optarg; 
                    uwb_port = uwb_port.substr(4, uwb_port.length() - 1);   
                    uwb_port_set = true;
                    break;
                default:
                    fprintf(stderr, "unrecognized option %c\n", optopt);
                    break;
            }
        }

        if(!uwb_port_set){
            std::cout << "UWB serial port not set. Run with -p <serial/port/path>" << std::endl;
        }
        if(!uwb_port_set){
            exit(EXIT_FAILURE);   
        }

    }
    
    uint64_t getTimeMicro()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    void Initialize(int argc, char **argv){

        std::string port = uwb_port;
        std::replace( port.begin(), port.end(), '/', '_');

        //initialize ROS
        ros::init(argc, argv, "kernel_msg_node" + port + "_" + std::to_string(getTimeMicro()));
        ros::NodeHandle nh;

        serial_event_pub = nh.advertise<uwb_interface::serial_event>("uwb/serial_event", 10); 
        serial_event_msg.port = uwb_port;

        struct sockaddr_nl src_addr;
        int ret;

        // Prepare source address
        memset(&src_addr, 0, sizeof(src_addr));
        src_addr.nl_family = AF_NETLINK;
        src_addr.nl_pid = getpid();
        src_addr.nl_groups = NETLINK_KOBJECT_UEVENT;

        nl_socket = socket(AF_NETLINK, (SOCK_DGRAM | SOCK_CLOEXEC), NETLINK_KOBJECT_UEVENT);
        if (nl_socket < 0) {
            printf("Failed to create socket for DeviceFinder");
            exit(1);
        }

        ret = bind(nl_socket, (struct sockaddr*) &src_addr, sizeof(src_addr));
        if (ret) {
            printf("Failed to bind netlink socket..");
            close(nl_socket);
        }   
    }

    void Cleanup(){

    }

    void Update(){

        int r = recv(nl_socket, msg, sizeof(msg), MSG_DONTWAIT);
        if (r >= 0){            
            std::string msg_string = msg;
            // std::cout << msg_string << std::endl;
            if (msg_string.find(uwb_port) != std::string::npos){
                if (msg_string.find("add") != std::string::npos){
                    // printf("add!\n");
                    publish_serial_event(1);
                }
                
                else if (msg_string.find("remove") != std::string::npos){
                    // printf("remove!\n");
                    publish_serial_event(0);
                }

            }

        }

    }
    

    void publish_serial_event(int event)
    {
        serial_event_msg.connection_event = event;
        serial_event_pub.publish(serial_event_msg);
    }
}
