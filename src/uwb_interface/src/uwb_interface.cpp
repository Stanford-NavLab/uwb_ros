#include "uwb_interface.h"
#include <typeinfo>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <uwb_interface/RangeEvent.h>

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

    
    double ros_rate = 100.0;
    ros::Publisher uwb_range_pub;
    uwb_interface::RangeEvent range_msg;

    std::string uwb_port = "";
    int serial_port;
    bool serial_configured = false;
    bool port_open = false;
    
    
    
    void ParseOptions(int argc, char **argv){

        int c;
        bool uwb_port_set = false;
        
        while((c = getopt(argc, argv, "p:")) != -1){
            switch(c){
                case 'p':
                    uwb_port = optarg;    
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

    inline bool serialExists (const std::string& name) {
        struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
    }

    void Initialize(int argc, char **argv){

        std::string port = uwb_port;
        std::replace( port.begin(), port.end(), '/', '_');
        //initialize ROS
        ros::init(argc, argv, "uwb_interface_node" + port + "_" + std::to_string(getTimeMicro()));
        ros::NodeHandle nh;

        uwb_range_pub = nh.advertise<uwb_interface::RangeEvent>("uwb/range", 10);
   
    }

    void Cleanup(){
        if(port_open){
            close(serial_port);
            port_open = false;
        }
    }

    void Update(){

        if(serial_configured){

            // Allocate memory for read buffer, set size according to your needs
            int buffer_length = 500;
            int pos = 0;
            char read_buf [buffer_length];
            memset(&read_buf, '\0', sizeof(read_buf));

            int n;

            // std::cout << "read" << std::endl;
            // Read bytes.
            while(pos < buffer_length){
                n = read(serial_port, read_buf+pos, 1);
                if (n != 0){
                    if( read_buf[pos] == '\n' ) break;
                    pos++;
                }
            }

            //process and publish the serial data 
            if(n > 0){
                publish_serial_data(read_buf, pos+1);
            }
            if(n<0){
                std::cout << "serial read error" << std::endl;
                serial_configured = false;
            }
            
        }else{
            check_setup_serial();
        }
    }
    
    bool check_setup_serial(){
        
        bool serial_okay = true;

        if(port_open){
            close(serial_port);
            port_open = false;
            serial_configured = false;
        }

        bool serial_path_exists = serialExists(uwb_port);

        if(serial_path_exists){

            //setup the serial port
            //see the following link for more information
            //https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#overview
            // Acquire non-blocking exclusive lock
            serial_port = open((const char*)uwb_port.c_str(), O_RDWR);
            port_open = true;

            // Check for errors
            if (serial_port < 0) {
                printf("Error %i from open: %s\n", errno, strerror(errno));
                serial_okay = false;
                serial_configured = false;
            }

            if(serial_okay){
                if(flock(serial_port, LOCK_EX | LOCK_NB) == -1) {
                    throw std::runtime_error("Serial port with file descriptor " + 
                        std::to_string(serial_port) + " is already locked by another process.");
                    serial_okay = false;
                }
            }

            if(serial_okay && !serial_configured)
            {
                // Create new termios struc, we call it 'tty' for convention
                struct termios tty;
                memset(&tty, 0, sizeof tty);

                // Read in existing settings, and handle any error
                if(tcgetattr(serial_port, &tty) != 0) {
                    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
                    serial_okay = false;
                }
                
                tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
                tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
                tty.c_cflag |= CS8; // 8 bits per byte (most common)
                tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
                tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
                tty.c_lflag &= ~ICANON; //Canonical mode is disabled
                tty.c_lflag &= ~ECHO; // Disable echo
                tty.c_lflag &= ~ECHOE; // Disable erasure
                tty.c_lflag &= ~ECHONL; // Disable new-line echo
                tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
                tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl ????
                tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
                tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
                tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
                tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
                tty.c_cc[VMIN] = 0;

                // Set in/out baud rate
                // cfsetispeed(&tty, B9600);
                // cfsetospeed(&tty, B9600);
                cfsetispeed(&tty, B115200);
                cfsetospeed(&tty, B115200);
            
                // Save tty settings, also checking for error
                if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                    serial_okay = false;
                }
                
                //clear any data in the serial buffer //TODO make sure this works as expected!
                sleep(2);
                if (tcsetattr(serial_port, TCSAFLUSH, &tty) != 0) {
                    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
                    serial_okay = false;
                }

                if(serial_okay){
                    serial_configured = true;
                }
            }

        }else{
            serial_okay = false;
        }

        

        if(!serial_okay){
            serial_configured = false;
        }

        return serial_okay;
    }

    void publish_serial_data(char data[], int buffer_length)
    {
        //the UWB sends a single string with the following information over USB for each range calculation:
        //the values are hexidecimal encoded and space separated.
        //source address                (16 character)
        //ranging anchor address        (16 characters)
        //ranging tag address           (16 characters)
        //range (distance corrected)    (8 characters)
        //range (RSL corrected)         (8 characters)
        //range (raw)                   (8 characters)
        //RSL                           (8 characters)


        //The values are in contained in a single string separated by spaces

        //replace any carriage return (\r) in the character array with a space
        char from = '\r';   
        char to = ' ';
        for(int i=0; i<buffer_length; i++)
        {
            if(data[i] == from)
            {
                data[i] = to;
            }
        }

      
        char *token = strtok(data, " "); 
        

        int idx = 0;

        while (token != NULL) 
        {
            switch(idx)
            {
                case 0: //check message source address
                    if(!(strlen(token) == 4 || strlen(token) == 16)){
                        // printf("source address wrong length: %s \n", token);
                        return;
                    }
                    range_msg.source_address = token;
                    break;
                case 1: //anchor address
                    if(!(strlen(token) == 4 || strlen(token) == 16)){
                        // printf("anchor address wrong length: %s \n", token);
                        return;
                    }                     
                    range_msg.anchor_address = token;
            
                    break;
                case 2: //tag address
                    if(!(strlen(token) == 4 || strlen(token) == 16)){
                        // printf("tag address wrong length: %s \n", token);
                        return;
                    } 
                    range_msg.tag_address = token; 
                    
                    break;
                case 3: //distance corrected range
                    
                    if(strlen(token) != 8){
                        // printf("distance corrected range data size incorrect: %s length: %lu \n", token, strlen(token));
                        return;
                    }                
                    range_msg.range_dist = (int32_t)strtol(token, 0, 16);
                    
                    break;
                case 4: //rsl corrected range                     
                    if(strlen(token) != 8){
                        // printf("rsl corrected range data size incorrect: %s length: %lu \n", token, strlen(token));
                        return;
                    }                 
                    range_msg.range_rsl = (int32_t)strtol(token, 0, 16);
                    
                    break;
                case 5: //raw range
                    
                    if(strlen(token) != 8){
                        // printf("raw range data size incorrect: %s length: %lu \n", token, strlen(token));
                        return;
                    }                  
                    range_msg.range_raw = (int32_t)strtol(token, 0, 16);
                    
                    break;
                case 6: //rsl
                    
                    if(strlen(token) != 8){
                        // printf("rsl data size incorrect: %s length: %lu \n", token, strlen(token));
                        return;
                    }                  
                    range_msg.rsl = ((float)((int32_t)strtol(token, 0, 16)))/1000.0;

                    break;
                default:
                    token = NULL; //stop processing the serial data
                    break;
            }

            token = strtok(NULL, " ");
            idx++; 
        }

        //add timestamp to message
        const auto p1 = std::chrono::system_clock::now();
        uint64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                p1.time_since_epoch()).count();
        range_msg.timestamp = nanoseconds;

        uwb_range_pub.publish(range_msg);
    }
}
