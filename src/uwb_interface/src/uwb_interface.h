#ifndef UWB_INTERFACE_H
#define UWB_INTERFACE_H

#include <ros/ros.h>
#include <list>

namespace uwb_interface{

    void ParseOptions(int argc, char **argv);
    void Initialize(int argc, char **argv);
    void Update();
    void Cleanup();
    bool check_setup_serial();
    void publish_serial_data(char data[], int buffer_length);
}



#endif