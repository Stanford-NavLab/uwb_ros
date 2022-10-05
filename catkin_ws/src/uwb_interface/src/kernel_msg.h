#ifndef KERNEL_MSG_H
#define KERNEL_MSG_H

#include <ros/ros.h>
#include <list>



namespace uwb_interface{
    
    #define NL_MAX_PAYLOAD 8192

    void ParseOptions(int argc, char **argv);
    void Initialize(int argc, char **argv);
    void Update();
    void Cleanup();
    void publish_serial_event(int event);

}



#endif