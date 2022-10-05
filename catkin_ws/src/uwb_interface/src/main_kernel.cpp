#include <iostream>
#include "kernel_msg.h"


using namespace uwb_interface;

int main(int argc, char **argv)
{
    std::cout << "kernel msg node started" << std::endl;
    ParseOptions(argc, argv);

    //need to initialize the modules
    Initialize(argc, argv);

    while(ros::ok()){
        Update();
    }

    Cleanup();

    std::cout << "kernel msg node stopped" << std::endl;

    return EXIT_SUCCESS;

}