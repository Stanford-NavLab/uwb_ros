#include <iostream>
#include "uwb_interface.h"

using namespace uwb_interface;


int main(int argc, char **argv)
{
    std::cout << "UWB interface node started" << std::endl;
    ParseOptions(argc, argv);

    //need to initialize the modules
    Initialize(argc, argv);

    while(ros::ok()){
        Update();
    }

    Cleanup();

    std::cout << "UWB interface node stopped" << std::endl;

    return EXIT_SUCCESS;
}