#include <fenv.h>
#include "config.h"
#include "MochaGui.h"

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
    google::InitGoogleLogging(argv[0]);

    ros::init(argc, argv, "mochagui");

    MochaGui *pGui = MochaGui::GetInstance();
    pGui->Init();
    pGui->Run();

    delete pGui;
    return 0;
}
