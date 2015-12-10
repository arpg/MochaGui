
#include "config.h"
#include "MochaGui/gui/MochaGui.h"
#include <fenv.h>

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv ) 
{
    //feenableexcept(FE_DIVBYZERO|/*FE_UNDERFLOW|FE_OVERFLOW|*/FE_INVALID);
    std::string sMesh = "/Users/crh/data/lab.ply";
    bool localizer = 0;
    std::string sRef = "";
    std::string sMode = "Simulation";
    std::string logFile = "/Users/crh/data/logs/log_test.txt";

    MochaGui *pGui = MochaGui::GetInstance();
    pGui->Init(sRef,sMesh,localizer,sMode,logFile);
    pGui->Run();

    delete pGui;
    return 0;
}

