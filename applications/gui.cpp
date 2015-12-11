
#include "config.h"
#include "GetPot"
#include "MochaGui.h"
#include <fenv.h>

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv ) 
{
    //feenableexcept(FE_DIVBYZERO|/*FE_UNDERFLOW|FE_OVERFLOW|*/FE_INVALID);
    GetPot cl( argc, argv );

    std::string sMesh = cl.follow("/Users/crh/data/lab.ply",1,"-mesh");
    bool localizer = cl.search("-localizer");
    std::string sRef = cl.follow( "", 1, "-ref" );
    std::string sMode = cl.follow( "Simulation", 1, "-mode" );
    std::string logFile = "logs/" + cl.follow( "", 1, "-log");

    MochaGui *pGui = MochaGui::GetInstance();
    pGui->Init(sRef,sMesh,localizer,sMode,logFile);
    pGui->Run();

    delete pGui;
    return 0;
}

