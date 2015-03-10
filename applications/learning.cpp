#include "config.h"
#include <fenv.h>
#include "GetPot"
#include "LearningGui.h"

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    GetPot cl( argc, argv );

    std::string sMesh = cl.follow("jump.blend",1,"-mesh");
    bool vicon = cl.search("-vicon");
    std::string sRef = cl.follow( "", 1, "-ref" );
    std::string sMode = cl.follow( "Simulation", 1, "-mode" );
    Mode eMode = Mode_Simulation;
    if(sMode == "Experiment"){
        eMode = Mode_Experiment;
    }
    LearningGui* pGui = LearningGui::GetInstance();
    pGui->Init(sRef,sMesh,vicon,eMode);
    pGui->Run();
    delete pGui;
    return 0;
}

