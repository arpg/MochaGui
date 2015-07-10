#include "config.h"
#include <fenv.h>
#include "MochaGui/GetPot"
#include "MochaGui/learning/LearningGui.h"

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  GetPot cl( argc, argv );

  std::string sMesh = cl.follow("lab.ply",1,"-mesh");
  bool localizer = cl.search("-localizer");
  std::string sRef = cl.follow( "", 1, "-ref" );
  std::string sMode = cl.follow( "Simulation", 1, "-mode" );
  Mode eMode = Mode_Simulation;
  if(sMode == "Experiment"){
    std::cerr << "Setting experimental mode."  << std::endl;
    eMode = Mode_Experiment;
  }
  LearningGui* pGui = LearningGui::GetInstance();
  pGui->Init(sRef,sMesh,localizer,eMode);
  pGui->Run();
  delete pGui;
  return 0;
}

