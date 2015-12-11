#include "config.h"
#include <fenv.h>
#include "MochaGui/learning/LearningGui.h"

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{

  std::string sMesh = "/Users/crh/data/lab.ply";
  bool localizer = false;
  std::string sRef = "";
  std::string sMode = "Simulation";
  Mode eMode = Mode_Simulation;
  if(sMode == "Experiment"){
    LOG(INFO) << "Setting experimental mode.";
    eMode = Mode_Experiment;
  }
  LearningGui* pGui = LearningGui::GetInstance();
  pGui->Init(sRef,sMesh,localizer,eMode);
  pGui->Run();
  delete pGui;
  return 0;
}

