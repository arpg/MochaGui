#include <fenv.h>
#include "config.h"
#include "MochaGui.h"

DEFINE_string(params, "", "File for car parameters.");		// /Users/corinsandford/Documents/ARPG/MochaGui/herbie/params.csv
DEFINE_string(mesh, "", "File for car parameters.");  		// /Users/corinsandford/Documents/ARPG/MochaGui/herbie/lab.ply
DEFINE_bool(localizer, false, "Whether or not to instantiate localization system.");
DEFINE_string(mode, "Simulation", "Mode: Experiment or Simulation (default).");
DEFINE_string(ref, "", "Reference plane for model if no triangle mesh.");
DEFINE_string(logfile, "", "Logfile for playing back.");	// /Users/corinsandford/Documents/ARPG/MochaGui/herbie/mocha_playback.log
DEFINE_string(car, "", "File for car mesh.");               // /Users/corinsandford/Documents/ARPG/MochaGui/herbie/herbie.ply
DEFINE_string(wheel,"", "File for wheel mesh.");            // /Users/corinsandford/Documents/ARPG/MochaGui/herbie/wheel.ply

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    if(FLAGS_params.empty()) {
        LOG(FATAL) << "Car parameters file not provided. Use parameter -params";
        return -1;
    }

    if(FLAGS_mesh.empty()) {
        LOG(FATAL) << "Mesh of environment not provided. Use parameter -mesh";
        return -1;
    }

    if(!FLAGS_localizer) {
        LOG(INFO) << "Localizer not initialized. To initialize, use parameter -localizer";
    }

    if(FLAGS_mode.empty()) {
        LOG(INFO) << "Mode not specified; assuming Simulation.";
    }

    if(FLAGS_ref.empty()) {
        LOG(INFO) << "Reference plane not specified.";
    }

    if(FLAGS_logfile.empty()) {
        LOG(FATAL) << "Missing required path to logfile for output.";
        return -1;
    }

    if(FLAGS_car.empty()) {
        LOG(FATAL) << "Mesh of car not provided. Use parameter -car";
        return -1;
    }

    if(FLAGS_wheel.empty()) {
        LOG(FATAL) << "Mesh of wheel not provided. Use parameter -wheel";
        return -1;
    }

    ros::init(argc, argv, "mochagui_node");

    MochaGui *pGui = MochaGui::GetInstance();
    pGui->Init( FLAGS_ref, FLAGS_mesh, FLAGS_localizer, FLAGS_mode, FLAGS_logfile, FLAGS_params, FLAGS_car, FLAGS_wheel );
    pGui->Run();

    delete pGui;
    return 0;
}

