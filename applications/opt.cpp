#include <thread>
#include <iomanip>
#include <fenv.h>
#include "CarPlanner/LocalPlanner.h"
#include "CarPlanner/CarController.h"

#include "config.h"
#include "MochaGui/GetPot"
#include "MochaGui/PlannerGui.h"


static int g_nIterationLimit = 10;
static double g_dT =  0.01;
static bool g_bInertialControlActive = false;
static bool g_bPointCost = false;
static double g_dSuccessNorm =  1e-10;


bool CanContinue(const bool bPaused, bool& bStep){
    if(bPaused == true){
        if(bStep == true){
            bStep = false;
            return true;
        }else{
            return false;
        }
    }else{
        return true;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
    g_dSuccessNorm = 1e-10;
    bool bPaused = false, bStep = false;
    bool bPlannerOn = true, bControllerOn = true;
    float fLookaheadTime = 1.5, fStartCurvature = 0.0;
    double dNorm = 0, dActualLookahead = fLookaheadTime;

    GetPot cl( argc, argv );

    //std::string sMesh = cl.follow("CU_luma.ply",1,"-mesh");

    //std::string sMesh = cl.follow("jump.blend",1,"-mesh");
    std::string sMesh = cl.follow("lab.ply",1,"-mesh");
    bool bLocalizer = cl.search("-localizer");

    GLMesh terrainMesh;
    GLAxis endPosAxis;
    LocalPlanner planner;
    //initialize the Gui
    PlannerGui gui;
    BulletCarModel carModel;
    GLCachedPrimitives trajectoryStrip,trajectoryBezierStrip,controlStrip,controlBezierStrip;
    trajectoryStrip.SetColor(GLColor(0.5f,0.0f,0.0f));
    trajectoryBezierStrip.SetColor(GLColor(0.8f,0.8f,0.8f));
    controlStrip.SetColor(GLColor(0.0f,0.5f,0.0f));
    controlBezierStrip.SetColor(GLColor(0.8f,0.8f,0.8f));
    gui.Init(sMesh,&terrainMesh,bLocalizer);

    gui.AddPanel(&panel);

    panel.SetVar("planner:PlannerOn",&bPlannerOn);
    panel.SetVar("control:LookaheadTime",&fLookaheadTime);
    panel.SetVar("control:StartCurvature",&fStartCurvature);
    panel.SetVar("planner:ControllerOn",&bControllerOn);
    panel.SetVar("planner:ActualLookahead",&dActualLookahead);
    panel.SetVar("planner:Norm",&dNorm);
    panel.SetVar("interface:Paused",&bPaused);


    //add some waypoints
    Eigen::Vector6d wp0Pose, wp1Pose, wp2Pose;
    wp0Pose << 0,1.5,0,0,0,0;
    gui.AddWaypoint(wp0Pose,1.0);
    wp1Pose << 3,1.5,0,0,0,0;
    gui.AddWaypoint(wp1Pose,1.0);
    wp2Pose << 0,2,0,0,0,0;
    gui.AddWaypoint(wp2Pose,1.0);

    GLWayPoint* pWaypoint0 = &(gui.GetWaypoint(0)->m_Waypoint);
    GLWayPoint* pWaypoint1 = &(gui.GetWaypoint(1)->m_Waypoint);
    GLWayPoint* pCarWaypoint = &(gui.GetWaypoint(2)->m_Waypoint);

    //register keystrokes
    pangolin::RegisterKeyPressCallback( ' ', [&bPaused] {bPaused = !bPaused;} );
    pangolin::RegisterKeyPressCallback( '\r', [&bStep] {bStep = true;} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 's', [&] {pCarWaypoint->SetPose(pWaypoint0->GetPose()); pCarWaypoint->SetDirty(true);} );
    pangolin::RegisterKeyPressCallback( PANGO_CTRL + 'r', [&] { pWaypoint0->SetPose(wp0Pose); pWaypoint0->SetDirty(true); pWaypoint1->SetPose(wp1Pose); pWaypoint1->SetDirty(true); pCarWaypoint->SetPose(wp2Pose); pCarWaypoint->SetDirty(true);} );
    pangolin::RegisterKeyPressCallback( PANGO_SPECIAL+PANGO_KEY_RIGHT, [&] {pCarWaypoint->SetPose(pCarWaypoint->GetPose()[0],pCarWaypoint->GetPose()[1]+0.01,pCarWaypoint->GetPose()[2],pCarWaypoint->GetPose()[3],pCarWaypoint->GetPose()[4],pCarWaypoint->GetPose()[5]); pCarWaypoint->SetDirty(true);} );
    pangolin::RegisterKeyPressCallback( PANGO_SPECIAL+PANGO_KEY_LEFT, [&] {pCarWaypoint->SetPose(pCarWaypoint->GetPose()[0],pCarWaypoint->GetPose()[1]-0.01,pCarWaypoint->GetPose()[2],pCarWaypoint->GetPose()[3],pCarWaypoint->GetPose()[4],pCarWaypoint->GetPose()[5]); pCarWaypoint->SetDirty(true);} );
    pangolin::RegisterKeyPressCallback( PANGO_SPECIAL+PANGO_KEY_UP, [&] {pCarWaypoint->SetPose(pCarWaypoint->GetPose()[0]+0.01,pCarWaypoint->GetPose()[1],pCarWaypoint->GetPose()[2],pCarWaypoint->GetPose()[3],pCarWaypoint->GetPose()[4],pCarWaypoint->GetPose()[5]); pCarWaypoint->SetDirty(true);} );
    pangolin::RegisterKeyPressCallback( PANGO_SPECIAL+PANGO_KEY_DOWN, [&] {pCarWaypoint->SetPose(pCarWaypoint->GetPose()[0]-0.01,pCarWaypoint->GetPose()[1],pCarWaypoint->GetPose()[2],pCarWaypoint->GetPose()[3],pCarWaypoint->GetPose()[4],pCarWaypoint->GetPose()[5]); pCarWaypoint->SetDirty(true);} );

    CarParameterMap params;
    //load te parameters
    CarParameters::LoadFromFile("../gui_params.csv",params);
    carModel.Init((const struct aiScene *)terrainMesh.GetScene(),params,LocalPlanner::GetNumWorldsRequired(OPT_DIM));

    gui.AddGLObject(&trajectoryStrip);
    gui.AddGLObject(&trajectoryBezierStrip);
    gui.AddGLObject(&controlStrip);
//    gui.AddGLObject(&controlBezierStrip);
    gui.AddGLObject(&endPosAxis);

    //holds all reference segments
    std::vector<MotionSample> vSegmentSamples;
    vSegmentSamples.push_back(MotionSample());
    //control plan structure
    ControlPlan controlPlan;
    controlPlan.m_nStartSampleIndex = 0;
    controlPlan.m_nStartSegmentIndex = 0;

    int nNumIterations = 0;

    //this lambda expression captures all local variables by REFERENCE, therefore
    //it cannot go out of local scope (which it won't as we are in main)
//    std::thread* pWorkThread = new std::thread([&] {
//    bool bComputeNewPath = false;
//    while(1){
//        usleep(1000);
//        if(CanContinue(bPaused,bStep) == true){
//            //first, solve the waypoints if we need to
//            if(pWaypoint0->GetDirty() || pWaypoint1->GetDirty()){
//                bComputeNewPath = true;
//                ApplyVelocitesFunctor5d functor(&carModel,Eigen::Vector3d::Zero());

//                //if the waypoints have been moved, raycast them to improve ground contact
//                Eigen::Vector3d dIntersect;
//                Sophus::SE3d pose(pWaypoint0->GetPose4x4_po());
//                if(carModel.RayCast(pose.translation(),GetBasisVector(pose,2)*0.2,dIntersect,true)){
//                    pose.translation() = dIntersect;
//                    pWaypoint0->SetPose(pose.matrix());
//                }

//                pose = Sophus::SE3d(pWaypoint1->GetPose4x4_po());
//                if(carModel.RayCast(pose.translation(),GetBasisVector(pose,2)*0.2,dIntersect,true)){
//                    pose.translation() = dIntersect;
//                    pWaypoint1->SetPose(pose.matrix());
//                }

//                VehicleState startState(Sophus::SE3d(pWaypoint0->GetPose4x4_po()),pWaypoint0->GetVelocity(),0);
//                VehicleState goalState(Sophus::SE3d(pWaypoint1->GetPose4x4_po()),pWaypoint1->GetVelocity(),0);
//                //now re-solve the path between these two
//                LocalProblem problem(&functor,startState,goalState,g_dT);
//                planner.InitializeLocalProblem(problem,0,NULL,eCostPoint);
//                problem.m_bInertialControlActive = g_bInertialControlActive;

//                //now iterate the planner
//                while(1){
//                    usleep(1000);
//                    if(CanContinue(bPaused,bStep) == true){
//                        MotionSample* pSample;
//                        bool bRes;
//                        if(bPlannerOn){
//                            bRes = planner.Iterate(problem);
//                            pSample = &problem.m_CurrentSolution.m_Sample;
//                            dNorm = problem.m_CurrentSolution.m_dNorm;
//                        }else{
//                            bRes = true;
//                            planner.SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem);
//                            pSample = &problem.m_CurrentSolution.m_Sample;
//                            dNorm = 0;
//                        }


//                        Eigen::Vector3dAlignedVec vPts;
//                        for(const VehicleState& state : pSample->m_vStates) {
//                            vPts.push_back(state.m_dTwv.translation());
//                        }
//                        trajectoryStrip.AddVerticesFromTrajectory(vPts);

//                        planner.SamplePath(problem,vPts);
//                        trajectoryBezierStrip.AddVerticesFromTrajectory(vPts);

//                        nNumIterations++;
//                        vSegmentSamples[0] = *pSample;

//                        if(bRes == true || nNumIterations > g_nIterationLimit){
//                            nNumIterations = 0;
//                            pWaypoint0->SetDirty(false);
//                            pWaypoint1->SetDirty(false);
//                            break;
//                        }
//                    }
//                }
//            }
//            else if(bComputeNewPath || pCarWaypoint->GetDirty() == true){
//                Eigen::Vector3d dIntersect;
//                Sophus::SE3d pose(pCarWaypoint->GetPose4x4_po());
//                if(carModel.RayCast(pose.translation(),GetBasisVector(pose,2)*0.2,dIntersect,true)){
//                    pose.translation() = dIntersect;
//                    pCarWaypoint->SetPose(pose.matrix());
//                }

//                controlPlan.m_StartState = VehicleState(Sophus::SE3d(pCarWaypoint->GetPose4x4_po()),pCarWaypoint->GetVelocity(),fStartCurvature);
//                //first find the closest point on the reference trajectory
//                CarController::AdjustStartingSample(vSegmentSamples,controlPlan.m_StartState,controlPlan.m_nStartSegmentIndex,controlPlan.m_nStartSampleIndex);

//                MotionSample trajectorySample;
//                VelocityProfile profile;
//                //prepare the trajectory ahead
//                CarController::PrepareLookaheadTrajectory(vSegmentSamples,&controlPlan,profile,trajectorySample,fLookaheadTime);

//                ApplyVelocitesFunctor5d functor(&carModel,Eigen::Vector3d::Zero(), NULL);
//                functor.SetNoDelay(true);
//                LocalProblem problem(&functor,controlPlan.m_StartState,controlPlan.m_GoalState,g_dT);
//                problem.m_CurrentSolution.m_dMinTrajectoryTime = fLookaheadTime;
//                problem.m_vVelProfile = profile;
//                planner.InitializeLocalProblem(problem,controlPlan.m_dStartTime,&problem.m_vVelProfile,g_bPointCost ? eCostPoint : eCostTrajectory);
//                problem.m_Trajectory = trajectorySample;

//                //now iterate the planner
//                while(1){
//                    usleep(1000);
//                    if(CanContinue(bPaused,bStep) == true){
//                        MotionSample* pSample;
//                        bool bRes;
//                        if(bControllerOn){
//                            bRes = planner.Iterate(problem);
//                            pSample = &problem.m_CurrentSolution.m_Sample;
//                            dNorm = problem.m_CurrentSolution.m_dNorm;
//                            dActualLookahead = problem.m_CurrentSolution.m_dMinTrajectoryTime;
//                        }else{
//                            bRes = true;
//                            planner.SimulateTrajectory(problem.m_CurrentSolution.m_Sample,problem);
//                            pSample = &problem.m_CurrentSolution.m_Sample;
//                            dNorm = 0;
//                            dActualLookahead = fLookaheadTime;
//                        }

//                        //set the axis position denoting the end of the sample
//                        endPosAxis.SetPose(pSample->m_vStates.back().m_dTwv.matrix());
//                        endPosAxis.SetAxisSize(pSample->m_vStates.back().m_dV.norm());


//                        Eigen::Vector3dAlignedVec vPts;
//                        for(const VehicleState& state : pSample->m_vStates) {
//                            vPts.push_back(state.m_dTwv.translation());
//                        }
//                        controlStrip.AddVerticesFromTrajectory(vPts);

//                        planner.SamplePath(problem,vPts);
//                        controlBezierStrip.AddVerticesFromTrajectory(vPts);

//                        nNumIterations++;

//                        if(bRes == true || nNumIterations > g_nIterationLimit){
//                            bComputeNewPath = false;
//                            pCarWaypoint->SetDirty(false);
//                            nNumIterations = 0;
//                            break;
//                        }
//                    }
//                }
//            }
//        }
//    }
//    });


    while( !pangolin::ShouldQuit() )
    {
        {
            //std::unique_lock<std::mutex> lock(m_DrawMutex); //crh old boost code was commented
            gui.Render();
        }
    }

//    pWorkThread->join();

    return 0;
}

