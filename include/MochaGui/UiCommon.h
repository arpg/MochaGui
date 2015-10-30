#pragma once

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include "VarHelpers.h"
#define VICON_CAR_HEIGHT_OFFSET 0.02

struct PlannerHandler : public SceneGraph::HandlerSceneGraph
{
    PlannerHandler(SceneGraph::GLSceneGraph& graph, pangolin::OpenGlRenderState& cam_state, pangolin::AxisDirection enforce_up=pangolin::AxisNone, float trans_scale=0.01f) :
        HandlerSceneGraph(graph,cam_state,enforce_up,trans_scale)
    { }

    void Mouse(pangolin::View& view, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
    {
      HandlerSceneGraph::Mouse(view,button,x,y,pressed,button_state);
    }

    void MouseMotion(pangolin::View& view, int x, int y, int button_state)
    {
      HandlerSceneGraph::MouseMotion(view,x,y,button_state);
    }

};

struct ActivateScissorBlendedDrawFunctor
{
    ActivateScissorBlendedDrawFunctor(SceneGraph::GLObject& glObject, const pangolin::OpenGlRenderState& renderState)
        :glObject(glObject), renderState(renderState)
    {
    }

    void operator()(pangolin::View& view) {
        view.ActivateAndScissor(renderState);

        glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT);
        glColor4f(0.0,0.0,0.0,0.5);
        glBegin(GL_QUADS);                      // Draw A Quad
            glVertex3f(-1.0f, 1.0f, 0.0f);              // Top Left
            glVertex3f( 1.0f, 1.0f, 0.0f);              // Top Right
            glVertex3f( 1.0f,-1.0f, 0.0f);              // Bottom Right
            glVertex3f(-1.0f,-1.0f, 0.0f);              // Bottom Left
        glEnd();
        glPopAttrib();

        glObject.DrawObjectAndChildren(eRenderVisible);
    }

    SceneGraph::GLObject& glObject;
    const pangolin::OpenGlRenderState& renderState;
};


enum MochaCommands{
  eMochaPause = 1,
  eMochaSolve = 2,
  eMochaRestart = 3,
  eMochaStep = 4,
  eMochaLearn = 5,
  eMochaToggleTrajectory = 6,
  eMochaTogglePlans = 7,
  eMochaLoad = 8,
  eMochaClear = 9,
  eMochaFixGround = 10,
  eMochaPpmControl = 11,
  eMochaSimulationControl = 12,

  eMochaLeft = 90,
  eMochaRight = 91,
  eMochaUp = 92,
  eMochaDown = 93
};

struct Waypoint
{
    Waypoint(): m_eType(eWaypoint_Normal)
    {}
    SceneGraph::GLWayPoint m_Waypoint;
    WaypointType m_eType;
};

//inline std::ostream& operator<<( std::ostream& Stream, Waypoint& waypoint )
//{
//    //write the waypoint to file
//    Eigen::MatrixXd pose = waypoint.m_Waypoint.GetPose();
//    Stream << pose << "," << (int)waypoint.m_eType;
//    return Stream;
//}

//inline std::istream& operator>>( std::istream& Stream, Waypoint& waypoint )
//{
//    //read the waypoiint from file
//    Eigen::MatrixXd pose;
//    Stream >> pose;
//    Eigen::Vector6d vecPose = pose;
//    waypoint.m_Waypoint.SetPose(vecPose);

//    //now read the type
//    char str[256];
//    Stream.getline(str,255);
//    waypoint.m_eType = (WaypointType)std::strtol(str,NULL,10);
//}
