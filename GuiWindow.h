#ifndef GUIWINDOW_H
#define GUIWINDOW_H

#include <SimpleGui/GLWindow.h>
#include <Eigen/Eigen>


////////////////////////////////////////////////////////////////////////////
// Hermes node
/*
#include "../../Hermes1.0/Node.h"
#include "Messages.pb.h"

extern float g_fTurnrate;
extern float g_fSpeed;

typedef bool(*HandlerFuncPtr)(std::vector<std::string> *);
typedef bool(*CommandHandlerFuncPtr)(MochaCommands command);

using namespace rpg;



class GuiWindow: public GLWindow
{
public:
    GuiWindow(int x,int y,int w,int h,
              const char *l=0, Node *pNode = 0,const char *rpcAddress = 0,
              HandlerFuncPtr setWaypointsHandler = NULL, HandlerFuncPtr setPathHandler = NULL,
              HandlerFuncPtr mochaStartHandler = NULL, HandlerFuncPtr mochaStopHandler = NULL,
              CommandHandlerFuncPtr commandHandler = NULL) : GLWindow(x,y,w,h,l)
    {
        m_bIsSpacebarPressed = false;
        m_pNode = pNode;
        m_pRpcAddress = rpcAddress;
        
        m_pSetPathFuncPtr = setPathHandler;
        m_pSetWaypointsFuncPtr = setWaypointsHandler;
        m_pMochaStartFuncPtr = mochaStartHandler;
        m_pMochaStopFuncPtr = mochaStopHandler;
        m_pCommandHandler = commandHandler;
        m_dCamTarget << 0,0,0;
    }
    


private:
    HandlerFuncPtr                  m_pSetPathFuncPtr;
    HandlerFuncPtr                  m_pSetWaypointsFuncPtr;
    HandlerFuncPtr                  m_pMochaStartFuncPtr;
    HandlerFuncPtr                  m_pMochaStopFuncPtr;
    HandlerFuncPtr                  m_pPauseFuncPtr;
    HandlerFuncPtr                  m_pStepFuncPtr;
    CommandHandlerFuncPtr                  m_pCommandHandler;
    Eigen::Vector2d                 m_vMouseReference;
    bool                            m_bIsSpacebarPressed;
    double                          m_dControlDt;
    Node*                           m_pNode;
    const char *                    m_pRpcAddress;


    ////////////////////////////////////////////////////////////////////////////
    virtual int HandleNavInput( int e )
    {
        switch( e ){
        case FL_PUSH:
            m_nMousePushX = Fl::event_x();
            m_nMousePushY = Fl::event_y();
            return 1;
        case FL_MOUSEWHEEL:
        {
            int dy = Fl::event_dy();
            // move camera along vector from CamPostion to CamTarget
            Eigen::Vector3d d = m_dCamTarget - m_dCamPosition;
            d = d / d.norm();
            if( Fl::event_ctrl() ) {
                // up and down
                m_dCamTarget   += -dy*m_dCamUp;
                m_dCamPosition += -dy*m_dCamUp;
            } else {
                m_dCamPosition = m_dCamPosition - dy*d;
                m_dCamTarget = m_dCamPosition + d;
            }
            return 1;
        }
            break;
        case FL_DRAG:
        {
            int dx = Fl::event_x() - m_nMousePushX;
            int dy = Fl::event_y() - m_nMousePushY;
            if( Fl::event_button3() || Fl::event_state(FL_CTRL) ) {
                // "move" the world
                double th = std::max( fabs(m_dCamPosition[2] / 100.0), 0.01 ); // move more or less depending on height
                Eigen::Vector3d dForward = m_dCamTarget - m_dCamPosition;
                dForward = dForward / dForward.norm();
                Eigen::Vector3d dLR = dForward.cross(m_dCamUp);
                // sideways
                m_dCamTarget   += -dx * th*dLR;
                m_dCamPosition += -dx * th*dLR;
                // re-use "forward"
                dForward = m_dCamUp.cross(dLR);
                m_dCamTarget   += dy * th*dForward;
                m_dCamPosition += dy * th*dForward;
            } else {
                // just aim the camera -- we can write a better UI later...
                Eigen::Vector3d dForward = m_dCamTarget - m_dCamPosition;

                double yaw = atan2(dForward[1], dForward[0]);
                double pitch = atan2(-dForward[2], sqrt((dForward[0] * dForward[0]) + (dForward[1] * dForward[1])));
                Eigen::Matrix3d Rw_v1 = GLCart2R( 0, pitch,yaw );
                double dYaw = 0.001 * -dx;
                double dPitch = 0.001 * dy;

                //use lie algebra to impose an infinitesimal rotation
                //which is faster than calculating R from euler angles
                Eigen::Matrix3d lieT;
                lieT << 1, -dYaw, dPitch, dYaw, 1, 0, -dPitch, 0, 1;

                //rotate the forward vector to the new heading
                dForward << 1,0,0;
                dForward = Rw_v1*lieT*dForward;
                //dForward = dPitch*dForward;

                m_dCamTarget = m_dCamPosition + dForward;
            }
            m_nMousePushX = Fl::event_x();
            m_nMousePushY = Fl::event_y();
            return 1;
        }
            break;
        case FL_KEYBOARD:
            switch( Fl::event_key() ) {
            case '`':
                m_Console.OpenConsole();
                m_eGuiMode = eConsole;
                break;
            case FL_F+1:
                ResetCamera();
                break;
            case FL_F+2:
                CameraOrtho();
                break;
            case 't':
                if( Fl::event_state(FL_CTRL) ) {
                    if( m_pCommandHandler != NULL){
                        m_pCommandHandler(eMochaSolve);
                    }
                }
                break;

            case 'r':
                if( Fl::event_state(FL_CTRL) ) {
                    if( m_pCommandHandler != NULL){
                        m_pCommandHandler(eMochaRestart);
                    }
                }
                break;

            case ' ':
                if(m_bIsSpacebarPressed == false )
                {
                    m_vMouseReference(0) = Fl::event_x();
                    m_vMouseReference(1) = Fl::event_y();
                    m_bIsSpacebarPressed = true;
                }
                g_fSpeed = 0;
                g_fTurnrate = 0;


                if( m_pCommandHandler != NULL){
                    m_pCommandHandler(eMochaPause);
                }

                break;

            case FL_Enter:
                if( m_pCommandHandler != NULL){
                    m_pCommandHandler(eMochaStep);
                }
                break;


            case 'a': case 'A':
                g_fTurnrate -= 0.1;
                break;
            case 's': case 'S':
                g_fSpeed -= 0.1;
                break;
            case 'd': case 'D':
                g_fTurnrate += 0.1;
                break;
            case 'w': case 'W':
                g_fSpeed += 0.1;
                break;

            }
            return 1;
            break;

        case FL_KEYUP:
            switch( Fl::event_key() )
            {
            case ' ':
                m_bIsSpacebarPressed = false;
                break;
            }
            break;
        }
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    int HandleSelectionInput( int e )
    {
        m_nMousePushX = Fl::event_x();
        m_nMousePushY = Fl::event_y();
        switch ( e ) {
        case FL_PUSH:
        {
            GLObject* pSelect = SelectedObject();
            if( pSelect ){
                pSelect->select( SelectedId() );
                m_pSelectedObject = pSelect;
            }
        }
            break;
        case FL_DRAG:
            if(m_pSelectedObject){
                m_pSelectedObject->drag();
            }
            break;
        case FL_RELEASE:
            m_eGuiMode = eFPSNav;
            if(m_pSelectedObject){
                m_pSelectedObject->release();
                m_pSelectedObject->UnSelect(SelectedId());
            }
        }
        return true;
    }


};
*/

#endif // GUIWINDOW_H
