#ifndef _GL_WAYPOINT_
#define	_GL_WAYPOINT_

#include <SimpleGui/Gui.h>
#include <SimpleGui/GLObject.h>
#include <SimpleGui/GLMesh.h>
#include "SE3.h"
#include "WayPoint.h"


#define VELOCITY_MULTIPLIER 2

class GLWayPoint : public GLObject 
{
public:
    
    GLWayPoint(WayPoint* pWp) 
    {
        m_pWayPoint = pWp;
        m_sObjectName = "Waypoint";
        m_bInitDone = false;
        m_bPerceptable = false;
        m_wpScale = 0.25;
    }    

    void select(unsigned int nId) 
    {
        assert(m_pWayPoint);
        m_pWayPoint->m_bDirty = true; // flag for update
        Eigen::Vector3d v = Window()->GetPosUnderCursor();
        m_nSelectedId = nId;
        if (nId == m_nBaseId) {
            m_pWayPoint->m_dPose[0] = v[0];
            m_pWayPoint->m_dPose[1] = v[1];
            m_pWayPoint->m_dPose[2] = v[2];
        }
    }

    ///
    void drag()
    {
        m_pWayPoint->m_bDirty = true; // flag for update
        Eigen::Matrix4d T = mvl::Cart2T(m_pWayPoint->m_dPose);
        char tBuff[100] = {};
        if (m_nSelectedId == m_nBaseId) {
            Eigen::Vector3d v = Window()->GetPosUnderCursor();
            m_pWayPoint->m_dPose.block<3,1>( 0, 0 ) = v;
            sprintf(tBuff, "(x: %.2f, y: %.2f, z: %.2f)", v[0], v[1], v[2]);
            Eigen::Vector3d d = -Window()->GetNormalUnderCursor();
            //d = d/d.norm();
            d << 0, 0, 1;
            Eigen::Vector3d f = T.block < 3, 1 > (0, 0);
            Eigen::Vector3d r = d.cross(f);
            //                r << 0,1,0;
            //                f = r.cross(d);
            Eigen::Matrix3d R;
            R.block < 3, 1 > (0, 0) = f;
            R.block < 3, 1 > (0, 1) = r;
            R.block < 3, 1 > (0, 2) = d;
            m_pWayPoint->m_dPose.block < 3, 1 > (3, 0) = mvl::R2Cart(R);
            m_pWayPoint->m_dWayPoint[0] = m_pWayPoint->m_dPose[0];
            m_pWayPoint->m_dWayPoint[1] = m_pWayPoint->m_dPose[1];
        }
        if (m_nSelectedId == m_nFrontId) {
            Eigen::Vector3d v = GetPosUnderCursor();
            double dx = v[0] - m_pWayPoint->m_dPose[0];
            double dy = v[1] - m_pWayPoint->m_dPose[1];
            double th = atan2(dy, dx);
            m_pWayPoint->m_dPose[5] = th;
            m_pWayPoint->m_dVelocity = sqrt(dx * dx + dy * dy)*VELOCITY_MULTIPLIER;
            sprintf(tBuff, "(a: %.2f, v: %.2f)", th, m_pWayPoint->m_dVelocity);
            
            m_pWayPoint->m_dWayPoint[2] = m_pWayPoint->m_dPose[5];
            m_pWayPoint->m_dWayPoint[3] = 0; //set curvature to 0
            m_pWayPoint->m_dWayPoint[4] = m_pWayPoint->m_dVelocity;
        }
        m_sLabel = tBuff;
        m_nLabelPos = GetCursorPos();
    }
    
    void release() {
        m_sLabel.clear();
    }
    
    void Init() {
        assert(m_pWayPoint);
        char buf[128];
        snprintf(buf, 128, "WayPoint-%d", m_pWayPoint->m_nId);
        SetName(buf);
        m_nFrontId = AllocSelectionId();
        m_nBaseId = AllocSelectionId();
        m_bInitDone = true;
    }
    
    void draw() {
        if (!m_bInitDone) {
            Init();
        }

        glPushAttrib(GL_ENABLE_BIT);

        glDisable(GL_CULL_FACE);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        
        glColor4ub(90, 90, 255, 200);
        glPushMatrix();
        
        Eigen::Matrix<double, 4, 4, Eigen::ColMajor> T = mvl::Cart2T(m_pWayPoint->m_dPose);
        T.block < 3, 1 > (0, 3) -= 0.05 * T.block < 3, 1 > (0, 2); // move away from the surface a bit
        glMultMatrixd(T.data());
        
        /*glBegin(GL_TRIANGLES);
        double s = 0.5;
        glVertex3d(s, 0, 0);
        glVertex3d(0, -s / 4, 0);
        glVertex3d(0, s / 4, 0);
        glEnd();*/
        
        glColor4ub(255, 255, 255, 78);
        glBegin(GL_LINES);
        glVertex3d(m_pWayPoint->m_dVelocity/VELOCITY_MULTIPLIER, 0, 0);
        glVertex3d(0, 0, 0);
        glEnd();

        // draw axis
        glBegin(GL_LINES);
        glColor4f(1, 0, 0, 1);
        glVertex3d(0, 0, 0);
        glVertex3d(m_wpScale, 0, 0);
        
        glColor4f(0, 1, 0, 1);
        glVertex3d(0, 0, 0);
        glVertex3d(0, m_wpScale, 0);
        
        glColor4f(0, 0, 1, 1);
        glVertex3d(0, 0, 0);
        glVertex3d(0, 0, m_wpScale);
        
        glEnd();
        
        // draw front point
        {
            glColor3ub(0, 255, 0);
            if (IsSelected(m_nFrontId)) {
                glColor3ub(255, 0, 0);
                UnSelect(m_nFrontId);
            }
            
            glPushName(m_nFrontId); // our GLObject ID
            glPointSize(5);
            glBegin(GL_POINTS);
            glVertex3d(m_pWayPoint->m_dVelocity/VELOCITY_MULTIPLIER, 0, 0);
            glEnd();
            glPopName();
        }
        
        // draw back point
        {
            glColor3ub(0, 255, 0);
            if (IsSelected(m_nBaseId)) {
                glColor3ub(255, 0, 0);
                UnSelect(m_nBaseId);
            }
            
            glPushName(m_nBaseId); // our GLObject ID
            glPointSize(5);
            glBegin(GL_POINTS);
            glVertex3d(0, 0, 0);
            glEnd();
            glPopName();
        }
        
        // draw label
        {
            PushOrtho(WindowWidth(), WindowHeight());
            glColor4f(1.0, 1.0, 1.0, 1.0);
            gl_font(0, 8);
            glRasterPos2f(m_nLabelPos(0), m_nLabelPos(1));
            gl_draw(m_sLabel.c_str(), m_sLabel.length());
            PopOrtho();
        }

  //      m_CarMesh.draw();

        glPopMatrix();
        glPopAttrib();

    }
    
    /*
     void SetWayPoint(
     const unsigned int nId,
     Eigen::Vector6d dPose,
     double dVelocity
     )
     {
     assert( m_pWayPoint );
     m_pWayPoint->m_nId = nId;
     m_pWayPoint->m_dVelocity = dVelocity;
     m_pWayPoint->m_dPose = dPose;
     }
     */
    
private:
//    GLMesh*         m_pCarMesh;
    float           m_wpScale;
    WayPoint*       m_pWayPoint;
    unsigned int    m_nFrontId;
    unsigned int    m_nBaseId;
    unsigned int    m_nSelectedId;
    std::string     m_sLabel;
    Eigen::Vector2i m_nLabelPos;
    bool            m_bInitDone;
    double          m_dScale;
};


#endif
