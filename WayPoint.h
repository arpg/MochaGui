//
//  WayPoint.h
//  MochaGui
//
//  Created by Gabe Sibley on 4/17/12.
//

#ifndef _MochaGui_WayPoint_h_
#define _MochaGui_WayPoint_h_


namespace Eigen {
    typedef Matrix<double, 6, 1 > Vector6d;
    typedef Matrix<double, 5, 1 > Vector5d;
}

// encapsulate a waypoint

struct WayPoint {

    WayPoint(const WayPoint & rhs) : m_dWayPoint(rhs.m_dWayPoint) {
        *this = rhs;
    }

    WayPoint& operator=(const WayPoint & w) {
        if (this != &w) { // make sure not same object
            m_bDirty = w.m_bDirty;
            m_dPose = w.m_dPose;
            m_dVelocity = w.m_dVelocity;
            m_dWayPoint = w.m_dWayPoint;
            m_nId = w.m_nId;
        }
        return *this; // Return ref for multiple assignment
    }//end operator=

    //    WayPoint()
    //    {
    //        m_bDirty = true;
    //        m_nId = 0;
    //        m_dWayPoint = Eigen::Vector5d::Zero();
    //    }

    WayPoint(int nId, Eigen::Vector5d & dWayPoint) : m_dWayPoint(dWayPoint) {
        m_nId = nId;
        //m_dWayPoint = dWayPoint;
        m_bDirty = true;
        m_dPose = Eigen::Vector6d::Zero();
        m_dVelocity = 1;
    }

    bool m_bDirty;
    unsigned int m_nId;
    Eigen::Vector5d& m_dWayPoint;
    Eigen::Vector5d m_dSolvedWayPoint;
    Eigen::Vector6d m_dPose;
    double m_dVelocity;
};



#endif
