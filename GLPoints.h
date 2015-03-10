#ifndef _GL_POINTS_
#define _GL_POINTS_

#include <SimpleGui/GLObject.h>


class GLPoints : public GLObject
{
    public:

        GLPoints()
        {
            m_cMarker = '.';
            m_Color   = GLColor();
			m_dPose = Eigen::Matrix4d::Identity();
            m_sObjectName = "Points";
        }

        void draw()
        {
			glPushMatrix();
			glMultMatrixd(m_dPose.data());

            switch( m_cMarker ) {
                case '.' :
//                	glEnable(GL_POINT_SMOOTH);
//                	glEnable(GL_BLEND);
                	glPointSize( 1.0 );
                    glColor4f( m_Color.r, m_Color.g, m_Color.b, m_Color.a );
                	glBegin(GL_POINTS);
					for( unsigned int ii = 0; ii < m_vPoints.size(); ii += 3 ) {
							glVertex3d( m_vPoints[ii], m_vPoints[ii+1], m_vPoints[ii+2] );
					}
                    glEnd();
                    break;
            }

			glPopMatrix();
        }

        void SetPoints( const std::vector<double>& vPoints ) {
			m_vPoints = vPoints;
        }

		void SetPose( const Eigen::Matrix4d& Pose )
		{
			m_dPose = Pose;
		}

        void ClearPoints() {
        	m_vPoints.clear();
        }

        // TODO add more markers (small circle, X and +), write SetColor, write SetMarker

    private:
        std::vector< double >	m_vPoints;
		Eigen::Matrix4d			m_dPose;
        char                    m_cMarker;
        GLColor                 m_Color;
};


#endif
