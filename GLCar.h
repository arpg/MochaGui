#ifndef _GL_CAR_
#define	_GL_CAR_


#include <CarPlanner/RigidBody.h>


/*
float& fScale = CVarUtils::CreateCVar("car.Scale", 0.1f, "rendering scale option");
float& fRoll = CVarUtils::CreateCVar("car.Roll", 0.0f, "rendering scale option");
float& fPitch = CVarUtils::CreateCVar("car.Pitch", 0.0f, "rendering scale option");
float& fYaw = CVarUtils::CreateCVar("car.Yaw", 0.0f, "rendering scale option");
*/

#include "pangolin/pangolin.h"
#include "SceneGraph/SceneGraph.h"
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

using namespace SceneGraph;
using namespace pangolin;


enum GLCarDrawType {
    eTriangle = 1,
    eMesh = 2
};

class GLCar : public GLMesh {
public:

    GLCar() {
        m_sObjectName = "Car";
        m_bPerceptable = false;
    }

    void Init(GLCarDrawType eCarDrawType, std::string bodyMeshName = "/Users/crh/data/herbie.ply") {
        m_eCarDrawType = eCarDrawType;
        m_sBodyMeshName = bodyMeshName;
        m_Color = GLColor();

        //only if the body isn't a triangle, load the meshes
        if (m_eCarDrawType != eTriangle) {
            //initialize the body mesh
            const struct aiScene* pBodyMesh;
            pBodyMesh = aiImportFile("/Users/crh/data/herbie.ply", aiProcess_Triangulate | aiProcess_GenSmoothNormals
                                     | aiProcess_JoinIdenticalVertices);
            m_pScene = pBodyMesh;
            //LoadMeshTextures();

            GLMesh::Init(m_pScene);

            const struct aiScene* pWheelMesh;
            pWheelMesh = aiImportFile("/Users/crh/data/wheel.ply", aiProcess_Triangulate | aiProcess_GenSmoothNormals);


            //load the wheels
            m_vWheels.resize(4);
            for (size_t i = 0; i < m_vWheels.size(); i++) {
                m_vWheels[i] = new GLMesh();
                m_vWheels[i]->Init(pWheelMesh);
                //m_vWheels[i]->SetScale(m_fScale*0.75);
            }
        }
    }

    void DrawCanonicalObject() {
        if (m_eCarDrawType == eTriangle) {
            glColor4f(m_Color.r, m_Color.g, m_Color.b, m_Color.a);
            glBegin(GL_TRIANGLES);
//            glVertex3d(m_fScale, 0, 0);
//            glVertex3d(-m_fScale, -m_fScale / 2, 0);
//            glVertex3d(-m_fScale, m_fScale / 2, 0);
            glEnd();
        } else {
            //glScaled(m_dScale(0),m_dScale(1),m_dScale(2));
            GLMesh::DrawCanonicalObject();
        }

        glColor4f(1.0,1.0,1.0,1.0);
    }


    void SetRelativeWheelPose(const unsigned int& id, const Sophus::SE3d& pose) {
        if( m_eCarDrawType == eMesh ) {
            m_vWheels[id]->SetPose(pose.matrix());
        }
    }

    void SetColor(GLColor C) {
        m_Color = C;
    }

    void SetCarScale(Eigen::Vector3d scale)
    {
        GLMesh::SetScale(scale);
        for (size_t i = 0; i < m_vWheels.size(); i++) {
            m_vWheels[i]->SetScale(scale(0));
        }
    }

    std::vector<GLMesh *>& GetWheels() { return m_vWheels; }


protected:
    // centroid position + orientation
    Eigen::Matrix4d m_Pose;

    // control scale of car
    GLColor m_Color;
    std::string m_sBodyMeshName;
    std::vector<GLMesh *> m_vWheels;
    GLCarDrawType m_eCarDrawType;

};


#endif	/* _GL_CAR_ */
