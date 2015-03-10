#ifndef _GL_MESHRAMP_H_
#define _GL_MESHRAMP_H_

#include <SimpleGui/GLObject.h>

#include "assimp/assimp.h"
#include "assimp/aiPostProcess.h"
#include "assimp/aiScene.h"


class GLMeshRamp : public GLObject
{

    public:

        GLMeshRamp()
        {
            m_sObjectName = "MeshRamp"; 
        }

        ////////////////////////////////////////////////////////////////////////////
        void Init( const struct aiScene* pScene )
        {
			m_Pose = Eigen::Matrix4d::Identity();
			m_pScene = pScene;

			SetName( "Ramp" );
		}

        ////////////////////////////////////////////////////////////////////////////
		void recursive_register( const struct aiNode* pNode ) {
				// iterate thought meshes
            for ( unsigned int ii = 0; ii < pNode->mNumMeshes; ii++ ) {
                const struct aiMesh* pMesh = m_pScene->mMeshes[pNode->mMeshes[ii]];


                for ( unsigned int jj = 0; jj < pMesh->mNumFaces; jj++ ) {
					m_vFaceId.push_back( AllocSelectionId() );
				}
			}

			// iterate through children
            for( int ii = 0; ii < pNode->mNumChildren; ii++ ) {
                recursive_register( pNode->mChildren[ii] );
            }
		}

        ////////////////////////////////////////////////////////////////////////////
        void color4_to_float4( const struct aiColor4D *c, float f[4] )
        {
            f[0] = c->r;
            f[1] = c->g;
            f[2] = c->b;
            f[3] = c->a;
        }

        // ----------------------------------------------------------------------------
        void set_float4(float f[4], float a, float b, float c, float d)
        {
            f[0] = a;
            f[1] = b;
            f[2] = c;
            f[3] = d;
        }


        ////////////////////////////////////////////////////////////////////////////
        void apply_material(const struct aiMaterial *mtl)
        {
            float c[4];

            GLenum fill_mode;
            int ret1, ret2;
            struct aiColor4D diffuse;
            struct aiColor4D specular;
            struct aiColor4D ambient;
            struct aiColor4D emission;
            float shininess, strength;
            int two_sided;
            int wireframe;
            unsigned int max;

            set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
            if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse)){
                color4_to_float4(&diffuse, c);
                }
            glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, c );

            set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
            if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular)){
                color4_to_float4(&specular, c);
            }
            glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, c );

            set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
            if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient)){
                color4_to_float4(&ambient, c);
            }
            glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, c );

            set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
            if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission)){
                color4_to_float4(&emission, c);
            }
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);


            max = 1;
            ret1 = aiGetMaterialFloatArray( mtl, AI_MATKEY_SHININESS, &shininess, &max );
            if(ret1 == AI_SUCCESS) {
                max = 1;
                ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
                if(ret2 == AI_SUCCESS){
                    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
                }
                else{
                    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
                }
            }
            else {
                glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
                set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
            }

            return;
            max = 1;
            if( AI_SUCCESS == aiGetMaterialIntegerArray(
                        mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max) ){
                fill_mode = wireframe ? GL_LINE : GL_FILL;
            }
            else{
                fill_mode = GL_FILL;
            }
            glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

            max = 1;
            if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided){
                glDisable(GL_CULL_FACE);
            }
            else{
                glEnable(GL_CULL_FACE);
            }
        }

        ////////////////////////////////////////////////////////////////////////////
        void RenderMesh( GLenum face_mode, GLenum fill_mode, float fAlpha, const struct aiFace* face,  const struct aiMesh* mesh )
        {
            glPolygonMode(GL_FRONT_AND_BACK, fill_mode );
			glPushMatrix();
			glMultMatrixd(m_Pose.data());

            glEnable(GL_DEPTH_TEST);
            glBegin(face_mode);
            for( int i = 0; i < face->mNumIndices; i++) {
                int index = face->mIndices[i];
                if(mesh->mColors[0] != NULL) {
                    float *c = (float*)&mesh->mColors[0][index];
                    glColor4f( c[0], c[1], c[2], fAlpha*c[3] );
                } else {
                    glColor4f( 0.5, 0.5, 0.5, 0.6 );
                }
                if(mesh->mNormals != NULL){
                    glNormal3fv(&mesh->mNormals[index].x);
                }
                glVertex3fv(&mesh->mVertices[index].x);
            }
            glEnd();
			glPopMatrix();
        }

        ////////////////////////////////////////////////////////////////////////////
        void recursive_render( const struct aiScene *sc, const struct aiNode* nd )
        {
//            int i;
            unsigned int n = 0, t;
            struct aiMatrix4x4 m = nd->mTransformation;

            // update transform
            aiTransposeMatrix4( &m );
            glPushMatrix();
            glMultMatrixf( (float*)&m );

            // draw all meshes assigned to this node
            for (; n < nd->mNumMeshes; ++n) {
                const struct aiMesh* mesh = m_pScene->mMeshes[nd->mMeshes[n]];

                apply_material( sc->mMaterials[mesh->mMaterialIndex] );

                if(mesh->mNormals == NULL) {
                    glDisable(GL_LIGHTING);
                } else {
                    glEnable(GL_LIGHTING);
                }
                glEnable (GL_BLEND);
                glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

                for (t = 0; t < mesh->mNumFaces; ++t) {
                    const struct aiFace* face = &mesh->mFaces[t];
                    GLenum face_mode;

                    switch(face->mNumIndices) {
                        case 1: face_mode = GL_POINTS; break;
                        case 2: face_mode = GL_LINES; break;
                        case 3: face_mode = GL_TRIANGLES; break;
                        default: face_mode = GL_POLYGON; break;
                    }

                    RenderMesh( face_mode, GL_FILL, 0.5, face, mesh );

                    bool bShowMesh = 1;
                    if( bShowMesh ) {
                        glDisable( GL_LIGHTING );
                        RenderMesh( face_mode, GL_LINE, 1.0, face, mesh );
                    }
                }
            }

            // draw all children
            for( n = 0; n < nd->mNumChildren; ++n ) {
                recursive_render( sc, nd->mChildren[n] );
            }
            glPopMatrix();
        }

        ////////////////////////////////////////////////////////////////////////////
        void  draw()
        {
            if( m_pScene ){
                recursive_render( m_pScene, m_pScene->mRootNode );
            }
        }

        ////////////////////////////////////////////////////////////////////////////
		void SetPose( const Eigen::Matrix<double,6,1>& Pose )
		{

			m_Pose = _Cart2T(Pose);
		}

	private:
		Eigen::Matrix4d _Cart2T( const Eigen::Matrix<double,6,1>& Pose )
		{
			Eigen::Matrix4d T;
			// psi = roll, th = pitch, phi = yaw
			double cq, cp, cr, sq, sp, sr;
			cr = cos( Pose(3,0) );
			cp = cos( Pose(4,0) );
			cq = cos( Pose(5,0) );

			sr = sin( Pose(3,0) );
			sp = sin( Pose(4,0) );
			sq = sin( Pose(5,0) );

			T(0,0) = cp*cq;
			T(0,1) = -cr*sq+sr*sp*cq;
			T(0,2) = sr*sq+cr*sp*cq;

			T(1,0) = cp*sq;
			T(1,1) = cr*cq+sr*sp*sq;
			T(1,2) = -sr*cq+cr*sp*sq;

			T(2,0) = -sp;
			T(2,1) = sr*cp;
			T(2,2) = cr*cp;

			T(0,3) = Pose(0,0);
			T(1,3) = Pose(1,0);
			T(2,3) = Pose(2,0);
			T.row(3) = Eigen::Vector4d( 0.0, 0.0, 0.0, 1.0 );
			return T;
		}

    private:
        const struct aiScene*			m_pScene;
		std::vector< unsigned int >		m_vFaceId;
		Eigen::Matrix4d					m_Pose;
};


#endif

