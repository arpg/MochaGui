/*
 * File:   MochaGui.h
 * Author: nima
 *
 * Created on April 24, 2012, 10:31 PM
 */

#ifndef _MOCHAGUI_H
#define	_MOCHAGUI_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <carplanner_msgs/Command.h>
#include <carplanner_msgs/VehicleState.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "/home/mike/code/MochaGui_ros/conversion_tools.h"

#include <sys/types.h>
#include <dirent.h>
#include <atomic>
#include <thread>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <glog/logging.h>

#include "CarMessages.pb.h"

#include <CarPlanner/CarController.h>
#include <CarPlanner/LocalPlanner.h>
#include <CarPlanner/CarRegressor.h>
#include <CarPlanner/Localizer.h>
#include <CarPlanner/CVarHelpers.h>

#include "GLCar.h"
#include "PlannerGui.h"
#include "GLBulletDebugDrawer.h"

#include "SensorFusionCeres.h"
#include "GLGuiPanel.h"
#include "assimp/DefaultLogger.hpp"
#include "EventLogger.h"
#include "ProcessModelFusion.h"

using namespace CVarUtils;
using namespace std;
using namespace Eigen;

extern float g_fTurnrate;
extern float g_fSpeed;

class MochaGui {
public:
    ////////////////////////////////////////////////////////////////

    void Run();
    void Init(const std::string& sRefPlane, const std::string& sMesh, bool bLocalizer,
              const std::string sMode, const std::string sLogFile, const std::string& sParamsFile,
              const std::string& sCarMesh, const std::string& sWheelMesh, bool enableROS=false);



    static MochaGui *GetInstance();
    ~MochaGui();



protected:
    MochaGui();
    bool _SetWaypointVel(std::vector<std::string> *vArgs);
    bool _Refresh(std::vector<std::string> *vArgs);
    void _RefreshWaypoints();
    bool _CommandFunc(MochaCommands command);
    bool _IteratePlanner(LocalProblem& problem,
                         MotionSample& sample,
                         Eigen::Vector3dAlignedVec& vActualTrajectory,
                         Eigen::Vector3dAlignedVec& vControlTrajectory,
            bool only2d = false);
    void _PhysicsFunc();
    void _ControlCommandFunc();
    void _ControlFunc();
    void _LocalizerReadFunc();
    void _PlannerFunc();
    void _LearningFunc(ControlPlan *m_pRegressionPlan);
    void _UpdateLearning(ControlCommand command,
                         VehicleState& state);
    void _UpdateWaypointFiles();
    void _StartThreads();
    void _KillThreads();
    void _KillController();
    void _CreateLogFilesIfNeeded();

    void _PopulateSceneGraph();
    void _UpdateVehicleStateFromFusion(VehicleState& currentState);

    void _UpdateVisuals();
    void _PlaybackLog(double dt);
    double m_dCurrentHue;
    bool _UpdateControlPathVisuals(const ControlPlan* pPlan);

    void _LoadDefaultParams();

    static bool SetWaypointVelHandler(std::vector<std::string> *vArgs) { return GetInstance()->_SetWaypointVel(vArgs); }
    static bool RefreshHandler(std::vector<std::string> *vArgs) { return GetInstance()->_Refresh(vArgs); }
    static bool CommandHandler(MochaCommands command) { return GetInstance()->_CommandFunc(command); }

    // UDP values
//    unsigned m_MochaPort; // UDP port for this gui
//    unsigned m_ComPort; // UDP port for m_bSIL (unused)
//    unsigned m_CarPort; // UDP port of the ninja car
//    struct sockaddr_in mochAddr; // Address of this gui (for sending commands from)
//    struct sockaddr_in comAddr;  // Address for m_bSIL (currently not running 1/16/17)
//    struct sockaddr_in carAddr;  // Address of the car (for sending commands to)
//    socklen_t addrLen = sizeof(mochAddr);
//    int recvLen;
//    int sockFD;
//    unsigned char buf[2048];
//    unsigned int msgSize = 0;

    // aiScene *scene;

    // ROS variable
    bool m_bEnableROS;
    ros::NodeHandle* m_nh;
    tf::TransformBroadcaster m_tfbr;

    void InitROS();

    boost::thread* m_pPublisherThread;
    void _PublisherFunc();

    ros::Publisher m_commandPub;
    void _pubCommand();
    void _pubCommand(ControlCommand& );

    // ros::Publisher m_statePub;
    // void _pubState();
    // void _pubState(VehicleState& );

    ros::Publisher m_meshPub;
    void _pubMesh();
    void _pubMesh(aiMesh*& );

    ros::Subscriber m_meshSub;
    void _meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr& );

    void _convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMeshStamped*&, aiMesh*& );
    // void _convertMeshMsgToGLMesh(mesh_msgs::TriangleMeshStamped*&, GLMesh*& );
    void _convertAssimpMeshToMeshMsg(aiMesh*&, mesh_msgs::TriangleMeshStamped*& );

    // GUIHelperInterface* m_guiHelper;

    //car variables
    GLBulletDebugDrawer m_BulletDebugDrawer;
    GLMesh m_TerrainMesh;

    int m_nStateStatusId;
    int m_nLearningStatusId;
    int m_nControlStatusId;
    int m_nGravityStatusId;

    int m_nDriveCarId;

    double m_dControlPlansPerS;

    double m_dDiagTime;

    std::vector<Eigen::MatrixXd*> m_vWayPoints;
    //vector<GLWayPoint*> m_vGLWayPoints;
    const GLWayPoint*m_pCurrentlySolving[2];
    vector<GLCachedPrimitives> m_vGLLineSegments;
    vector<GLCachedPrimitives> m_vTerrainLineSegments;
    list<GLCachedPrimitives*> m_lPlanLineSegments;
    list<std::vector<VehicleState> *> m_lPlanStates;
    GLCachedPrimitives* m_pControlLine;
    std::vector<MotionSample> m_vSegmentSamples;

    LocalPlanner m_Planner; // Car planner for trajectory plotting
    CarController m_Controller;
    BulletCarModel m_PlanCarModel;
    BulletCarModel m_LearningCarModel;
    BulletCarModel m_DriveCarModel;
    BulletCarModel m_ControlCarModel;

    //CVars
    vector<int>& m_Path;
#define WAYPOINT_VEL_INDEX 6
#define WAYPOINT_AIR_INDEX 7

    //flags
    std::atomic<bool> m_StillRun;
    std::atomic<bool> m_StillControl;
    bool& m_bPlannerOn;
    bool& m_bShowProjectedPath;
    bool& m_bCompute3dPath;
    bool& m_bSimulate3dPath;
    int& m_eControlTarget;
    bool& m_bControl3dPath;
    bool m_bSIL = false;

    bool m_bPlanning;
    bool m_bControllerRunning;
    bool m_bLearningRunning;
    unsigned int& m_nPathSegments;
    double& m_dTimeInterval;
    double m_dFps;
    bool m_bPause;
    bool m_bStep;
    bool m_bLastPause;
    bool m_bAllClean;

    Vector3dAlignedVec m_vFixGroundSamples;
    //Eigen::Matrix3d m_RfixGround;

    Localizer m_Localizer;
    std::string m_sCarObjectName;
    //std::vector<MochaEntity> m_vEntities;

    boost::thread* m_pPlannerThread;
    boost::thread* m_pPhysicsThread;
    boost::thread* m_pControlThread;
    boost::thread* m_pCommandThread;
    boost::thread* m_pLearningThread;
    boost::thread* m_pLocalizerThread;

    ControlCommand m_ControlCommand;
    double m_dTargetVel;
    double m_dPoseError;
    double m_dVelError;
    double m_dControlAccel;
    double m_dControlPhi;

    //logging
    EventLogger m_Logger;
    EventLogger m_FusionLogger;
    double m_dStartTime;
    bool& m_bLoggerEnabled;
    std::string& m_sLogFileName;
    bool& m_bFusionLoggerEnabled;
    bool& m_bLoggerPlayback;
    unsigned int m_nNumControlSignals;
    unsigned int m_nNumPoseUpdates;
    std::string m_sPlaybackLogFile;
    double m_dPlaybackTimer;

    ProcessModelFusion m_Fusion;

    boost::mutex m_ControlMutex;
    boost::mutex m_DrawMutex;

    PlannerGui m_Gui;

    //ui widgets
    GLGuiPanel m_GuiPanel;
    double m_dLocalizerFreq;
    double m_dVel;
    Eigen::Vector3d m_dPos;

    //load/save of files
    bool m_bLoadWaypoints;
    bool m_bSaveWaypoints;
    char m_pSaveFileName[100];
    int m_nSaveFileNameLen;
    int m_nSelectedFileName;
    std::string m_sParamsFile;
    std::vector<std::string> m_vFileNames;

    std::vector<RegressionParameter> m_vLearningParams;
    std::vector<RegressionParameter> m_vControlParams;
    std::vector<RegressionParameter> m_vPlannerParams;
    std::vector<RegressionParameter> m_vDriveParams;
    CarParameterMap m_mDefaultParameters;

    pangolin::DataLog m_Log;
    pangolin::View* m_pGraphView;
    Eigen::Vector3d m_vTargetVel;
    Sophus::SE3d m_dTargetPose;
    double m_dControlDelay;

    std::atomic<double> m_dPlanTime;

    GLAxis m_DestAxis;
};

namespace mochagui{
  //
  //
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // inline void CollisionShape2TriangleMesh(btCollisionShape* collisionShape, const btTransform& parentTransform, btAlignedObjectArray<btVector3>& vertexPositions, btAlignedObjectArray<btVector3>& vertexNormals, btAlignedObjectArray<int>& indicesOut)
  // {
  // // CollisionShape2TriangleMesh(mesh, parentTransform, vertexPositions, vertexNormals, indicesOut);
  //   switch (collisionShape->getShapeType())
  //   {
  //     case SOFTBODY_SHAPE_PROXYTYPE:
  //     {
  //       //skip the soft body collision shape for now
  //       break;
  //     }
  //     case STATIC_PLANE_PROXYTYPE:
  //     {
  //       //draw a box, oriented along the plane normal
  //       const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(collisionShape);
  //       btScalar planeConst = staticPlaneShape->getPlaneConstant();
  //       const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
  //       btVector3 planeOrigin = planeNormal * planeConst;
  //       btVector3 vec0, vec1;
  //       btPlaneSpace1(planeNormal, vec0, vec1);
  //       btScalar vecLen = 100.f;
  //       btVector3 verts[4];
  //
  //       verts[0] = planeOrigin + vec0 * vecLen + vec1 * vecLen;
  //       verts[1] = planeOrigin - vec0 * vecLen + vec1 * vecLen;
  //       verts[2] = planeOrigin - vec0 * vecLen - vec1 * vecLen;
  //       verts[3] = planeOrigin + vec0 * vecLen - vec1 * vecLen;
  //
  //       int startIndex = vertexPositions.size();
  //       indicesOut.push_back(startIndex + 0);
  //       indicesOut.push_back(startIndex + 1);
  //       indicesOut.push_back(startIndex + 2);
  //       indicesOut.push_back(startIndex + 0);
  //       indicesOut.push_back(startIndex + 2);
  //       indicesOut.push_back(startIndex + 3);
  //
  //       btVector3 triNormal = parentTransform.getBasis() * planeNormal;
  //
  //       for (int i = 0; i < 4; i++)
  //       {
  //         btVector3 vtxPos;
  //         btVector3 pos = parentTransform * verts[i];
  //         vertexPositions.push_back(pos);
  //         vertexNormals.push_back(triNormal);
  //       }
  //       break;
  //     }
  //     case TRIANGLE_MESH_SHAPE_PROXYTYPE:
  //     {
  //       btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)collisionShape;
  //       btVector3 trimeshScaling = trimesh->getLocalScaling();
  //       btStridingMeshInterface* meshInterface = trimesh->getMeshInterface();
  //       btAlignedObjectArray<btVector3> vertices;
  //       btAlignedObjectArray<int> indices;
  //
  //       for (int partId = 0; partId < meshInterface->getNumSubParts(); partId++)
  //       {
  //         const unsigned char* vertexbase = 0;
  //         int numverts = 0;
  //         PHY_ScalarType type = PHY_INTEGER;
  //         int stride = 0;
  //         const unsigned char* indexbase = 0;
  //         int indexstride = 0;
  //         int numfaces = 0;
  //         PHY_ScalarType indicestype = PHY_INTEGER;
  //         //PHY_ScalarType indexType=0;
  //
  //         btVector3 triangleVerts[3];
  //         meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, partId);
  //         btVector3 aabbMin, aabbMax;
  //
  //         // printf("%s %d faces\n", "[CollisionShape2TriangleMesh]", numfaces);
  //
  //         for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
  //         {
  //           unsigned int* gfxbase = (unsigned int*)(indexbase + triangleIndex * indexstride);
  //
  //           for (int j = 2; j >= 0; j--)
  //           {
  //             int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : gfxbase[j];
  //             if (type == PHY_FLOAT)
  //             {
  //               float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);
  //               triangleVerts[j] = btVector3(
  //                 graphicsbase[0] * trimeshScaling.getX(),
  //                 graphicsbase[1] * trimeshScaling.getY(),
  //                 graphicsbase[2] * trimeshScaling.getZ());
  //             }
  //             else
  //             {
  //               double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);
  //               triangleVerts[j] = btVector3(btScalar(graphicsbase[0] * trimeshScaling.getX()),
  //                              btScalar(graphicsbase[1] * trimeshScaling.getY()),
  //                              btScalar(graphicsbase[2] * trimeshScaling.getZ()));
  //             }
  //           }
  //           indices.push_back(vertices.size());
  //           vertices.push_back(triangleVerts[0]);
  //           indices.push_back(vertices.size());
  //           vertices.push_back(triangleVerts[1]);
  //           indices.push_back(vertices.size());
  //           vertices.push_back(triangleVerts[2]);
  //
  //           btVector3 triNormal = (triangleVerts[1] - triangleVerts[0]).cross(triangleVerts[2] - triangleVerts[0]);
  //           btScalar dot = triNormal.dot(triNormal);
  //
  //           //cull degenerate triangles
  //           if (dot >= SIMD_EPSILON * SIMD_EPSILON)
  //           {
  //             triNormal /= btSqrt(dot);
  //             for (int v = 0; v < 3; v++)
  //             {
  //               btVector3 pos = parentTransform * triangleVerts[v];
  //               indicesOut.push_back(vertexPositions.size());
  //               vertexPositions.push_back(pos);
  //               vertexNormals.push_back(triNormal);
  //             }
  //           }
  //         }
  //       }
  //
  //       break;
  //     }
  //     default:
  //     {
  //       if (collisionShape->isConvex())
  //       {
  //         btConvexShape* convex = (btConvexShape*)collisionShape;
  //         {
  //           const btConvexPolyhedron* pol = 0;
  //           if (convex->isPolyhedral())
  //           {
  //             btPolyhedralConvexShape* poly = (btPolyhedralConvexShape*)convex;
  //             pol = poly->getConvexPolyhedron();
  //           }
  //
  //           if (pol)
  //           {
  //             for (int v = 0; v < pol->m_vertices.size(); v++)
  //             {
  //               vertexPositions.push_back(pol->m_vertices[v]);
  //               btVector3 norm = pol->m_vertices[v];
  //               norm.safeNormalize();
  //               vertexNormals.push_back(norm);
  //             }
  //             for (int f = 0; f < pol->m_faces.size(); f++)
  //             {
  //               for (int ii = 2; ii < pol->m_faces[f].m_indices.size(); ii++)
  //               {
  //                 indicesOut.push_back(pol->m_faces[f].m_indices[0]);
  //                 indicesOut.push_back(pol->m_faces[f].m_indices[ii - 1]);
  //                 indicesOut.push_back(pol->m_faces[f].m_indices[ii]);
  //               }
  //             }
  //           }
  //           else
  //           {
  //             btShapeHull* hull = new btShapeHull(convex);
  //             hull->buildHull(0.0, 1);
  //
  //             {
  //               //int strideInBytes = 9*sizeof(float);
  //               //int numVertices = hull->numVertices();
  //               //int numIndices =hull->numIndices();
  //
  //               for (int t = 0; t < hull->numTriangles(); t++)
  //               {
  //                 btVector3 triNormal;
  //
  //                 int index0 = hull->getIndexPointer()[t * 3 + 0];
  //                 int index1 = hull->getIndexPointer()[t * 3 + 1];
  //                 int index2 = hull->getIndexPointer()[t * 3 + 2];
  //                 btVector3 pos0 = parentTransform * hull->getVertexPointer()[index0];
  //                 btVector3 pos1 = parentTransform * hull->getVertexPointer()[index1];
  //                 btVector3 pos2 = parentTransform * hull->getVertexPointer()[index2];
  //                 triNormal = (pos1 - pos0).cross(pos2 - pos0);
  //                 triNormal.safeNormalize();
  //
  //                 for (int v = 0; v < 3; v++)
  //                 {
  //                   int index = hull->getIndexPointer()[t * 3 + v];
  //                   btVector3 pos = parentTransform * hull->getVertexPointer()[index];
  //                   indicesOut.push_back(vertexPositions.size());
  //                   vertexPositions.push_back(pos);
  //                   vertexNormals.push_back(triNormal);
  //                 }
  //               }
  //             }
  //             delete hull;
  //           }
  //         }
  //       }
  //       else
  //       {
  //         if (collisionShape->isCompound())
  //         {
  //           btCompoundShape* compound = (btCompoundShape*)collisionShape;
  //           for (int i = 0; i < compound->getNumChildShapes(); i++)
  //           {
  //             btTransform childWorldTrans = parentTransform * compound->getChildTransform(i);
  //             CollisionShape2TriangleMesh(compound->getChildShape(i), childWorldTrans, vertexPositions, vertexNormals, indicesOut);
  //           }
  //         }
  //         else
  //         {
  //           if (collisionShape->getShapeType() == SDF_SHAPE_PROXYTYPE)
  //           {
  //             //not yet
  //           }
  //           else
  //           {
  //             btAssert(0);
  //           }
  //         }
  //       }
  //     }
  //   };
  // }
  //
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, const btTransform* parentTransform, mesh_msgs::TriangleMesh** meshMsg)
  // {
  //   // const btTransform parentTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  //   btAlignedObjectArray<btVector3> vertexPositions;
  //   btAlignedObjectArray<btVector3> vertexNormals;
  //   btAlignedObjectArray<int> indicesOut;
  //
  //   CollisionShape2TriangleMesh(collisionShape, *parentTransform, vertexPositions, vertexNormals, indicesOut);
  //
  //   // printf("%s %d indices, %d vertices\n", "[convertCollisionShape2MeshMsg]", indicesOut.size(), vertexPositions.size());
  //
  //   uint numVerts = indicesOut.size();
  //   (*meshMsg)->vertices = std::vector<geometry_msgs::Point>(numVerts);
  //   for( uint iv=0; iv<numVerts; iv++)
  //   {
  //     btVector3 vertexPosition = vertexPositions[iv];
  //     geometry_msgs::Point sVertex;
  //     sVertex.x = vertexPosition[0];
  //     sVertex.y = vertexPosition[1];
  //     sVertex.z = -vertexPosition[2];
  //     (*meshMsg)->vertices[iv] = sVertex;
  //   }
  //
  //   uint numTris = indicesOut.size()/3;
  //   (*meshMsg)->triangles = std::vector<mesh_msgs::TriangleIndices>(numTris);
  //   for( uint it=0; it<numTris; it++)
  //   {
  //     for( uint ii=0; ii<3; ii++)
  //     {
  //       (*meshMsg)->triangles[it].vertex_indices[ii] = indicesOut[it*3+ii];
  //     }
  //   }
  //
  //   return;
  // }
  //
  // inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, mesh_msgs::TriangleMesh** meshMsg)
  // {
  //   const btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  //   convertCollisionShape2MeshMsg(collisionShape, parentTransform, meshMsg);
  //   return;
  // }
  //
  // inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, mesh_msgs::TriangleMeshStamped** mesh_stamped)
  // {
  //   mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
  //   convertCollisionShape2MeshMsg(collisionShape, &mesh);
  //   (*mesh_stamped)->mesh = *mesh;
  //   return;
  // }
  //
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // inline void convertMeshMsg2CollisionShape(mesh_msgs::TriangleMesh* meshMsg, btCollisionShape** collisionShape)
  // {
  //   btTriangleMesh* triangleMesh = new btTriangleMesh();
  //   for( uint it=0; it<meshMsg->triangles.size(); it++)
  //   {
  //     geometry_msgs::Point vertex0 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]];
  //     geometry_msgs::Point vertex1 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]];
  //     geometry_msgs::Point vertex2 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]];
  //
  //     triangleMesh->addTriangle(btVector3(vertex0.x, vertex0.y, vertex0.z),
  //                               btVector3(vertex1.x, vertex1.y, vertex1.z),
  //                               btVector3(vertex2.x, vertex2.y, vertex2.z),
  //                               it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);
  //   }
  //
  //   *collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);
  //
  //   return;
  // }
  //
  // inline void convertMeshMsg2CollisionShape(mesh_msgs::TriangleMeshStamped* meshMsg, btCollisionShape** collisionShape)
  // {
  //   mesh_msgs::TriangleMesh* mesh = &(meshMsg->mesh);
  //   convertMeshMsg2CollisionShape(mesh, collisionShape);
  //   return;
  // }
  //
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // inline void convertAssimpMeshToMeshMsg(aiMesh*& sMesh, mesh_msgs::TriangleMesh*& mesh)
  // {
  //   mesh_msgs::TriangleMesh* dMesh = new mesh_msgs::TriangleMesh();
  //
  //   // copy vertices
  //   dMesh->vertices = std::vector<geometry_msgs::Point>(sMesh->mNumVertices);
  //   for( uint iv=0; iv<sMesh->mNumVertices; iv++)
  //   {
  //     aiVector3D aVertex = sMesh->mVertices[iv];
  //     geometry_msgs::Point rVertex;
  //     rVertex.x = aVertex[0];
  //     rVertex.y = aVertex[1];
  //     rVertex.z = aVertex[2];
  //     dMesh->vertices[iv] = rVertex;
  //   }
  //
  //   // copy faces
  //   dMesh->triangles = std::vector<mesh_msgs::TriangleIndices>(sMesh->mNumFaces);
  //   for( uint it=0; it<sMesh->mNumFaces; it++)
  //   {
  //     aiFace* aFace = &(sMesh->mFaces[it]);
  //     mesh_msgs::TriangleIndices* rFace = &(dMesh->triangles[it]);
  //     for( uint ii=0; ii<aFace->mNumIndices; ii++)
  //     {
  //       rFace->vertex_indices[ii] = aFace->mIndices[ii];
  //     }
  //   }
  //
  //   mesh = dMesh;
  // }
  //
  // inline void convertAssimpMeshToMeshMsg(aiMesh*& sMesh, mesh_msgs::TriangleMeshStamped*& mesh)
  // {
  //   mesh_msgs::TriangleMesh* tmpmesh;
  //   convertAssimpMeshToMeshMsg(sMesh, tmpmesh);
  //   mesh->mesh = *tmpmesh;
  // }
  //
  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //
  // inline void convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMesh*& sMesh, aiMesh*& mesh)
  // {
  //   aiMesh* dMesh = new aiMesh();
  //
  //   // dMesh->mVertices = std::vector<aiVector3D>(sMesh->vertices.size());
  //   std::vector<aiVector3D> vVertices(sMesh->vertices.size());
  //   dMesh->mVertices = &(vVertices[0]);
  //   for( uint iv=0; iv<sMesh->vertices.size(); iv++)
  //   {
  //     geometry_msgs::Point sVertex = sMesh->vertices[iv];
  //     dMesh->mVertices[iv] = aiVector3D(sVertex.x, sVertex.y, sVertex.z);
  //   }
  //
  //   std::vector<aiFace> vFaces(sMesh->triangles.size());
  //   dMesh->mFaces = &(vFaces[0]);
  //   for( uint it=0; it<sMesh->triangles.size(); it++)
  //   {
  //     mesh_msgs::TriangleIndices* sFace = &(sMesh->triangles[it]);
  //     aiFace* dFace = &(dMesh->mFaces[it]);
  //     for( uint ii=0; ii<sFace->vertex_indices.size(); ii++)
  //     {
  //       dFace->mIndices[ii] = sFace->vertex_indices[ii];
  //     }
  //   }
  //
  //   mesh = dMesh;
  // }
  //
  // inline void convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMeshStamped*& sMesh, aiMesh*& mesh)
  // {
  //   mesh_msgs::TriangleMesh* tmpmesh = &(sMesh->mesh);
  //   convertMeshMsgToAssimpMesh(tmpmesh, mesh);
  // }

}

#endif	/* MOCHAGUI_H */
