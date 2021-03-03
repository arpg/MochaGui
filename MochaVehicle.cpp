#include "CarPlannerCommon.h"
#include "MochaVehicle.h"
// #include "cvars/CVar.h"

// static bool& g_bSkidCompensationActive(CVarUtils::CreateCVar("debug.SkidCompensationActive", false, ""));

// namespace MochaVehicle{
//     struct ApplyVelocitiesClientFunctor 
// {
//     ApplyVelocitiesClientFunctor(ros::NodeHandle* nh_, const VehicleState& startState_, MotionSample& sample_,
//         int nWorldId_= 0, bool noCompensation_= false) 
//         :   ros::NodeHandle* nh(nh_),
//             const VehicleState& startState(startState_),
//             MotionSample& sample(sample_),
//             int nWorldId(nWorldId_),
//             bool noCompensation(noCompensation_),
//             actionName("plan_car/apply_velocities"+std::to_string(nWorldId_)),
//             timeout(15.0),

//     {
//         client(actionName,true);
//     }

//     void operator()()
//     {
//         SetThreadName((boost::format("Bullet ApplyVelocities Thread #%d") % m_index).str().c_str());

//         double t0 = Tic();

//         client.waitForServer();

//         double t1 = Tic();

//         carplanner_msgs::ApplyVelocitiesGoal goal;
//         carplanner_msgs::ApplyVelocitiesResultConstPtr result;
        
//         goal.initial_state = startState.toROS();
//         goal.initial_motion_sample = sample.toROS();
//         goal.world_id = nWorldId;
//         goal.no_compensation = noCompensation;
//         // ROS_INFO("Sending goal %d with %d states at %.2fs", nWorldId, goal.initial_motion_sample.states.size(), ros::Time::now().toSec());
//         client.sendGoal(goal
//             // , boost::bind(&MochaPlanner::ApplyVelocitiesDoneCb, this, _1, _2)
//             // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
//             // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
//             );

//         double t2 = Tic();

//         // {
//         // boost::mutex::scoped_lock lock(m_mutexApplyVelocitiesInfo);
//         // m_vApplyVelocitiesGoalIds.push_back(goal.goal_id);
//         // m_vApplyVelocitiesMotionSamples.push_back(&sample);
//         // }

//         // DLOG(INFO) << "AV called:" 
//         //     << " world " << std::to_string(goal.world_id) 
//         // "\nstart " << std::to_string(VehicleState::fromROS(goal->initial_state))
//         ;

//         bool success;

//         // float timeout(15.0);
//         // timeout = 0.1 * goal.initial_motion_sample.states.size();
//         bool finished_before_timeout = client.waitForResult(ros::Duration(timeout));
//         if (finished_before_timeout)
//         {
//             success = true;
//             actionlib::SimpleClientGoalState state = client.getState();
//             // DLOG(INFO) << "ApplyVelocities finished: " << state.toString();

//             result = client.getResult();
//             sample.fromROS(result->motion_sample);
//         }
//         else
//         {
//             success = false;
//             DLOG(INFO) << "ApplyVelocities (" << std::to_string(nWorldId) << ") did not finish before the " << std::to_string(timeout) << "s time out.";
//         }

//         double t3 = Tic();
//         // ROS_INFO("Result %d received at %.2fs", nWorldId, ros::Time::now().toSec());

//         // spin_thread.join();

//         // while (client.getState() == actionlib::SimpleClientGoalState::PENDING
//         //     || client.getState() == actionlib::SimpleClientGoalState::ACTIVE
//         //     )
//         // {  
//         //     DLOG(INFO) << std::to_string(goal.world_id) << ": " << client.getState().toString();
//         //     ros::Rate(10).sleep();
//         // }

//         // result = client.getResult();
//         // sample.fromROS(result->motion_sample);

//         // DLOG(INFO) << "AV done:" 
//         //     << " world " << std::to_string(goal.world_id) 
//         //     << " " << client.getState().toString()
//         //     // << " poseOut " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x) 
//         //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x)  
//         //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.y)
//         //     //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.z) 
//         //     ;

//         // ROS_INFO("ApplyVelocities for world %d %s, server wait %.2fs, goal prep %.2fs, server call %.2fs, total %.2fs", nWorldId, (success ? "succeeded" : "failed"), t1-t0, t2-t1, t3-t2, t3-t0);

//         return;
//     }


//     ros::NodeHandle* nh;
//     const VehicleState& startState;
//     MotionSample& sample;
//     int nWorldId;
//     bool noCompensation;
//     std::string actionName;
//     float timeout;
//     actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction> client;

// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// };
// }

MochaVehicle::MochaVehicle(ros::NodeHandle& private_nh, ros::NodeHandle& nh) :
    m_private_nh(private_nh),
    m_nh(nh),
    // m_actionCreateSimpleServer_server(m_nh, "plan_car/create_server", boost::bind(&MochaVehicle::CreateServerService, this, _1), false),
    // m_actionApplyVelocities_server(m_nh, "plan_car/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server0(m_nh, "vehicle/0/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server1(m_nh, "vehicle/1/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server2(m_nh, "vehicle/2/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server3(m_nh, "vehicle/3/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server4(m_nh, "vehicle/4/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server5(m_nh, "vehicle/5/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server6(m_nh, "vehicle/6/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server7(m_nh, "vehicle/7/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server8(m_nh, "vehicle/8/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionApplyVelocities_server9(m_nh, "vehicle/9/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    // m_actionApplyVelocities_server(m_nh, "vehicle/apply_all_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), false),
    m_actionSetState_server(m_nh, "vehicle/set_state", boost::bind(&MochaVehicle::SetStateService, this, _1), false),
    m_actionGetState_server(m_nh, "vehicle/get_state", boost::bind(&MochaVehicle::GetStateService, this, _1), false),
    m_actionUpdateState_server(m_nh, "vehicle/update_state", boost::bind(&MochaVehicle::UpdateStateService, this, _1), false),
    // m_actionGetInertiaTensor_server (m_nh, "vehicle/manager/get_inertia_tensor",  boost::bind(&MochaVehicle::GetInertiaTensorService, this, _1), false),
    // m_actionSetNoDelay_server(m_nh, "vehicle/manager/set_no_delay", boost::bind(&MochaVehicle::SetNoDelayService, this, _1), false),
    m_actionRaycast_server(m_nh, "vehicle/raycast", boost::bind(&MochaVehicle::RaycastService, this, _1), false)//,
    // m_dGravity(0,0,-BULLET_MODEL_GRAVITY)
{
    ROS_INFO("[Vehicle] constructed.");
    DLOG(INFO) << "Constructed Vehicle";
    // Init();
    Initialize();
}

/////////////////////////////////////////////////////////////////////////////////////////
MochaVehicle::~MochaVehicle()
{
  //    close(sockFD);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::DebugDrawWorld(int worldId)
{
    BulletWorldInstance * pWorld = GetWorldInstance(worldId);
    if( pWorld->m_pDynamicsWorld != NULL ) {
        pWorld->m_pDynamicsWorld->debugDrawWorld();
        // m_guiHelper->autogenerateGraphicsObjects(pWorld->m_pDynamicsWorld);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
btVector3 MochaVehicle::GetUpVector(int upAxis,btScalar regularValue,btScalar upValue)
{
    btAssert(upAxis >= 0 && upAxis <= 2 && "bad up axis");

    btVector3 v(regularValue, regularValue, regularValue);
    v[upAxis] = upValue;

    return v;
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::GenerateStaticHull( const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh, btVector3& dMin, btVector3& dMax )
{
    aiMesh *pAIMesh;

    aiFace *pAIFace;

    for (size_t x = 0; x < pAINode->mNumMeshes; x++ )
    {
        pAIMesh = pAIScene->mMeshes[pAINode->mMeshes[x]];

        for (size_t y = 0; y < pAIMesh->mNumFaces; y++ )
        {
            pAIFace = &pAIMesh->mFaces[y];

            if ( pAIFace->mNumIndices != 3 )
            {
              continue;
            }


            aiVector3D v1 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[0]];
            aiVector3D v2 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[1]];
            aiVector3D v3 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[2]];

            dMin[0] = std::min((float)dMin[0],std::min(v1.x,std::min(v2.x, v3.x)));
            dMax[0] = std::max((float)dMax[0],std::min(v1.x,std::max(v2.x, v3.x)));

            dMin[1] = std::min((float)dMin[1],std::min(v1.y,std::min(v2.y, v3.y)));
            dMax[1] = std::max((float)dMax[1],std::max(v1.y,std::max(v2.y, v3.y)));

            dMin[2] = std::min((float)dMin[2],std::min(v1.z,std::min(v2.z, v3.z)));
            dMax[2] = std::max((float)dMax[2],std::max(v1.z,std::max(v2.z, v3.z)));

            triangleMesh.addTriangle( btVector3(v1.x * flScale, v1.y * flScale, v1.z * flScale),
            btVector3(v2.x * flScale, v2.y * flScale, v2.z * flScale),
            btVector3(v3.x * flScale, v3.y * flScale, v3.z * flScale),
            false );
        }
    }

    for (size_t x = 0; x < pAINode->mNumChildren; x++ )
    {
        GenerateStaticHull( pAIScene, pAINode->mChildren[x],parentTransform*pAINode->mChildren[x]->mTransformation, flScale, triangleMesh,dMin,dMax );
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::Init(btCollisionShape* pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, CarParameterMap &parameters, unsigned int numWorlds, bool real )
{
  //  if ( real ) {
  //    m_poseThreadPub = m_nh.advertise<nav_msgs::Odometry>("pose",1);
  //    m_commandThreadSub = m_nh.subscribe<carplanner_msgs::Command>("command", 1, boost::bind(&MochaVehicle::_CommandThreadFunc, this, _1));
  //  }

    // reset_mesh_frame = true;

    m_nNumWorlds = numWorlds;
    //initialize a number of worlds
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
        BulletWorldInstance *pWorld = new BulletWorldInstance();
        pWorld->m_nIndex = ii;

        DLOG(INFO) << "Initing world " << std::to_string(ii);
        _InitWorld(pWorld,pCollisionShape,dMin,dMax,false);

        //initialize the car
        DLOG(INFO) << "Initing vehicle " << std::to_string(ii);
        _InitVehicle(pWorld,parameters);

        m_vWorlds.push_back(pWorld);
    }

    InitROS();

  //  if (real) {
  //    m_pPoseThread = new boost::thread( std::bind( &MochaVehicle::_PoseThreadFunc, this ));
  //  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::Init(const struct aiScene *pAIScene, CarParameterMap &parameters, unsigned int numWorlds, bool real )
{
  //  if ( real ) {
  //    m_poseThreadPub = m_nh.advertise<nav_msgs::Odometry>("pose",1);
  //    m_commandThreadSub = m_nh.subscribe<carplanner_msgs::Command>("command", 1, boost::bind(&MochaVehicle::_CommandThreadFunc, this, _1));
  //  }

    // std::cout << "Initing scene" << std::endl;
    // const aiScene *pScene = aiImportFile( m_config.terrain_mesh_file.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    // std::cout << "Assigning target";
    // if(m_config.mode == MochaVehicle::Config::Mode::Simulation){
    //     std::cout << " to Simulation";
    //     pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
    //                                                       0, 1, 0, 0,
    //                                                       0, 0,-1, 0,
    //                                                       0, 0, 0, 1 );
    // }else{
    //     std::cout << " to Experiment";
    //     pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
    //                                                       0, 1, 0, 0,
    //                                                       0, 0,-1, 0,
    //                                                       0, 0, 0, 1 );
    // }
    // std::cout << std::endl;
    // btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    // btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    // btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    // /// Using pTriangleMesh and the terrain mesh, fill in the gaps to create a static hull.
    // std::cout << "Generating static hull" << std::endl;
    // MochaVehicle::GenerateStaticHull(pScene,pScene->mRootNode,pScene->mRootNode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);
    // /// Now generate the collision shape from the triangle mesh --- to know where the ground is.
    // std::cout << "Initing CollisionShape" << std::endl;
    // btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    // /// Initialize the car parameters.
    // CarParameterMap defaultParams;
    // CarParameters::LoadFromFile(m_config.params_file,defaultParams);
    // /// As well as the car that we're actually driving.
    // if ( m_config.mode == MochaVehicle::Config::Mode::Simulation )
    //     Init( pCollisionShape,dMin,dMax, defaultParams,1, false);
    // else
    //     Init( pCollisionShape,dMin,dMax, defaultParams,1, true);

    // //generate the triangle mesh
    // aiNode *pAINode = pAIScene->mRootNode;
    // btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    // btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    // btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    // GenerateStaticHull(pAIScene,pAINode,pAINode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);
    // btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    // Init(pCollisionShape, dMin, dMax, parameters, numWorlds, real);
}

void MochaVehicle::Initialize()
{
    InitializeParameters();
    InitializeScene();
    InitializeSimulations();
    InitializeExternals();

    ROS_INFO_NAMED("vehicle","[Vehicle] initialized.");
}

void MochaVehicle::InitializeParameters()
{
    ROS_INFO_NAMED("vehicle","[Vehicle] initializing parameters.");

    m_private_nh.param("params_file", m_config.params_file, m_config.params_file);
    m_private_nh.param("mode", (int&)m_config.mode, (int&)m_config.mode);
    m_private_nh.param("terrain_mesh_file", m_config.terrain_mesh_file, m_config.terrain_mesh_file);
    m_private_nh.param("car_mesh_file", m_config.car_mesh_file, m_config.car_mesh_file);
    m_private_nh.param("wheel_mesh_file", m_config.wheel_mesh_file, m_config.wheel_mesh_file);
    m_private_nh.param("map_frame", m_config.map_frame, m_config.map_frame);
    m_private_nh.param("base_link_frame", m_config.base_link_frame, m_config.base_link_frame);
    m_private_nh.param("opt_dim", m_config.opt_dim, m_config.opt_dim);

    CarParameters::LoadFromFile(m_config.params_file,m_DefaultParams);
}

void MochaVehicle::InitializeScene()
{
    ROS_INFO_NAMED("vehicle","[Vehicle] initializing scene.");

    // DLOG(INFO) << "Initing scene";
    const aiScene *pScene = aiImportFile( m_config.terrain_mesh_file.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    if(!pScene)
    {
        ROS_ERROR("Could not import mesh.");
        return;
    }
    pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0,
                                                        0, 0, 0, 1 );

    // std::cout << std::endl;
    m_vMinBounds = btVector3(DBL_MAX,DBL_MAX,DBL_MAX);
    m_vMaxBounds = btVector3(DBL_MIN,DBL_MIN,DBL_MIN);
    btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    /// Using pTriangleMesh and the terrain mesh, fill in the gaps to create a static hull.
    DLOG(INFO) << "Generating static hull" << std::endl;
    MochaVehicle::GenerateStaticHull(pScene,pScene->mRootNode,pScene->mRootNode->mTransformation,1.0,*pTriangleMesh,m_vMinBounds,m_vMaxBounds);
    /// Now generate the collision shape from the triangle mesh --- to know where the ground is.
    DLOG(INFO) << "Initing CollisionShape" << std::endl;
    // btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    m_pDefaultCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
}

void MochaVehicle::InitializeSimulations()
{
    ROS_INFO_NAMED("vehicle","[Vehicle] initializing simulations.");

    m_nNumWorlds = GetNumWorldsRequired(m_config.opt_dim);
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
        BulletWorldInstance *pWorld = new BulletWorldInstance();
        pWorld->m_nIndex = ii;

        DLOG(INFO) << "Initing world " << std::to_string(ii);
        _InitWorld(pWorld,m_pDefaultCollisionShape,m_vMinBounds,m_vMaxBounds,false);

        //initialize the car
        DLOG(INFO) << "Initing vehicle " << std::to_string(ii);
        _InitVehicle(pWorld,m_DefaultParams);

        m_vWorlds.push_back(pWorld);
    }
}

void MochaVehicle::InitializeExternals()
{
    ROS_INFO_NAMED("vehicle","[Vehicle] initializing externals.");

    m_terrainMeshPub = m_nh.advertise<mesh_msgs::TriangleMeshStamped>("vehicle/output_terrain_mesh",1);
    m_vehiclePub = m_nh.advertise<visualization_msgs::MarkerArray>("vehicle/output_vehicle_shape",1);
 
    m_terrainMeshSub = m_nh.subscribe<mesh_msgs::TriangleMeshStamped>("vehicle/input_terrain_mesh", 1, &MochaVehicle::meshCb, this);

    // m_pPublisherThread = new boost::thread( std::bind( &MochaVehicle::_PublisherFunc, this ));

    m_timerStatePubLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dStatePubRate), &MochaVehicle::StatePubLoopFunc, this);

    // m_timerTerrainMeshPubLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dTerrainMeshPubRate), &MochaVehicle::TerrainMeshPubLoopFunc, this);

    m_srvSetDriveMode = m_nh.advertiseService("vehicle/set_drive_mode", &MochaVehicle::SetDriveModeSvcCb, this);

    InitializeStatePublishers();
    InitializeCommandSubscribers();

    m_actionApplyVelocities_server0.start();
    m_actionApplyVelocities_server1.start();
    m_actionApplyVelocities_server2.start();
    m_actionApplyVelocities_server3.start();
    m_actionApplyVelocities_server4.start();
    m_actionApplyVelocities_server5.start();
    m_actionApplyVelocities_server6.start();
    m_actionApplyVelocities_server7.start();
    m_actionApplyVelocities_server8.start();
    m_actionApplyVelocities_server9.start();

    m_actionSetState_server.start();

    m_actionUpdateState_server.start();

    m_actionRaycast_server.start();
}

/////////////////////////////////

void MochaVehicle::Init()
{
  //  if ( real ) {
  //    m_poseThreadPub = m_nh.advertise<nav_msgs::Odometry>("pose",1);
  //    m_commandThreadSub = m_nh.subscribe<carplanner_msgs::Command>("command", 1, boost::bind(&MochaVehicle::_CommandThreadFunc, this, _1));
  //  }

    m_private_nh.param("params_file", m_config.params_file, m_config.params_file);
    m_private_nh.param("mode", (int&)m_config.mode, (int&)m_config.mode);
    m_private_nh.param("terrain_mesh_file", m_config.terrain_mesh_file, m_config.terrain_mesh_file);
    m_private_nh.param("car_mesh_file", m_config.car_mesh_file, m_config.car_mesh_file);
    m_private_nh.param("wheel_mesh_file", m_config.wheel_mesh_file, m_config.wheel_mesh_file);
    m_private_nh.param("map_frame", m_config.map_frame, m_config.map_frame);
    m_private_nh.param("base_link_frame", m_config.base_link_frame, m_config.base_link_frame);
    m_private_nh.param("opt_dim", m_config.opt_dim, m_config.opt_dim);

    DLOG(INFO) << "Initing scene";
    const aiScene *pScene = aiImportFile( m_config.terrain_mesh_file.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    if(!pScene)
    {
        ROS_ERROR("Could not import mesh.");
        return;
    }
    pScene->mRootNode->mTransformation = aiMatrix4x4( 1, 0, 0, 0,
                                                        0, 1, 0, 0,
                                                        0, 0, 1, 0,
                                                        0, 0, 0, 1 );

    // std::cout << std::endl;
    btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    /// Using pTriangleMesh and the terrain mesh, fill in the gaps to create a static hull.
    DLOG(INFO) << "Generating static hull" << std::endl;
    MochaVehicle::GenerateStaticHull(pScene,pScene->mRootNode,pScene->mRootNode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);
    /// Now generate the collision shape from the triangle mesh --- to know where the ground is.
    DLOG(INFO) << "Initing CollisionShape" << std::endl;
    btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    /// Initialize the car parameters.
    CarParameterMap defaultParams;
    CarParameters::LoadFromFile(m_config.params_file,defaultParams);
    
    /// As well as the car that we're actually driving.
    // if ( m_config.mode == MochaVehicle::Config::Mode::Simulation )
    //     Init( pCollisionShape,dMin,dMax, defaultParams,1, false);
    // else
    //     Init( pCollisionShape,dMin,dMax, defaultParams,1, true);

    // #define OPT_DIM 4
    Init( pCollisionShape,dMin,dMax, defaultParams, GetNumWorldsRequired(m_config.opt_dim), false);

    // //generate the triangle mesh
    // aiNode *pAINode = pAIScene->mRootNode;
    // btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    // btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    // btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    // GenerateStaticHull(pAIScene,pAINode,pAINode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);
    // btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    // Init(pCollisionShape, dMin, dMax, parameters, numWorlds, real);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh)
{
    //add this to the shapes
    pWorld->m_pTerrainShape = pGroundShape;
    pWorld->m_vCollisionShapes.push_back(pWorld->m_pTerrainShape);
    //pWorld->m_vCollisionShapes.push_back(groundShape);

    pWorld->m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
    pWorld->m_pDispatcher = new btCollisionDispatcher(pWorld->m_pCollisionConfiguration);

    //btVector3 worldMin(pHeightMap->GetMinX(),pHeightMap->GetMinY(),dMin(2)-100);
    //btVector3 worldMax(pHeightMap->GetMaxX(),pHeightMap->GetMaxY(),dMax(2)+100);
    //btVector3 worldMin(-1000,-1000,-1000);
    //btVector3 worldMax(1000,1000,1000);
    pWorld->m_pOverlappingPairCache = new btAxisSweep3(dMin,dMax);
    pWorld->m_pConstraintSolver = new btSequentialImpulseConstraintSolver();
    pWorld->m_pDynamicsWorld = new btDiscreteDynamicsWorld(pWorld->m_pDispatcher,pWorld->m_pOverlappingPairCache,pWorld->m_pConstraintSolver,pWorld->m_pCollisionConfiguration);
    // pWorld->m_pDynamicsWorld->setDebugDrawer(&pWorld->m_DebugDrawer);
    // pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_FastWireframe);
    //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
    //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);

    // Temporary fix
    pWorld->m_pDynamicsWorld->setBroadphase(pWorld->m_pOverlappingPairCache);
    //set the gravity vector
    // pWorld->m_pDynamicsWorld->setGravity(btVector3(0,0,-BULLET_MODEL_GRAVITY));
    // float gravity[3];
    // gravity[CAR_UP_AXIS] = -BULLET_MODEL_GRAVITY;
    btVector3 gravity(0,0,0);
    gravity[CAR_UP_AXIS] = -BULLET_MODEL_GRAVITY;
    pWorld->m_pDynamicsWorld->setGravity(gravity);

    btTransform tr;
    tr.setIdentity();
    if(centerMesh == true){
        tr.setOrigin(btVector3((dMax[0] + dMin[0])/2,(dMax[1] + dMin[1])/2,(dMax[2] + dMin[2])/2));
    }

    // if(pWorld->m_pTerrainBody != NULL) {
    //   pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
    //   delete pWorld->m_pTerrainBody;
    // }
    pWorld->m_pTerrainBody = _LocalAddRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);


    // btAssert((!pWorld->m_pTerrainShape || pWorld->m_pTerrainShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    // btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
    // btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,pWorld->m_pTerrainShape,btVector3(0,0,0));
    // btRigidBody* body = new btRigidBody(cInfo);
    // body->setContactProcessingThreshold(BT_LARGE_FLOAT);
    // pWorld->m_pTerrainBody = body;
    // pWorld->m_pDynamicsWorld->addRigidBody(body,COL_GROUND,COL_RAY|COL_CAR);

    // pWorld->enableGUI(true);

    //create the ground object
    // _LocalAddRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);

    //m_pHeightMap = pHeightMap;


    // m_guiHelper->createPhysicsDebugDrawer(pWorld->m_pDynamicsWorld);
    // m_guiHelper->setUpAxis(2);
    // m_guiHelper->autogenerateGraphicsObjects(pWorld->m_pDynamicsWorld);
}

/////////////////////////////////////////////////////////////////////////////////////////
// void MochaVehicle::_InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters)
// {
//     pWorld->m_Parameters = parameters;

//     //delete any previous collision shapes
//     for(int ii = 0 ; ii < pWorld->m_vVehicleCollisionShapes.size() ; ii++) {
//         delete pWorld->m_vVehicleCollisionShapes[ii];
//     }
//     pWorld->m_vVehicleCollisionShapes.clear();

//     pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(pWorld->m_Parameters[CarParameters::WheelBase],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
//     // pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(0.5,0.7,0.7)*2);
//     pWorld->m_vVehicleCollisionShapes.push_back(pWorld->m_pVehicleChassisShape);

//     /*btCompoundShape* compound = new btCompoundShape();
//     pWorld->m_vVehicleCollisionShapes.push_back(compound);
//     btTransform localTrans;
//     localTrans.setIdentity();
//     //localTrans effectively shifts the center of mass with respect to the chassis
//     localTrans.setOrigin(btVector3(0,0,0));

//     compound->addChildShape(localTrans,pWorld->m_pVehicleChassisShape);*/

//     btTransform tr;
//     tr.setIdentity();
//     tr.setOrigin(btVector3(0,0,0));
//     btVector3 vWheelDirectionCS0(0,0,1); //wheel direction is z
//     btVector3 vWheelAxleCS(0,1,0); //wheel axle in y direction

//     if(pWorld->m_pCarChassis != NULL) {
//         pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pCarChassis);
//         delete pWorld->m_pCarChassis;
//     }

//     pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters[CarParameters::Mass],tr,pWorld->m_pVehicleChassisShape, COL_CAR,COL_NOTHING);//chassisShape);
//     //pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters.m_dMass,tr,compound, COL_CAR,COL_GROUND);//chassisShape);

//     /// create vehicle
//     pWorld->m_pVehicleRayCaster = new DefaultVehicleRaycaster(pWorld->m_pDynamicsWorld);

//     if( pWorld->m_pVehicle != NULL ) {
//         pWorld->m_pDynamicsWorld->removeVehicle(pWorld->m_pVehicle);
//         delete pWorld->m_pVehicle;
//     }

//     pWorld->m_Tuning.m_frictionSlip = pWorld->m_Parameters[CarParameters::TractionFriction];
//     pWorld->m_Tuning.m_suspensionCompression = pWorld->m_Parameters[CarParameters::CompDamping];
//     pWorld->m_Tuning.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
//     pWorld->m_Tuning.m_suspensionDamping = pWorld->m_Parameters[CarParameters::ExpDamping];
//     pWorld->m_Tuning.m_maxSuspensionForce = pWorld->m_Parameters[CarParameters::MaxSuspForce];
//     pWorld->m_Tuning.m_maxSuspensionTravelCm = pWorld->m_Parameters[CarParameters::MaxSuspTravel]*100.0;

//     pWorld->m_pVehicle = new RaycastVehicle(pWorld->m_Tuning,pWorld->m_pCarChassis,pWorld->m_pVehicleRayCaster,&pWorld->m_pDynamicsWorld->getSolverInfo());
//     pWorld->m_pVehicle->setCoordinateSystem(CAR_RIGHT_AXIS,CAR_UP_AXIS,CAR_FORWARD_AXIS);
//     ///never deactivate the vehicle
//     pWorld->m_pCarChassis->forceActivationState(DISABLE_DEACTIVATION);
//     pWorld->m_pDynamicsWorld->addVehicle(pWorld->m_pVehicle);

//     // front right wheel
//     bool bIsFrontWheel=true;
//     btVector3 connectionPointCS0(pWorld->m_Parameters[CarParameters::WheelBase]/2,pWorld->m_Parameters[CarParameters::Width]/2-(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
//     pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
//     // front left wheel
//     connectionPointCS0 = btVector3(pWorld->m_Parameters[CarParameters::WheelBase]/2, -pWorld->m_Parameters[CarParameters::Width]/2+(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]),pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
//     pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
//     // back left wheel
//     bIsFrontWheel = false;
//     connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase]/2,-pWorld->m_Parameters[CarParameters::Width]/2+(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
//     pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
//     // back right wheel
//     connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase]/2,pWorld->m_Parameters[CarParameters::Width]/2-(0.3*pWorld->m_Parameters[CarParameters::WheelWidth]), pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
//     pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);

//     for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
//     {
//         WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
//         wheel.m_rollInfluence = pWorld->m_Parameters[CarParameters::RollInfluence];
//         Sophus::SE3d wheelTransform(Sophus::SO3d(),
//         Eigen::Vector3d(wheel.m_chassisConnectionPointCS[0],wheel.m_chassisConnectionPointCS[1],wheel.m_chassisConnectionPointCS[2] /*+ wheel.getSuspensionRestLength()/2*/));
//         pWorld->m_vWheelTransforms.push_back(wheelTransform);
//     }

//     pWorld->m_pVehicle->SetDynamicFrictionCoefficient(pWorld->m_Parameters[CarParameters::DynamicFrictionCoef]);
//     pWorld->m_pVehicle->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[CarParameters::StaticSideFrictionCoef]);
//     pWorld->m_pVehicle->SetSlipCoefficient(pWorld->m_Parameters[CarParameters::SlipCoefficient]);
//     pWorld->m_pVehicle->SetMagicFormulaCoefficients(pWorld->m_Parameters[CarParameters::MagicFormula_B],
//                                                     pWorld->m_Parameters[CarParameters::MagicFormula_C],
//                                                     pWorld->m_Parameters[CarParameters::MagicFormula_E]);


//     //reset all parameters
//     //m_pCarChassis->setCenterOfMassTransform(btTransform::getIdentity());
//     pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
//     pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
//     pWorld->m_pDynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(pWorld->m_pCarChassis->getBroadphaseHandle(),pWorld->m_pDynamicsWorld->getDispatcher());
//     if (pWorld->m_pVehicle)
//     {
//       pWorld->m_pVehicle->resetSuspension();
//       for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
//       {
//         //synchronize the wheels with the (interpolated) chassis worldtransform
//         pWorld->m_pVehicle->updateWheelTransform(i,true);
//       }
//     }
//     pWorld->m_vehicleBackup.SaveState(pWorld->m_pVehicle);
// }

// void MochaVehicle::_InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters)
// {
//     pWorld->m_Parameters = parameters;

//     //delete any previous collision shapes
//     for(int ii = 0 ; ii < pWorld->m_vVehicleCollisionShapes.size() ; ii++) {
//         delete pWorld->m_vVehicleCollisionShapes[ii];
//     }
//     pWorld->m_vVehicleCollisionShapes.clear();

//     pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(pWorld->m_Parameters[CarParameters::WheelBase]+pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
//     // pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(0.5,0.7,0.7)*2);
//     pWorld->m_vVehicleCollisionShapes.push_back(pWorld->m_pVehicleChassisShape);

//     /*btCompoundShape* compound = new btCompoundShape();
//     pWorld->m_vVehicleCollisionShapes.push_back(compound);
//     btTransform localTrans;
//     localTrans.setIdentity();
//     //localTrans effectively shifts the center of mass with respect to the chassis
//     localTrans.setOrigin(btVector3(0,0,0));

//     compound->addChildShape(localTrans,pWorld->m_pVehicleChassisShape);*/


//     pWorld->m_pVehicleRayCaster = new DefaultVehicleRaycaster(pWorld->m_pDynamicsWorld);

//     btTransform tr;
//     tr.setIdentity();
//     tr.setOrigin(btVector3(0,0,0));
//     btVector3 vWheelDirectionCS0(0,0,1); //wheel direction is z
//     btVector3 vWheelAxleCS(0,1,0); //wheel axle in y direction

//     if(pWorld->m_pCarChassis != NULL) {
//         pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pCarChassis);
//         delete pWorld->m_pCarChassis;
//     }

//     pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters[CarParameters::Mass],tr,pWorld->m_pVehicleChassisShape, COL_CAR,COL_GROUND);//chassisShape);
//     //pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters.m_dMass,tr,compound, COL_CAR,COL_GROUND);//chassisShape);


//     if( pWorld->m_pVehicle != NULL ) {
//         pWorld->m_pDynamicsWorld->removeVehicle(pWorld->m_pVehicle);
//         delete pWorld->m_pVehicle;
//     }

//     // pWorld->m_Tuning.m_frictionSlip = pWorld->m_Parameters[CarParameters::TractionFriction];
//     // pWorld->m_Tuning.m_suspensionCompression = pWorld->m_Parameters[CarParameters::CompDamping];
//     // pWorld->m_Tuning.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
//     // pWorld->m_Tuning.m_suspensionDamping = pWorld->m_Parameters[CarParameters::ExpDamping];
//     // pWorld->m_Tuning.m_maxSuspensionForce = pWorld->m_Parameters[CarParameters::MaxSuspForce];
//     // pWorld->m_Tuning.m_maxSuspensionTravelCm = pWorld->m_Parameters[CarParameters::MaxSuspTravel]*100.0;


//     pWorld->m_pVehicle = new btHinge2Vehicle(pWorld->m_pCarChassis);
// 	pWorld->m_pDynamicsWorld->addVehicle(pWorld->m_pVehicle);
// 	// m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

// 	// pos = pWorld->m_pVehicle->getChassisWorldTransform().getOrigin();
// 	// prtmsg += "\tchassisPos: "+std::to_string(pWorld->m_pVehicle->getChassisWorldTransform().getOrigin()[0])+" "+std::to_string(pWorld->m_pVehicle->getChassisWorldTransform().getOrigin()[1])+" "+std::to_string(pWorld->m_pVehicle->getChassisWorldTransform().getOrigin()[2])+"\n";

// 	// add wheels
// 	btVector3 wheelPosCS[4];
// 	wheelPosCS[0] = btVector3(	-btScalar(pWorld->m_Parameters[CarParameters::Width] + pWorld->m_Parameters[CarParameters::WheelWidth]),	btScalar(-pWorld->m_Parameters[CarParameters::Height]),	 btScalar(pWorld->m_Parameters[CarParameters::WheelBase])	); // front right
// 	wheelPosCS[1] = btVector3(	 btScalar(pWorld->m_Parameters[CarParameters::Width] + pWorld->m_Parameters[CarParameters::WheelWidth]),	btScalar(-pWorld->m_Parameters[CarParameters::Height]),	 btScalar(pWorld->m_Parameters[CarParameters::WheelBase])	); // front left
// 	wheelPosCS[2] = btVector3(	 btScalar(pWorld->m_Parameters[CarParameters::Width] + pWorld->m_Parameters[CarParameters::WheelWidth]),	btScalar(-pWorld->m_Parameters[CarParameters::Height]),	-btScalar(pWorld->m_Parameters[CarParameters::WheelBase])	); // back left
// 	wheelPosCS[3] = btVector3(	-btScalar(pWorld->m_Parameters[CarParameters::Width] + pWorld->m_Parameters[CarParameters::WheelWidth]),	btScalar(-pWorld->m_Parameters[CarParameters::Height]),	-btScalar(pWorld->m_Parameters[CarParameters::WheelBase])	); // back right

// 	btCollisionShape* wheelShape = new btCylinderShapeX(btVector3(pWorld->m_Parameters[CarParameters::WheelWidth], pWorld->m_Parameters[CarParameters::WheelRadius], pWorld->m_Parameters[CarParameters::WheelRadius]));
// 	pWorld->m_vVehicleCollisionShapes.push_back(wheelShape);

// 	btVector4 wheelColor(1,0,0,1);

// 	for (int i = 0; i < 4; i++)
// 	{
// 		btTransform wheelTranWS = pWorld->m_pVehicle->getChassisWorldTransform() * btTransform(btQuaternion(0,0,0,1), wheelPosCS[i]);
// 		btRigidBody* wheelBody = _LocalAddRigidBody(pWorld, 0.01, wheelTranWS, wheelShape, COL_CAR, COL_GROUND);

// 		// prtmsg += "Adding wheel "+std::to_string(i)+":\n";

// 		btTypedConstraint* constraint = pWorld->m_pVehicle->addWheel(wheelBody);
// 		pWorld->m_pDynamicsWorld->addConstraint(constraint, true);

// 		pWorld->m_pVehicle->getWheel(i)->setDamping(pWorld->m_Parameters[CarParameters::ExpDamping]);
// 		pWorld->m_pVehicle->getWheel(i)->setStiffness(pWorld->m_Parameters[CarParameters::Stiffness]);

// 		if (i<2) // front wheels
// 		{
// 			pWorld->m_pVehicle->getWheel(i)->setSteeringEnabled(true);
// 		}
// 		// else
// 		// {
// 			pWorld->m_pVehicle->getWheel(i)->setMotorEnabled(true);
// 			pWorld->m_pVehicle->getWheel(i)->setBrakeEnabled(true);
// 		// }
		
// 		// prtmsg += pWorld->m_pVehicle->wheel2str(i,"","","\t","");
// 	}
// 	// std::cout << prtmsg;
// }

void MochaVehicle::_InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters)
{
    pWorld->m_Parameters = parameters;

    pWorld->m_pVehicleRayCaster = new DefaultVehicleRaycaster(pWorld->m_pDynamicsWorld);

    //delete any previous collision shapes
    for(int ii = 0 ; ii < pWorld->m_vVehicleCollisionShapes.size() ; ii++) {
        delete pWorld->m_vVehicleCollisionShapes[ii];
    }
    pWorld->m_vVehicleCollisionShapes.clear();

    if(pWorld->m_pCarChassis != NULL) {
        pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pCarChassis);
        delete pWorld->m_pCarChassis;
    }

    if( pWorld->m_pVehicle != NULL ) {
        pWorld->m_pDynamicsWorld->removeVehicle(pWorld->m_pVehicle);
        delete pWorld->m_pVehicle;
    }

    // spawn collision vehicle
    // pWorld->m_pVehicle = new btDefaultHinge2Vehicle(
    //     pWorld->m_Parameters[CarParameters::Width],
    //     pWorld->m_Parameters[CarParameters::Height],
    //     pWorld->m_Parameters[CarParameters::WheelBase]+pWorld->m_Parameters[CarParameters::WheelRadius],
    //     pWorld->m_Parameters[CarParameters::Mass],
    //     pWorld->m_Parameters[CarParameters::WheelRadius],
    //     pWorld->m_Parameters[CarParameters::WheelWidth],
    //     pWorld->m_Parameters[CarParameters::WheelMass],
    //     pWorld->m_Parameters[CarParameters::Stiffness],
    //     pWorld->m_Parameters[CarParameters::CompDamping],
    //     pWorld->m_Parameters[CarParameters::MaxSteering]/2,
    //     10.f/pWorld->m_Parameters[CarParameters::WheelRadius],
    //     pWorld->m_Parameters[CarParameters::SuspRestLength],
    //     1.f,
    //     pWorld->m_Parameters[CarParameters::StallTorqueCoef]);
	// pWorld->m_pVehicle->setCoordinateSystem(CAR_RIGHT_AXIS,CAR_UP_AXIS,CAR_FORWARD_AXIS); // car in nwu, solver in ned
	// dynamic_cast<btDefaultHinge2Vehicle*>(pWorld->m_pVehicle)->spawn(pWorld->m_pDynamicsWorld);

    // spawn raycast vehicle
    pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(pWorld->m_Parameters[CarParameters::WheelBase]+pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
  // pWorld->m_pVehicleChassisShape = new btBoxShape(btVector3(0.5,0.7,0.7)*2);
    pWorld->m_vVehicleCollisionShapes.push_back(pWorld->m_pVehicleChassisShape);
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0,0,0));
    btVector3 vWheelDirectionCS0(0,0,-1); //wheel direction is z
    btVector3 vWheelAxleCS(0,-1,0); //wheel axle in y direction
    pWorld->m_pCarChassis = _LocalAddRigidBody(pWorld,pWorld->m_Parameters[CarParameters::Mass],tr,pWorld->m_pVehicleChassisShape, COL_CAR,COL_GROUND);//chassisShape);
    pWorld->m_Tuning.m_frictionSlip = pWorld->m_Parameters[CarParameters::TractionFriction];
    pWorld->m_Tuning.m_suspensionCompression = pWorld->m_Parameters[CarParameters::CompDamping];
    pWorld->m_Tuning.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
    pWorld->m_Tuning.m_suspensionDamping = pWorld->m_Parameters[CarParameters::ExpDamping];
    pWorld->m_Tuning.m_maxSuspensionForce = pWorld->m_Parameters[CarParameters::MaxSuspForce];
    pWorld->m_Tuning.m_maxSuspensionTravelCm = pWorld->m_Parameters[CarParameters::MaxSuspTravel]*100.0;
    pWorld->m_pVehicle = new RaycastVehicle(pWorld->m_Tuning,pWorld->m_pCarChassis,pWorld->m_pVehicleRayCaster,&pWorld->m_pDynamicsWorld->getSolverInfo());
    pWorld->m_pVehicle->setCoordinateSystem(CAR_RIGHT_AXIS,CAR_UP_AXIS,CAR_FORWARD_AXIS);
    ///never deactivate the vehicle
    pWorld->m_pCarChassis->forceActivationState(DISABLE_DEACTIVATION);
    pWorld->m_pDynamicsWorld->addVehicle(pWorld->m_pVehicle);
    // front right wheel
    bool bIsFrontWheel=true;
    btVector3 connectionPointCS0(pWorld->m_Parameters[CarParameters::WheelBase],-pWorld->m_Parameters[CarParameters::Width]-(0.5*pWorld->m_Parameters[CarParameters::WheelWidth]), -pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    // front left wheel
    connectionPointCS0 = btVector3(pWorld->m_Parameters[CarParameters::WheelBase], pWorld->m_Parameters[CarParameters::Width]+(0.5*pWorld->m_Parameters[CarParameters::WheelWidth]),-pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    // back left wheel
    bIsFrontWheel = false;
    connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase],pWorld->m_Parameters[CarParameters::Width]+(0.5*pWorld->m_Parameters[CarParameters::WheelWidth]), -pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    // back right wheel
    connectionPointCS0 = btVector3(-pWorld->m_Parameters[CarParameters::WheelBase],-pWorld->m_Parameters[CarParameters::Width]-(0.5*pWorld->m_Parameters[CarParameters::WheelWidth]), -pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    pWorld->m_pVehicle->addWheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[CarParameters::SuspRestLength],pWorld->m_Parameters[CarParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    {
        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
        wheel.m_rollInfluence = pWorld->m_Parameters[CarParameters::RollInfluence];
        Sophus::SE3d wheelTransform(Sophus::SO3d(),
            Eigen::Vector3d(wheel.m_chassisConnectionPointCS[0],wheel.m_chassisConnectionPointCS[1],wheel.m_chassisConnectionPointCS[2] /*+ wheel.getSuspensionRestLength()/2*/));
        pWorld->m_vWheelTransforms.push_back(wheelTransform);
        // ROS_INFO("- %d wheels %f %f %f", i, pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getZ());
    }
    pWorld->m_pVehicle->SetDynamicFrictionCoefficient(pWorld->m_Parameters[CarParameters::DynamicFrictionCoef]);
    pWorld->m_pVehicle->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[CarParameters::StaticSideFrictionCoef]);
    pWorld->m_pVehicle->SetSlipCoefficient(pWorld->m_Parameters[CarParameters::SlipCoefficient]);
    pWorld->m_pVehicle->SetMagicFormulaCoefficients(pWorld->m_Parameters[CarParameters::MagicFormula_B],
        pWorld->m_Parameters[CarParameters::MagicFormula_C],
        pWorld->m_Parameters[CarParameters::MagicFormula_E]);
    //reset all parameters
    //m_pCarChassis->setCenterOfMassTransform(btTransform::getIdentity());
    pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
    pWorld->m_pDynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(pWorld->m_pCarChassis->getBroadphaseHandle(),pWorld->m_pDynamicsWorld->getDispatcher());
    if (pWorld->m_pVehicle)
    {
        pWorld->m_pVehicle->resetSuspension();
        for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            pWorld->m_pVehicle->updateWheelTransform(i,true);
        }
    }

    // pWorld->m_pCarChassis = pWorld->m_pVehicle->getRigidBody();
    // pWorld->m_pVehicleChassisShape = pWorld->m_pCarChassis->getCollisionShape();
    // btCollisionShape* wheelShape = pWorld->m_pVehicle->getWheel(0)->getShape();
    // pWorld->m_vVehicleCollisionShapes.push_back(pWorld->m_pVehicleChassisShape);
	// pWorld->m_vVehicleCollisionShapes.push_back(wheelShape);
    pWorld->m_vehicleBackup.SaveState(pWorld->m_pVehicle);
    SyncStateToVehicle(pWorld);
}

////////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::InitROS()
{
    DLOG(INFO) << "Initing ROS";

    // m_private_nh.param("params_file", m_config.params_file, m_config.params_file);
    // m_private_nh.param("mode", (int&)m_config.mode, (int&)m_config.mode);
    // m_private_nh.param("terrain_mesh_file", m_config.terrain_mesh_file, m_config.terrain_mesh_file);
    // m_private_nh.param("car_mesh_file", m_config.car_mesh_file, m_config.car_mesh_file);
    // m_private_nh.param("wheel_mesh_file", m_config.wheel_mesh_file, m_config.wheel_mesh_file);

    std::string terrain_mesh_topic;
    m_private_nh.param("terrain_mesh_topic", terrain_mesh_topic, std::string("vehicle/input_terrain_mesh"));

    // m_statePub = m_nh.advertise<carplanner_msgs::VehicleState>("state",1);
    m_terrainMeshPub = m_nh.advertise<mesh_msgs::TriangleMeshStamped>("vehicle/output_terrain_mesh",1);
    m_vehiclePub = m_nh.advertise<visualization_msgs::MarkerArray>("vehicle/output_vehicle_shape",1);
    // m_meshSub = m_nh.subscribe<mesh_msgs::TriangleMeshStamped>(m_meshSubTopic, 1, boost::bind(&MochaVehicle::_meshCB, this, _1))
    // m_terrainMeshSub = m_nh.subscribe<mesh_msgs::TriangleMeshStamped>("input_terrain_mesh", 1, boost::bind(&MochaVehicle::_meshCB, this, _1));
    // m_meshSub2 = m_nh.subscribe<mesh_msgs::TriangleMeshStamped>("/fake_mesh_publisher/mesh", 1, boost::bind(&MochaVehicle::_meshCB, this, _1));

    // m_resetmeshSrv = m_nh.advertiseService("reset_mesh", &MochaVehicle::ResetMesh, this);

    m_terrainMeshSub = m_nh.subscribe<mesh_msgs::TriangleMeshStamped>(terrain_mesh_topic, 1, &MochaVehicle::meshCb, this);

    m_pPublisherThread = new boost::thread( std::bind( &MochaVehicle::_PublisherFunc, this ));
    // m_pStatePublisherThread = new boost::thread( std::bind( &MochaVehicle::_StatePublisherFunc, this ));

    m_timerStatePubLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dStatePubRate), &MochaVehicle::StatePubLoopFunc, this);

    // m_pTerrainMeshPublisherThread = new boost::thread( std::bind( &MochaVehicle::_TerrainMeshPublisherFunc, this ));
    m_timerTerrainMeshPubLoop = m_private_nh.createTimer(ros::Duration(1.0/m_dTerrainMeshPubRate), &MochaVehicle::TerrainMeshPubLoopFunc, this);

    m_srvSetDriveMode = m_nh.advertiseService("vehicle/set_drive_mode", &MochaVehicle::SetDriveModeSvcCb, this);

    InitializeStatePublishers();
    InitializeCommandSubscribers();

    DLOG(INFO) << "Starting services";

    // m_actionCreateSimpleServer_server.start();

    // m_actionApplyVelocities_server = new actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction>(m_nh, "plan_car/apply_velocities", boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1));
    // m_actionApplyVelocities_server.registerGoalCallback(boost::bind(&MochaVehicle::ApplyVelocitiesGoalCb, this, _1));
    // m_actionApplyVelocities_server.start();
    m_actionApplyVelocities_server0.start();
    m_actionApplyVelocities_server1.start();
    m_actionApplyVelocities_server2.start();
    m_actionApplyVelocities_server3.start();
    m_actionApplyVelocities_server4.start();
    m_actionApplyVelocities_server5.start();
    m_actionApplyVelocities_server6.start();
    m_actionApplyVelocities_server7.start();
    m_actionApplyVelocities_server8.start();
    m_actionApplyVelocities_server9.start();

    m_actionSetState_server.start();

    // m_actionUpdateState_server = new actionlib::SimpleActionServer<carplanner_msgs::UpdateStateAction>(m_nh, "plan_car/update_state", boost::bind(&MochaVehicle::UpdateStateService, this, _1));
    m_actionUpdateState_server.start();

    // m_actionGetGravityCompensation_server = new actionlib::SimpleActionServer<carplanner_msgs::GetGravityCompensationAction>(m_nh, "plan_car/get_gravity_compensation", boost::bind(&MochaVehicle::GetGravityCompensationService, this, _1));
    // m_actionGetGravityCompensation_server->start();

    // m_actionGetControlDelay_server = new actionlib::SimpleActionServer<carplanner_msgs::GetControlDelayAction>(m_nh, "plan_car/get_control_delay", boost::bind(&MochaVehicle::GetControlDelayService, this, _1));
    // m_actionGetControlDelay_server->start();

    // m_actionGetInertiaTensor_server = new actionlib::SimpleActionServer<carplanner_msgs::GetInertiaTensorAction>(m_nh, "plan_car/get_inertia_tensor", boost::bind(&MochaVehicle::GetInertiaTensorService, this, _1));
    // m_actionGetInertiaTensor_server.start();

    // m_actionSetNoDelay_server = new actionlib::SimpleActionServer<carplanner_msgs::SetNoDelayAction>(m_nh, "plan_car/set_no_delay", boost::bind(&MochaVehicle::SetNoDelayService, this, _1));
    // m_actionSetNoDelay_server.start();

    m_actionRaycast_server.start();
    
}

void MochaVehicle::SetStateService(const carplanner_msgs::SetStateGoalConstPtr &goal)
{
    carplanner_msgs::SetStateFeedback actionSetState_feedback;
    carplanner_msgs::SetStateResult actionSetState_result;

    // DLOG(INFO) << "SetState service called.";
    
    try
    {
        VehicleState stateIn;
        stateIn.fromROS(goal->state_in);
        SetState(goal->world_id, stateIn, goal->raycast);
        VehicleState stateOut;
        GetVehicleState(goal->world_id, stateOut);
        actionSetState_result.state_out = stateOut.toROS();
        m_actionSetState_server.setSucceeded(actionSetState_result);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR(e.what());
        m_actionSetState_server.setAborted();
    }
    return;
}

void MochaVehicle::GetStateService(const carplanner_msgs::GetStateGoalConstPtr &goal)
{
    carplanner_msgs::GetStateFeedback actionGetState_feedback;
    carplanner_msgs::GetStateResult actionGetState_result;

    // DLOG(INFO) << "GetState service called.";
    
    try
    {
        VehicleState stateOut;
        GetVehicleState(goal->world_id, stateOut);
        actionGetState_result.state_out = stateOut.toROS();
        m_actionGetState_server.setSucceeded(actionGetState_result);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR(e.what());
        m_actionGetState_server.setAborted();
    }
    return;
}

void MochaVehicle::UpdateStateService(const carplanner_msgs::UpdateStateGoalConstPtr &goal)
{
    // m_updatestate_af.sequence.clear();
    // m_updatestate_af.sequence.push_back(0);
    // as_->publishFeedback(m_updatestate_af);
    // as_result_.sequence = feedback_.sequence;
    // as_->setSucceeded(as_result_);

    carplanner_msgs::UpdateStateFeedback actionUpdateState_feedback;
    carplanner_msgs::UpdateStateResult actionUpdateState_result;

    // DLOG(INFO) << "UpdateState service called.";
    
    try
    {
        UpdateState(
            goal->worldId, 
            ControlCommand(
                goal->command.force, 
                goal->command.curvature, 
                Eigen::Vector3d(
                    goal->command.torques[0],
                    goal->command.torques[1],
                    goal->command.torques[2]), 
                goal->command.dt, 
                goal->command.dphi), 
            goal->command.dt, 
            goal->noDelay);
        VehicleState stateOut;
        GetVehicleState(goal->worldId, stateOut);
        actionUpdateState_result.state = stateOut.toROS();
        m_actionUpdateState_server.setSucceeded(actionUpdateState_result);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR(e.what());
        m_actionUpdateState_server.setAborted();
    }
    return;
}

// void MochaVehicle::GetGravityCompensationService(const carplanner_msgs::GetGravityCompensationGoalConstPtr &goal)
// {
//     carplanner_msgs::GetGravityCompensationResult actionGetGravityCompensation_result;

//     DLOG(INFO) << "GetGravityCompensation service called.";
    
//     double val = -(GetTotalGravityForce(GetWorldInstance(goal->worldId))/GetWorldInstance(goal->worldId)->m_Parameters[CarParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/;

//     actionGetGravityCompensation_result.val = val;
//     m_actionGetGravityCompensation_server->setSucceeded(actionGetGravityCompensation_result);
// }

// void MochaVehicle::GetControlDelayService(const carplanner_msgs::GetControlDelayGoalConstPtr &goal)
// {
//     carplanner_msgs::GetControlDelayResult actionGetControlDelay_result;

//     // DLOG(INFO) << "GetControlDelay service called.";
    
//     double val = GetParameters(goal->worldId)[CarParameters::ControlDelay];

//     actionGetControlDelay_result.val = val;
//     m_actionGetControlDelay_server->setSucceeded(actionGetControlDelay_result);
// }

// void MochaVehicle::GetInertiaTensorService(const carplanner_msgs::GetInertiaTensorGoalConstPtr &goal)
// {
//     carplanner_msgs::GetInertiaTensorResult actionGetInertiaTensor_result;

//     // DLOG(INFO) << "GetInertiaTensor service called.";

//     Eigen::Vector3d vals = GetVehicleInertiaTensor(goal->world_id);
//     // actionGetInertiaTensor_result.vals[0] = vals[0];
//     // actionGetInertiaTensor_result.vals[1] = vals[1];
//     // actionGetInertiaTensor_result.vals[2] = vals[2];
//     actionGetInertiaTensor_result.vals.push_back(vals[0]);
//     actionGetInertiaTensor_result.vals.push_back(vals[1]);
//     actionGetInertiaTensor_result.vals.push_back(vals[2]);

//     m_actionGetInertiaTensor_server.setSucceeded(actionGetInertiaTensor_result);
// }

// void MochaVehicle::SetNoDelayService(const carplanner_msgs::SetNoDelayGoalConstPtr &goal)
// {
//     carplanner_msgs::SetNoDelayResult actionSetNoDelay_result;

//     // DLOG(INFO) << "SetNoDelay service called.";

//     SetNoDelay(goal->no_delay);

//     m_actionSetNoDelay_server.setSucceeded(actionSetNoDelay_result);
// }

void MochaVehicle::RaycastService(const carplanner_msgs::RaycastGoalConstPtr &goal)
{
    carplanner_msgs::RaycastResult actionRaycast_result;
    Eigen::Vector3d dIntersect;
    static int count=0;

    DLOG(INFO) << "Raycast service called.";

    bool success = RayCast(
        Eigen::Vector3d(goal->source.x, goal->source.y, goal->source.z), 
        Eigen::Vector3d(goal->ray.x, goal->ray.y, goal->ray.z), 
        dIntersect,
        goal->bidirectional,
        goal->index
        );

    if (success)
    {
        actionRaycast_result.intersect.x = dIntersect[0];
        actionRaycast_result.intersect.y = dIntersect[1];
        actionRaycast_result.intersect.z = dIntersect[2];

        m_actionRaycast_server.setSucceeded(actionRaycast_result);
    }
    else
    {
        m_actionRaycast_server.setAborted();
    }
    
}

bool MochaVehicle::SetDriveModeSvcCb(carplanner_msgs::SetDriveMode::Request &req, carplanner_msgs::SetDriveMode::Response &res)
{   
    DLOG(INFO) << "SetDriveMode service called.";

    SetDriveMode(req.world_id, req.mode);

    // if (!m_bPlanContinuously)
    //     replan();

    return true;
}

void MochaVehicle::SetDriveMode(uint nWorldId, uint mode)
{
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    // pWorld->m_pVehicle->setDriveMode((btWheelVehicle::DriveMode)mode);
}

// void MochaVehicle::CreateServerService(const carplanner_msgs::CreateServerGoalConstPtr &goal)
// {
//     actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> server(m_nh, "plan_car/apply_velocities/"+std::to_string(goal->id), boost::bind(&MochaVehicle::ApplyVelocitiesService, this, _1), true); 
//     servers_.push_back(&server);
//     m_actionCreateSimpleServer_server.setSucceeded(carplanner_msgs::CreateServerResult());
// }

void MochaVehicle::ApplyVelocitiesService(const carplanner_msgs::ApplyVelocitiesGoalConstPtr &goal)
{
    // ROS_INFO("Goal %d received at %.2fs", goal->world_id, ros::Time::now().toSec());

    carplanner_msgs::ApplyVelocitiesFeedback actionApplyVelocities_feedback;
    carplanner_msgs::ApplyVelocitiesResult actionApplyVelocities_result;

    // DLOG(INFO) << "ApplyVelocities called:" << 
    //   " world " << std::to_string(goal->world_id)
    //   // "\nstart " << std::to_string(VehicleState::fromROS(goal->initial_state))
    //   ;

    // global_time = 0;
    // double t0 = Tic();

    VehicleState state;
    state.fromROS(goal->initial_state);
    MotionSample sample;
    sample.fromROS(goal->initial_motion_sample);
    ApplyVelocities(
        state,
        sample,
        goal->world_id,
        goal->no_compensation,
        goal->no_delay);

    // ROS_INFO("Cumulative step sim time %f", global_time);
    // ROS_INFO("Total apply vel time %f", Toc(t0));
        
    actionApplyVelocities_result.motion_sample = sample.toROS();
    // actionApplyVelocities_result.last_state = sample.m_vStates.back().toROS();
    // actionApplyVelocities_result.world_id = goal->world_id;
    // m_actionApplyVelocities_server.setSucceeded(actionApplyVelocities_result);

    switch (goal->world_id)
    {
        case 0:
            m_actionApplyVelocities_server0.setSucceeded(actionApplyVelocities_result);
            break;
        case 1:
            m_actionApplyVelocities_server1.setSucceeded(actionApplyVelocities_result);
            break;
        case 2:
            m_actionApplyVelocities_server2.setSucceeded(actionApplyVelocities_result);
            break;
        case 3:
            m_actionApplyVelocities_server3.setSucceeded(actionApplyVelocities_result);
            break;
        case 4:
            m_actionApplyVelocities_server4.setSucceeded(actionApplyVelocities_result);
            break;
        case 5:
            m_actionApplyVelocities_server5.setSucceeded(actionApplyVelocities_result);
            break;
        case 6:
            m_actionApplyVelocities_server6.setSucceeded(actionApplyVelocities_result);
            break;
        case 7:
            m_actionApplyVelocities_server7.setSucceeded(actionApplyVelocities_result);
            break;
        case 8:
            m_actionApplyVelocities_server8.setSucceeded(actionApplyVelocities_result);
            break;
        case 9:
            m_actionApplyVelocities_server9.setSucceeded(actionApplyVelocities_result);
            break;
        default:
            break;
    }

    // ROS_INFO("Sending result %d at %.2fs", goal->world_id, ros::Time::now().toSec());

    // BulletWorldInstance* pWorld = GetWorldInstance(goal->world_id);
    // {
    //     std::string prtstr = "\nid\tidx\tdt\taccel\tphi\tx\ty\tz\ttheta\tv";
    //     for (uint i=0; i<actionApplyVelocities_result.motion_sample.commands.size(); i++)
    //     {
    //         VehicleState state; state.fromROS(actionApplyVelocities_result.motion_sample.states[i]);
    //         Eigen::Vector6d state_vec = state.ToXYZTCV();
    //         ControlCommand command; command.fromROS(actionApplyVelocities_result.motion_sample.commands[i]);
    //         prtstr += "\n" + std::to_string(goal->world_id) + "\t" 
    //             + std::to_string(i) + "\t";
    //         {
    //             std::string tempstr = std::to_string(command.m_dT);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+4);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(command.m_dForce);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+4);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(command.m_dPhi);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+4);
    //             prtstr += tempstr + "\t";
    //         }

    //         //     + std::to_string((state_vec[0]) + "\t" 
    //         //     + std::to_string((state_vec[1]) + "\t" 
    //         //     + std::to_string((state_vec[2]) + "\t" 
    //         //     + std::to_string((state_vec[3]) + "\t" 
    //         //     + std::to_string(command.m_dForce) + "\t" 
    //         //     + std::to_string(command.m_dPhi) ;
    //         {
    //             std::string tempstr = std::to_string(state_vec[0]);
    //             // std::cout << "*** a: " << tempstr << std::endl;
    //             // std::cout << "*** b: " << std::to_string(tempstr.find_first_of(".")) << std::endl;
    //             // std::cout << "*** c: " << tempstr.substr(0,tempstr.find_first_of(".")+2) << std::endl;
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+3);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(state_vec[1]);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+3);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(state_vec[2]);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+3);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(state_vec[3]);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+3);
    //             prtstr += tempstr + "\t";
    //         }
    //         {
    //             std::string tempstr = std::to_string(state_vec[5]);
    //             tempstr = tempstr.substr(0,tempstr.find_first_of(".")+3);
    //             prtstr += tempstr ;
    //         }
    //     }
    //     ROS_INFO(prtstr.c_str());
    // }
}

// void MochaVehicle::ApplyVelocitiesService(const carplanner_msgs::ApplyVelocitiesGoalConstPtr &goal)
// {
//     carplanner_msgs::ApplyVelocitiesGoalConstPtr new_goal(goal);
//     boost::thread av_thread(boost::bind(&MochaVehicle::ApplyVelocities, this, goal));
//     av_thread.join();
// }
// void MochaVehicle::ApplyVelocities(carplanner_msgs::ApplyVelocitiesGoalConstPtr& goal)
// {
//     carplanner_msgs::ApplyVelocitiesFeedback actionApplyVelocities_feedback;
//     carplanner_msgs::ApplyVelocitiesResult actionApplyVelocities_result;

//     DLOG(INFO) << "ApplyVelocities called:" << 
//       " world " << std::to_string(goal->world_id)
//       // "\nstart " << std::to_string(VehicleState::fromROS(goal->initial_state))
//       ;

//     VehicleState state;
//     state.fromROS(goal->initial_state);
//     MotionSample sample;
//     sample.fromROS(goal->initial_motion_sample);
//     ApplyVelocities(
//         state,
//         sample,
//         goal->world_id,
//         goal->no_compensation);
        
//     actionApplyVelocities_result.motion_sample = sample.toROS();
//     // actionApplyVelocities_result.last_state = sample.m_vStates.back().toROS();
//     m_actionApplyVelocities_server.setSucceeded(actionApplyVelocities_result);

//     DLOG(INFO) << "ApplyVelocities done:" << 
//         " world " << std::to_string(goal->world_id)
//         // << " poseOut " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x) 
//         //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.x)  
//         //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.y)
//         //     << " " << std::to_string(actionApplyVelocities_result.motion_sample.states.back()->pose.transform.translation.z) 
//         ;
// }

/////////////////////////

// bool MochaVehicle::ApplyAllVelocitiesFromClient(ApplyAllVelocitiesClient* client,
//                                     const std::vector<VehicleState>& startStates,
//                                     std::vector<MotionSample>& samples,
//                                     bool noCompensation /*= false*/,
//                                     bool noDelay /*=false*/)
// {
//     ROS_INFO("Applying all velocities.");

//     carplanner_msgs::ApplyAllVelocitiesGoal goal;
//     carplanner_msgs::ApplyAllVelocitiesResultConstPtr result;

//     if (!client)
//     {
//         ROS_WARN("No client provided, creating new one.");
//         client = new ApplyAllVelocitiesClient("vehicle/apply_all_velocities",true);
//         client->waitForServer();
//     }
    
//     assert(samples.size()==startStates.size());
//     for (uint i=0; i<samples.size(); i++)
//     {
//         // goal.initial_state = startState.toROS();
//         // goal.initial_motion_sample = sample.toROS();
//         // goal.world_id = nWorldId;
//         // goal.no_compensation = noCompensation;
//         // goal.no_delay = noDelay;
//     }

//     ROS_DBG("Sending goal.");
//     double t1 = Tic();

//     client->sendGoal(goal
//         // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
//         // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
//         // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
//         );

//     bool success;

//     float timeout(15.0);
//     bool finished_before_timeout = client->waitForResult(ros::Duration(timeout));

//     double t3 = Tic();
//     ROS_DBG("Got goal result, took %fs", t3-t1);

//     if (finished_before_timeout)
//     {
//         success = true;

//         // actionlib::SimpleClientGoalState state = client->getState();

//         result = client->getResult();
//         // sample.fromROS(result->motion_sample);
//     }
//     else
//     {
//         success = false;
//         ROS_ERROR("ApplyAllVelocities did not finish before the %fs timeout.", timeout);
//     }
    
//     return success;
// }

// void MochaVehicle::ApplyAllCommands(std::vector<VehicleState>& startStates,
//                                     std::vector<MotionSample>& samples,
//                                     bool noCompensation /*= false*/,
//                                     bool noDelay /*=false*/) 
// {
//     assert(startStates.size()==samples.size());
//     for (uint i=0; i<startStates.size(); i++)
//     {
//         ApplyVelocities(startStates[i],
//                         samples[i].m_vCommands,
//                         samples[i].m_vStates,
//                         0,
//                         samples[i].m_vCommands.size(),
//                         i,
//                         noCompensation,
//                         NULL,
//                         noDelay);
//     }
// }

// void MochaVehicle::ApplyCommands( VehicleState& startingState,
//                                     std::vector<ControlCommand>& vCommands,
//                                     std::vector<VehicleState>& vStatesOut,
//                                     const int iMotionStart,
//                                     const int iMotionEnd,
//                                     const int nWorldId,
//                                     const bool bNoCompensation /*= false (bUsingBestSolution)*/,
//                                     const CommandList *pPreviousCommands /*= NULL*/,
//                                     bool noDelay /*=false*/)
// {
//     double t0 = Tic();
//     ROS_INFO("Applying commands (%d)", nWorldId);
    


//     double t1 = Tic();
//     ROS_INFO("Done applying commands (%d), took %fs", nWorldId, t1-t0);
// }

///////////////////////

// bool MochaVehicle::ApplyAllVelocitiesFromClient(ApplyAllVelocitiesClient* client,
//                                     const std::vector<VehicleState>& startStates,
//                                     std::vector<MotionSample>& samples,
//                                     bool noCompensation /*= false*/,
//                                     bool noDelay /*=false*/)
// {
//     ROS_INFO("Applying all velocities.");

//     carplanner_msgs::ApplyAllVelocitiesGoal goal;
//     carplanner_msgs::ApplyAllVelocitiesResultConstPtr result;

//     if (!client)
//     {
//         ROS_WARN("No client provided, creating new one.");
//         client = new ApplyAllVelocitiesClient("vehicle/apply_all_velocities",true);
//         client->waitForServer();
//     }
    
//     assert(samples.size()==startStates.size());
//     for (uint i=0; i<samples.size(); i++)
//     {
//         // goal.initial_state = startState.toROS();
//         // goal.initial_motion_sample = sample.toROS();
//         // goal.world_id = nWorldId;
//         // goal.no_compensation = noCompensation;
//         // goal.no_delay = noDelay;
//     }

//     ROS_DBG("Sending goal.");
//     double t1 = Tic();

//     client->sendGoal(goal
//         // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
//         // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
//         // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
//         );

//     bool success;

//     float timeout(15.0);
//     bool finished_before_timeout = client->waitForResult(ros::Duration(timeout));

//     double t3 = Tic();
//     ROS_DBG("Got goal result, took %fs", t3-t1);

//     if (finished_before_timeout)
//     {
//         success = true;

//         // actionlib::SimpleClientGoalState state = client->getState();

//         result = client->getResult();
//         // sample.fromROS(result->motion_sample);
//     }
//     else
//     {
//         success = false;
//         ROS_ERROR("ApplyAllVelocities did not finish before the %fs timeout.", timeout);
//     }
    
//     return success;
// }

// void MochaVehicle::ApplyAllVelocities(std::vector<VehicleState>& startStates,
//                                     std::vector<MotionSample>& samples,
//                                     bool noCompensation /*= false*/,
//                                     bool noDelay /*=false*/) 
// {
//     assert(startStates.size()==samples.size());
//     for (uint i=0; i<startStates.size(); i++)
//     {
//         ApplyVelocities(startStates[i],
//                         samples[i].m_vCommands,
//                         samples[i].m_vStates,
//                         0,
//                         samples[i].m_vCommands.size(),
//                         i,
//                         noCompensation,
//                         NULL,
//                         noDelay);
//     }
// }


bool MochaVehicle::ApplyVelocitiesFromClient(ApplyVelocitiesClient* client,
                                    const VehicleState& startState,
                                    MotionSample& sample,
                                    int nWorldId /*= 0*/,
                                    bool noCompensation /*= false*/,
                                    bool noDelay /*=false*/)
{
    ROS_DBG("Starting AV client (%d).", nWorldId);

    carplanner_msgs::ApplyVelocitiesGoal goal;
    carplanner_msgs::ApplyVelocitiesResultConstPtr result;

    if (!client)
    {
        ROS_WARN("No client provided, creating new one.");
        client = new ApplyVelocitiesClient("vehicle/"+std::to_string(nWorldId)+"/apply_velocities",true);
        client->waitForServer();
    }
    
    goal.initial_state = startState.toROS();
    goal.initial_motion_sample = sample.toROS();
    goal.world_id = nWorldId;
    goal.no_compensation = noCompensation;
    goal.no_delay = noDelay;

    ROS_DBG("Sending AV goal (%d) with %d commands.", nWorldId, goal.initial_motion_sample.commands.size());
    double t1 = Tic();

    client->sendGoal(goal
        // , boost::bind(&MochaProblem::ApplyVelocitiesDoneCb, this, _1, _2)
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleActiveCallback()
        // , actionlib::SimpleActionClient<carplanner_msgs::ApplyVelocitiesAction>::SimpleFeedbackCallback()
        );

    bool success;

    float timeout(15.0);
    timeout = 0.1 * goal.initial_motion_sample.states.size();
    bool finished_before_timeout = client->waitForResult(ros::Duration(timeout));

    double t3 = Tic();
    ROS_DBG("Got goal result (%d), took %fs", nWorldId, t3-t1);

    if (finished_before_timeout)
    {
        success = true;

        // actionlib::SimpleClientGoalState state = client->getState();

        result = client->getResult();
        sample.fromROS(result->motion_sample);
    }
    else
    {
        success = false;
        ROS_ERROR("ApplyVelocities (%d) did not finish before the %fs timeout.", nWorldId, timeout);
    }
    
    return success;
}

void MochaVehicle::ApplyVelocities( VehicleState& startingState,
                                    std::vector<ControlCommand>& vCommands,
                                    std::vector<VehicleState>& vStatesOut,
                                    const int iMotionStart,
                                    const int iMotionEnd,
                                    const int nWorldId,
                                    const bool bNoCompensation /*= false (bUsingBestSolution)*/,
                                    const CommandList *pPreviousCommands /*= NULL*/,
                                    bool noDelay /*=false*/)
{
    double t0 = Tic();
    ROS_INFO("[Vehicle] applying velocities (%d)", nWorldId);

/*
    Eigen::Vector3d torques;
    Eigen::Vector4dAlignedVec vCoefs;
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);

    vStatesOut.clear();

    double dTime = 0;

    VehicleState currentState;
    SetState(nWorldId,startingState);
    GetVehicleState(nWorldId,currentState);
    // VehicleState* pCurrentState = &currentState; //this is necessary as we need to get a pointer to the current state for compensations
    //clear all the previous commands but chose between the member list or the one passed to the function
    SetCommandHistory(nWorldId, pPreviousCommands == NULL ? m_lPreviousCommands : *pPreviousCommands);
    // ResetCommandHistory(nWorldId);

    vStatesOut.resize(iMotionEnd-iMotionStart);

    // double t0 = Tic();

    ControlCommand command;
    for (int iMotion = iMotionStart; iMotion < iMotionEnd; iMotion++) {
        // ROS_INFO("Updating state %d world %d", iMotion, nWorldId);

        //update the vehicle state
        //approximation for this dt
        command = vCommands[iMotion];

        double dCorrectedCurvature;
        command.m_dPhi = GetSteeringAngle(command.m_dCurvature,
                                                dCorrectedCurvature,nWorldId,3.);

        //set the timestamp for this command
        command.m_dTime = dTime;
        dTime += command.m_dT;

        UpdateState(nWorldId, command, command.m_dT, m_bNoDelay);
        GetVehicleState(nWorldId, vStatesOut[iMotion-iMotionStart]);

        // vStatesOut[iMotion-iMotionStart] = UpdateState(nWorldId,command,command.m_dT,m_bNoDelay);
        
        vStatesOut[iMotion-iMotionStart].m_dCurvature = command.m_dCurvature;
        vStatesOut[iMotion-iMotionStart].m_dTime = dTime;
        // pCurrentState = &vStatesOut[iMotion-iMotionStart];
        // pCurrentState->m_dTime = dTime;

        vCommands[iMotion] = command;
    }

    // double t1 = Tic();
    // ROS_INFO("UpdateStates for world %d and %d states took %.2fs", nWorldId, iMotionEnd-iMotionStart, t1-t0);
*/

    Eigen::Vector3d torques;
    Eigen::Vector4dAlignedVec vCoefs;
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);

    vStatesOut.clear();

    double dTime = 0;

    VehicleState currentState;
    SetState(nWorldId,startingState);
    GetVehicleState(nWorldId,currentState);
    SetCommandHistory(nWorldId, pPreviousCommands == NULL ? m_lPreviousCommands : *pPreviousCommands);

    // currentState.m_dTwv.translation() += GetBasisVector(currentState.m_dTwv,2)*
    //   (pWorld->m_Parameters[CarParameters::SuspRestLength] +
    //   pWorld->m_Parameters[CarParameters::WheelRadius]+
    //   pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    // ROS_INFO("Applying Velocities (%d) from %s", nWorldId, currentState.toString(" | ").c_str());

    vStatesOut.resize(iMotionEnd-iMotionStart);

    ControlCommand command;
    for (int iMotion = iMotionStart; iMotion < iMotionEnd; iMotion++) {
        //update the vehicle state
        //approximation for this dt
        command = vCommands[iMotion];
        if(bNoCompensation == false ){
            //HACK: SampleAcceleration actually returns this as acceleration and
            //not force, so we have to change that here
            double totalAccel = command.m_dForce;
            // ROS_INFO("initial accel %f", totalAccel);

            //compensate for gravity/slope
            double aExtra = 0;
            double dCorrectedCurvature;
            command.m_dPhi = GetSteeringAngle(command.m_dCurvature,
                                                dCorrectedCurvature,nWorldId,pWorld->m_Parameters[CarParameters::SteeringCoef]);

            // ROS_INFO("K %f corrK %f phi %f accel %f", command.m_dCurvature, dCorrectedCurvature, command.m_dPhi, command.m_dForce);

            //if(dRatio < 1.0){
            // if(g_bSkidCompensationActive){
            //     //get the steering compensation
            //     std::pair<double,double> leftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nWorldId,0,command.m_dPhi,command.m_dT);
            //     std::pair<double,double> rightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nWorldId,1,command.m_dPhi,command.m_dT);
            //     double dRatio = std::max(fabs(leftWheel.second/leftWheel.first),fabs(rightWheel.second/rightWheel.first));

            //     for(int ii = 0 ; ii < 5 && dRatio < 1.0 ; ii++){
            //         command.m_dCurvature *=1.5;///= (dRatio);
            //         command.m_dPhi = m_pCarModel->GetSteeringAngle(command.m_dCurvature,
            //                                             dCorrectedCurvature,nWorldId,1.0);
            //         std::pair<double,double> newLeftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nWorldId,0,command.m_dPhi,command.m_dT);
            //         std::pair<double,double> newRightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(nWorldId,1,command.m_dPhi,command.m_dT);
            //         dRatio = std::max(fabs(newLeftWheel.second/leftWheel.first),fabs(newRightWheel.second/rightWheel.first));
            //     }
            // }



            // aExtra += GetGravityCompensation(nWorldId);
            aExtra += -(GetTotalGravityForce(GetWorldInstance(nWorldId))/GetWorldInstance(nWorldId)->m_Parameters[CarParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/;
            // ROS_INFO("adding gravity accel %f => %f", -(GetTotalGravityForce(GetWorldInstance(nWorldId))/GetWorldInstance(nWorldId)->m_Parameters[CarParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/, aExtra);
            //aExtra += GetSteeringCompensation(*pCurrentState,command.m_dPhi,command.m_dCurvature,nWorldId);
            aExtra += -GetTotalWheelFriction(nWorldId,command.m_dT)/GetWorldInstance(nWorldId)->m_Parameters[CarParameters::Mass];
            // ROS_INFO("adding friction accel %f => %f", -GetTotalWheelFriction(nWorldId,command.m_dT)/GetWorldInstance(nWorldId)->m_Parameters[CarParameters::Mass], aExtra);


            totalAccel += aExtra;

//            if(dRatio < 1.0){
//                totalAccel = 0;//(dRatio*dRatio*dRatio);
//            }

            //actually convert the accel (up to this point) to a force to be applied to the car
            command.m_dForce = totalAccel;//*GetWorldInstance(nWorldId)->m_Parameters[CarParameters::Mass];
            //here Pwm = (torque+slope*V)/Ts
            // command.m_dForce = sgn(command.m_dForce)* (fabs(command.m_dForce) + pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*pWorld->m_state.m_dV.norm())/pWorld->m_Parameters[CarParameters::StallTorqueCoef];
            // command.m_dForce += pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;

            //offset and coef are in 0-1 range, so multiplying by SERVO_RANGE is necessary
            // command.m_dPhi = command.m_dPhi*pWorld->m_Parameters[CarParameters::SteeringCoef] +
            //                               pWorld->m_Parameters[CarParameters::SteeringOffset];
            // command.m_dPhi = SERVO_RANGE*(command.m_dPhi);

            // ROS_INFO("corrphi %f corrAccel %f force %f", command.m_dPhi, totalAccel, command.m_dForce);


            //save the command changes to the command array -- this is so we can apply
            //the commands to the vehicle WITH compensation
            vCommands[iMotion] = command;
        }

        //set the timestamp for this command
        vCommands[iMotion].m_dTime = dTime;
        dTime += command.m_dT;
        // ROS_INFO("New command: f %.2f p %.2f dt %f t %.2f %.2f %.2f", command.m_dForce, command.m_dPhi, command.m_dT, command.m_dTorque[0], command.m_dTorque[1], command.m_dTorque[2]);
        UpdateState(nWorldId,command,command.m_dT,noDelay);
        GetVehicleState(nWorldId,vStatesOut[iMotion-iMotionStart]);
        // if (vStatesOut[iMotion-iMotionStart].m_bChassisInCollision)
        //     ROS_INFO("* CHASSIS IN COLLISION *******");
        // ROS_INFO("New state: p %.2f %.2f %.2f v %.2f %.2f %.2f f %.2f %.2f %.2f", 
        //     vStatesOut[iMotion-iMotionStart].m_dTwv.translation().x(), vStatesOut[iMotion-iMotionStart].m_dTwv.translation().y(), vStatesOut[iMotion-iMotionStart].m_dTwv.translation().z(), 
        //     vStatesOut[iMotion-iMotionStart].m_dV[0], vStatesOut[iMotion-iMotionStart].m_dV[1], vStatesOut[iMotion-iMotionStart].m_dV[2],
        //     (pWorld->m_pVehicle->getRigidBody()->getWorldTransform().getRotation() * pWorld->m_pVehicle->getRigidBody()->getTotalForce())[0], (pWorld->m_pVehicle->getRigidBody()->getWorldTransform().getRotation() * pWorld->m_pVehicle->getRigidBody()->getTotalForce())[1], (pWorld->m_pVehicle->getRigidBody()->getWorldTransform().getRotation() * pWorld->m_pVehicle->getRigidBody()->getTotalForce())[2]);
        // ROS_INFO("+ %d wheels %f %f %f", 3, pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getZ());
        vStatesOut[iMotion-iMotionStart].m_dCurvature = command.m_dCurvature;
        vStatesOut[iMotion-iMotionStart].m_dTime = dTime;
        // pCurrentState = &vStatesOut[iMotion-iMotionStart];
        // pCurrentState->m_dTime = dTime;

        // usleep(.1e6);
    }

    double t1 = Tic();
    ROS_INFO("[Vehicle] done applying velocities (%d), took %fs", nWorldId, t1-t0);
}

VehicleState MochaVehicle::ApplyVelocities( VehicleState& startState,
                                                      MotionSample& sample,
                                                      int nWorldId /*= 0*/,
                                                      bool noCompensation /*= false*/,
                                                      bool noDelay /*=false*/) {
    ApplyVelocities(startState,
                    sample.m_vCommands,
                    sample.m_vStates,
                    0,
                    sample.m_vCommands.size(),
                    nWorldId,
                    noCompensation,
                    NULL,
                    noDelay);
    return sample.m_vStates.back();
}

void MochaVehicle::pubTerrainMesh(uint nWorldId)
{

    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    time_t t0 = clock();

    _pubMesh(pWorld->m_pTerrainBody->getCollisionShape(), &(pWorld->m_pTerrainBody->getWorldTransform()), &m_terrainMeshPub);
    
    time_t t1 = clock();
    uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(pWorld->m_pTerrainBody->getCollisionShape())->getMeshInterface())->getNumTriangles();
    ROS_INFO_THROTTLE(0.5,"pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    
    // btCollisionObjectArray objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // for(uint i=0; i<objarr.size(); i++)
    // {
    //   if(objarr[i] != pWorld->m_pCarChassis)
    //   {
    //     time_t t0 = clock();

    //     _pubMesh(objarr[i]->getCollisionShape(), &(objarr[i]->getWorldTransform()), &m_terrainMeshPub);
    //     
    //     time_t t1 = clock();
    //     uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(objarr[i]->getCollisionShape())->getMeshInterface())->getNumTriangles();
    //     ROS_INFO("pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    //   }
    // }
}

/////////////////////////////////////////////////////////////////
void MochaVehicle::_pubMesh(btCollisionShape* collisionShape, ros::Publisher* pub)
{
    btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
    _pubMesh(collisionShape, parentTransform, pub);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_pubMesh(btCollisionShape* collisionShape, btTransform* parentTransform, ros::Publisher* pub)
{
     // ROS_INFO("about to pub mesh with %d faces", dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(collisionShape)->getMeshInterface())->getNumTriangles());

      time_t t0 = std::clock();

      mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
    //   btTransform*  = new btTransform(btQuaternion(1,0,0,0),btVector3(0,0,0)); // rot 180 x
    //   (*transformedToRosParent) *= (transformedToRosParent*parentTransform);
      convertCollisionShape2MeshMsg(collisionShape, parentTransform, &mesh);

      mesh_msgs::TriangleMeshStamped* mesh_stamped = new mesh_msgs::TriangleMeshStamped();
      mesh_stamped->mesh = *mesh;
      mesh_stamped->header.frame_id = m_config.map_frame;
      mesh_stamped->header.stamp = ros::Time::now();

      time_t t1 = std::clock();

      // ROS_INFO("pubbing mesh, %d faces, %d vertices, %.2f sec", mesh->triangles.size(), mesh->vertices.size(), std::difftime(t1,t0)/CLOCKS_PER_SEC); 
      pub->publish(*mesh_stamped);
      ros::spinOnce();
      // ros::Rate(10).sleep();
}

void MochaVehicle::meshCb(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg)
{
    float t0 = Tic();
    static tf::StampedTransform Twm;
    /* Temp removal
    try
    {
        m_tflistener.waitForTransform(m_config.map_frame, "infinitam", ros::Time::now(), ros::Duration(1.0));
        m_tflistener.lookupTransform(m_config.map_frame, "infinitam", ros::Time(0), Twm);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        usleep(10000);
        return;
    }
    */

    // time_t t0 = std::clock();
    // ros::Time t0 = ros::Time::now();

    // tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
    // Twm.setData(rot_180_x*Twm);

    float t1 = Tic();

    btCollisionShape* meshShape;// = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
    convertMeshMsg2CollisionShape(new mesh_msgs::TriangleMeshStamped(*mesh_msg), &meshShape);

    float t2 = Tic();

    for (uint i=0; i<GetNumWorlds(); i++)
    {
    //    appendMesh(i, meshShape, Twm);
        replaceMesh(i, meshShape, Twm);
    }

    float t3 = Tic();

    // time_t t1 = std::clock();
    // ros::Time t1 = ros::Time::now();
    ROS_INFO("[Vehicle] got mesh (%d faces, %d vertices), at %.4fs, tf lookup took %.4fs, conv took %.4fs, integ took %.4fs", 
      mesh_msg->mesh.triangles.size(), 
      mesh_msg->mesh.vertices.size(),  
      t0, 
      t1-t0,
      t2-t1,
      t3-t2 );

    DLOG(INFO) << "Mesh Callback: Address: " << mesh_msg.get();
}

void MochaVehicle::replaceMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
    // btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    // boost::unique_lock<boost::mutex> lock(*pWorld);
    pWorld->lock();

    if(pWorld->m_pTerrainBody != NULL)
    {
        pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
    }

    pWorld->m_pTerrainShape = meshShape;
    pWorld->m_pTerrainBody->setCollisionShape(pWorld->m_pTerrainShape);
    pWorld->m_pTerrainBody->setWorldTransform(btTransform(
      btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
      btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
    pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);
    
    // boost::unique_lock<boost::mutex> unlock(*pWorld);
    pWorld->unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::appendMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
    btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    // boost::unique_lock<boost::mutex> lock(*pWorld);
    pWorld->lock();
    
    uint max_num_coll_objs = 3;
    if( pWorld->m_pDynamicsWorld->getCollisionWorld()->getNumCollisionObjects() >= max_num_coll_objs )
    {
        // boost::unique_lock<boost::mutex> unlock(*pWorld);
        pWorld->unlock();
        replaceMesh(worldId, meshShape, Twm);
        return;
    }

    btTransform tr;
    tr.setIdentity();
    btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
    
    btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,meshShape,btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);
    body->setWorldTransform(btTransform(
      btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
      btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
    pWorld->m_pTerrainBody = body;
    pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);

    // boost::unique_lock<boost::mutex> unlock(*pWorld);
    pWorld->unlock();
}

void MochaVehicle::_pubTFs(uint nWorldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);

    // static tf::TransformListener tflistener;
    // try
    // {
    //     tflistener.waitForTransform("/map", "/world", ros::Time.now(), ros::Duration(1.0));
    //     tflistener.lookupTransform("/map", "world", ros::Time(0), local_transform);
    // }
    // catch (tf::TransformException& ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    // }

    static tf::TransformBroadcaster tfcaster;
    // tf::Transform rot_180_x( tf::Quaternion(1, 0, 0, 0), tf::Vector3(0, 0, 0) );
    // map -> base_link
    {
        btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform(); // nwu
        ros::Time now = ros::Time::now();

        tf::Transform tform_chassis(
            tf::Quaternion(btform_chassis.getRotation()[0], btform_chassis.getRotation()[1], btform_chassis.getRotation()[2], btform_chassis.getRotation()[3]),
            tf::Vector3(btform_chassis.getOrigin()[0], btform_chassis.getOrigin()[1], btform_chassis.getOrigin()[2]) );

        // tform_chassis = rot_180_x*tform_chassis*rot_180_x;

        tf::StampedTransform stform(tform_chassis, now, m_config.map_frame, m_config.base_link_frame+"/"+std::to_string(nWorldId));
        tfcaster.sendTransform(stform);
    }
    // // base_link -> front_right_wheel
    // {
    //     btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform();
    //     // btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheel(0)->getBody()->getWorldTransform(); 
    //     btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform;
    //     // Sophus::SE3d state = pWorld->m_state.m_vWheelStates[0];
    //     // btTransform btform_wheel_ws = btTransform(btQuaternion(state.unit_quaternion().x(),state.unit_quaternion().y(),state.unit_quaternion().z(),state.unit_quaternion().w()),btVector3(state.translation().x(),state.translation().y(),state.translation().z()));
    //     // ROS_INFO("Setting state %d w wheels %f %f %f", 0, btform_wheel_ws.getOrigin().getX(), btform_wheel_ws.getOrigin().getY(), btform_wheel_ws.getOrigin().getZ());
    //     ros::Time now = ros::Time::now();

    //     btTransform btform_wheel_cs = btform_chassis.inverse()*btform_wheel_ws; 
    //     tf::Transform tform_wheel_cs(
    //         tf::Quaternion(btform_wheel_cs.getRotation()[0], btform_wheel_cs.getRotation()[1], btform_wheel_cs.getRotation()[2], btform_wheel_cs.getRotation()[3]),
    //         tf::Vector3(btform_wheel_cs.getOrigin()[0], btform_wheel_cs.getOrigin()[1], btform_wheel_cs.getOrigin()[2]) );

    //     tf::StampedTransform stform(tform_wheel_cs, now, m_config.base_link_frame+"/"+std::to_string(nWorldId), pWorld->m_state.GetWheelFrame(0)+"/"+std::to_string(nWorldId));
    //     tfcaster.sendTransform(stform);
    // }
    // // base_link -> front_left_wheel
    // {
    //     btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform();
    //     // btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheel(1)->getBody()->getWorldTransform();  
    //     btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform;
    //     // Sophus::SE3d state = pWorld->m_state.m_vWheelStates[1];
    //     // btTransform btform_wheel_ws = btTransform(btQuaternion(state.unit_quaternion().x(),state.unit_quaternion().y(),state.unit_quaternion().z(),state.unit_quaternion().w()),btVector3(state.translation().x(),state.translation().y(),state.translation().z()));
    //     // ROS_INFO("Setting state %d w wheels %f %f %f", 1, btform_wheel_ws.getOrigin().getX(), btform_wheel_ws.getOrigin().getY(), btform_wheel_ws.getOrigin().getZ());
    //     ros::Time now = ros::Time::now();

    //     btTransform btform_wheel_cs = btform_chassis.inverse()*btform_wheel_ws; 
    //     tf::Transform tform_wheel_cs(
    //         tf::Quaternion(btform_wheel_cs.getRotation()[0], btform_wheel_cs.getRotation()[1], btform_wheel_cs.getRotation()[2], btform_wheel_cs.getRotation()[3]),
    //         tf::Vector3(btform_wheel_cs.getOrigin()[0], btform_wheel_cs.getOrigin()[1], btform_wheel_cs.getOrigin()[2]) );

    //     tf::StampedTransform stform(tform_wheel_cs, now, m_config.base_link_frame+"/"+std::to_string(nWorldId), pWorld->m_state.GetWheelFrame(1)+"/"+std::to_string(nWorldId));
    //     tfcaster.sendTransform(stform);
    // }
    // // base_link -> back_left_wheel
    // {
    //     btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform();
    //     // btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheel(2)->getBody()->getWorldTransform();  
    //     btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform;
    //     // Sophus::SE3d state = pWorld->m_state.m_vWheelStates[2];
    //     // btTransform btform_wheel_ws = btTransform(btQuaternion(state.unit_quaternion().x(),state.unit_quaternion().y(),state.unit_quaternion().z(),state.unit_quaternion().w()),btVector3(state.translation().x(),state.translation().y(),state.translation().z()));
    //     // ROS_INFO("Setting state %d w wheels %f %f %f", 2, btform_wheel_ws.getOrigin().getX(), btform_wheel_ws.getOrigin().getY(), btform_wheel_ws.getOrigin().getZ());
    //     ros::Time now = ros::Time::now();

    //     btTransform btform_wheel_cs = btform_chassis.inverse()*btform_wheel_ws; 
    //     tf::Transform tform_wheel_cs(
    //         tf::Quaternion(btform_wheel_cs.getRotation()[0], btform_wheel_cs.getRotation()[1], btform_wheel_cs.getRotation()[2], btform_wheel_cs.getRotation()[3]),
    //         tf::Vector3(btform_wheel_cs.getOrigin()[0], btform_wheel_cs.getOrigin()[1], btform_wheel_cs.getOrigin()[2]) );

    //     tf::StampedTransform stform(tform_wheel_cs, now, m_config.base_link_frame+"/"+std::to_string(nWorldId), pWorld->m_state.GetWheelFrame(2)+"/"+std::to_string(nWorldId));
    //     tfcaster.sendTransform(stform);
    // }
    // // base_link -> back_right_wheel
    // {
    //     btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform();
    //     // btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheel(3)->getBody()->getWorldTransform(); 
    //     btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform;
    //     // Sophus::SE3d state = pWorld->m_state.m_vWheelStates[3];
    //     // btTransform btform_wheel_ws = btTransform(btQuaternion(state.unit_quaternion().x(),state.unit_quaternion().y(),state.unit_quaternion().z(),state.unit_quaternion().w()),btVector3(state.translation().x(),state.translation().y(),state.translation().z())); 
    //     // ROS_INFO("Setting state %d w wheels %f %f %f", 3, btform_wheel_ws.getOrigin().getX(), btform_wheel_ws.getOrigin().getY(), btform_wheel_ws.getOrigin().getZ());
    //     ros::Time now = ros::Time::now();

    //     btTransform btform_wheel_cs = btform_chassis.inverse()*btform_wheel_ws; 
    //     tf::Transform tform_wheel_cs(
    //         tf::Quaternion(btform_wheel_cs.getRotation()[0], btform_wheel_cs.getRotation()[1], btform_wheel_cs.getRotation()[2], btform_wheel_cs.getRotation()[3]),
    //         tf::Vector3(btform_wheel_cs.getOrigin()[0], btform_wheel_cs.getOrigin()[1], btform_wheel_cs.getOrigin()[2]) );

    //     tf::StampedTransform stform(tform_wheel_cs, now, m_config.base_link_frame+"/"+std::to_string(nWorldId), pWorld->m_state.GetWheelFrame(3)+"/"+std::to_string(nWorldId));
    //     tfcaster.sendTransform(stform);
    // }

    for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    {
        btTransform btform_chassis = pWorld->m_pVehicle->getChassisWorldTransform();
        // btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheel(3)->getBody()->getWorldTransform(); 
        btTransform btform_wheel_ws = pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform;
        // Sophus::SE3d state = pWorld->m_state.m_vWheelStates[3];
        // btTransform btform_wheel_ws = btTransform(btQuaternion(state.unit_quaternion().x(),state.unit_quaternion().y(),state.unit_quaternion().z(),state.unit_quaternion().w()),btVector3(state.translation().x(),state.translation().y(),state.translation().z())); 
        // ROS_INFO("Setting state %d w wheels %f %f %f", 3, btform_wheel_ws.getOrigin().getX(), btform_wheel_ws.getOrigin().getY(), btform_wheel_ws.getOrigin().getZ());
        ros::Time now = ros::Time::now();

        btTransform btform_wheel_cs = btform_chassis.inverse()*btform_wheel_ws; 
        tf::Transform tform_wheel_cs(
            tf::Quaternion(btform_wheel_cs.getRotation()[0], btform_wheel_cs.getRotation()[1], btform_wheel_cs.getRotation()[2], btform_wheel_cs.getRotation()[3]),
            tf::Vector3(btform_wheel_cs.getOrigin()[0], btform_wheel_cs.getOrigin()[1], btform_wheel_cs.getOrigin()[2]) );

        tf::StampedTransform stform(tform_wheel_cs, now, m_config.base_link_frame+"/"+std::to_string(nWorldId), pWorld->m_state.GetWheelFrame(i)+"/"+std::to_string(nWorldId));
        tfcaster.sendTransform(stform);
    }

    ros::spinOnce();
}

////////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_PublisherFunc()
{
    while( ros::ok() )
    {
        _pubTFs(0);
        _pubVehicleMesh(0);

        // BulletWorldInstance* pWorld = GetWorldInstance(0);
        // _pubMesh(pWorld->m_pCarChassis->getCollisionShape(), &(pWorld->m_pCarChassis->getWorldTransform()), &m_chassisMeshPub);

        ros::Rate(100).sleep();
    }
}

void MochaVehicle::_pubVehicleMesh(uint nWorldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    // boost::mutex::scoped_lock lock(*pWorld);
    
    // btCompoundShape* vehicle_shape = new btCompoundShape();
    // btTransform chassisTranWSinv = pWorld->m_pVehicle->getRigidBody()->getWorldTransform().inverse();
    // // for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    // // {
    //     // btTransform wheelTranCS = chassisTranWSinv * pWorld->m_pVehicle->getWheel(i)->getBody()->getWorldTransform();
    //     // btTransform wheelTranCS = chassisTranWSinv * pWorld->m_pVehicle->getWheelTransformWS();
    //     // btTransform wheelTranCS = chassisTranWSinv * pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform;
    //     // vehicle_shape->addChildShape(wheelTranCS, pWorld->m_pVehicle->getWheel(i)->getBody()->getCollisionShape());
    //     // vehicle_shape->addChildShape(wheelTranCS, pWorld->m_pVehicle->getWheel(i)->getBody()->getWheelCollisionShape());
    //     // vehicle_shape->addChildShape(wheelTranCS, pWorld->m_pVehicle->getWheel(i)->getBody()->getCollisionShape());
    // // }
    // vehicle_shape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)), pWorld->m_pVehicle->getRigidBody()->getCollisionShape());
    // _pubMesh(vehicle_shape, new btTransform(chassisTranWSinv.inverse()), &m_chassisMeshPub);
    
    visualization_msgs::MarkerArray veh_marker_array_msg;
    // chassis
    // Eigen::Vector7d Tchassis = pWorld->m_state.ToXYZQuat();
    visualization_msgs::Marker chassis_marker_msg;
    chassis_marker_msg.header.frame_id = m_config.base_link_frame+"/"+std::to_string(nWorldId);
    chassis_marker_msg.header.stamp = ros::Time();
    // chassis_marker_msg.ns = "chassis";
    chassis_marker_msg.id = 0;
    chassis_marker_msg.type = visualization_msgs::Marker::CUBE;
    chassis_marker_msg.action = visualization_msgs::Marker::ADD;
    chassis_marker_msg.pose.position.x    = 0; //Tchassis[0];
    chassis_marker_msg.pose.position.y    = 0; //Tchassis[1];
    chassis_marker_msg.pose.position.z    = 0; //Tchassis[2];
    chassis_marker_msg.pose.orientation.x = 0; //Tchassis[3];
    chassis_marker_msg.pose.orientation.y = 0; //Tchassis[4];
    chassis_marker_msg.pose.orientation.z = 0; //Tchassis[5];
    chassis_marker_msg.pose.orientation.w = 1; //Tchassis[6];
    chassis_marker_msg.scale.x = (pWorld->m_Parameters[CarParameters::WheelBase]+pWorld->m_Parameters[CarParameters::WheelRadius])*2;
    chassis_marker_msg.scale.y = pWorld->m_Parameters[CarParameters::Width]*2;
    chassis_marker_msg.scale.z = pWorld->m_Parameters[CarParameters::Height]*2;
    chassis_marker_msg.color.a = 1.0; // Don't forget to set the alpha!
    chassis_marker_msg.color.r = (pWorld->m_state.IsChassisInCollision() ? 1.0 : 0.0);
    chassis_marker_msg.color.g = (pWorld->m_state.IsChassisInCollision() ? 0.0 : 1.0);
    chassis_marker_msg.color.b = 0.0;
    veh_marker_array_msg.markers.push_back(chassis_marker_msg);
    // wheels
    for (uint i=0; i<pWorld->m_state.GetNumWheels(); i++)
    {
        // Eigen::Vector7d Twheel = pWorld->m_state.GetWheelPoseCS(i);
        visualization_msgs::Marker wheel_marker_msg;
        wheel_marker_msg.header.frame_id = pWorld->m_state.GetWheelFrame(i)+"/"+std::to_string(nWorldId);
        wheel_marker_msg.header.stamp = ros::Time();
        // wheel_marker_msg.ns = "wheel";
        wheel_marker_msg.id = i+1;
        wheel_marker_msg.type = visualization_msgs::Marker::CYLINDER;
        wheel_marker_msg.action = visualization_msgs::Marker::ADD;
        wheel_marker_msg.pose.position.x    = 0; //Twheel[0];
        wheel_marker_msg.pose.position.y    = 0; //Twheel[1];
        wheel_marker_msg.pose.position.z    = 0; //Twheel[2];
        wheel_marker_msg.pose.orientation.x = 0.7; //Twheel[3];
        wheel_marker_msg.pose.orientation.y = 0; //Twheel[4];
        wheel_marker_msg.pose.orientation.z = 0; //Twheel[5];
        wheel_marker_msg.pose.orientation.w = 0.7; //Twheel[6];
        wheel_marker_msg.scale.x = pWorld->m_Parameters[CarParameters::WheelRadius]*2;
        wheel_marker_msg.scale.y = pWorld->m_Parameters[CarParameters::WheelRadius]*2;
        wheel_marker_msg.scale.z = pWorld->m_Parameters[CarParameters::WheelWidth];
        wheel_marker_msg.color.a = 1.0; // Don't forget to set the alpha!
        wheel_marker_msg.color.r = (pWorld->m_state.IsWheelInContact(i) ? 0.0 : 1.0);
        wheel_marker_msg.color.g = 0.0;
        wheel_marker_msg.color.b = (pWorld->m_state.IsWheelInContact(i) ? 1.0 : 0.0);
        veh_marker_array_msg.markers.push_back(wheel_marker_msg);
    }

    m_vehiclePub.publish(veh_marker_array_msg);
    ros::spinOnce();
}

void MochaVehicle::_pubPreviousCommands(uint nWorldId)
{
    // BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    // for (uint i=0; i<pWorld->m_lPreviousCommands.size(); i++)
    // {
    //     _pubCommand();
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////
// void MochaVehicle::_TerrainMeshPublisherFunc()
// {
//     while( ros::ok() )
//     {
//         pubTerrainMesh(0);
      
//         ros::Rate(100).sleep();
//     }
// }

void MochaVehicle::TerrainMeshPubLoopFunc(const ros::TimerEvent& event)
{
    pubTerrainMesh(0);
}

/////////////////////////////////////////////////////////////////////////////////////////////

void MochaVehicle::InitializeStatePublishers()
{
    m_vStatePublishers.clear();
    for (uint i=0; i<GetNumWorlds(); i++)
    {
        ros::Publisher* pub = new ros::Publisher(m_nh.advertise<carplanner_msgs::VehicleState>("vehicle/"+std::to_string(i)+"/state",1));
        m_vStatePublishers.push_back(pub);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////

void MochaVehicle::InitializeCommandSubscribers()
{
    m_vCommandSubscribers.clear();
    for (uint i=0; i<GetNumWorlds(); i++)
    {
        ros::Subscriber* sub = new ros::Subscriber(m_nh.subscribe<carplanner_msgs::Command>("vehicle/"+std::to_string(i)+"/command", 1, boost::bind(&MochaVehicle::commandCb, this, _1, i)));
        m_vCommandSubscribers.push_back(sub);
    }
}

void MochaVehicle::commandCb(const carplanner_msgs::Command::ConstPtr& msg, int worldId)
{
    ControlCommand command;
    carplanner_msgs::Command newmsg(*msg);
    command.fromROS(newmsg);
    UpdateState(worldId, command, command.m_dT, true, false);
}

//////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::StatePubLoopFunc(const ros::TimerEvent& event)
{
    for (uint i=0; i<GetNumWorlds(); i++)
    {
        pubState(i, m_vStatePublishers[i]);
    }
}

void MochaVehicle::pubState(uint nWorldId, ros::Publisher* pub)
{
    VehicleState stateOut;
    GetVehicleState(nWorldId, stateOut);
    stateOut.m_dTime = ros::Time::now().toSec();
    pubState(stateOut, pub);
}

void MochaVehicle::pubState(VehicleState& state, ros::Publisher* pub)
{
    carplanner_msgs::VehicleState state_msg = state.toROS();
    pub->publish(state_msg);
}

//////////////////////////////////////////////s///////////////////////////////////////////
void MochaVehicle::PushDelayedControl(int worldId, ControlCommand& delayedCommands)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    //lock the world to prevent changes
    boost::mutex::scoped_lock lock(*pWorld);
    pWorld->m_lPreviousCommands.push_front(delayedCommands);
    //add to the total command time
    pWorld->m_dTotalCommandTime += delayedCommands.m_dT;
    //int s = pWorld->m_lPreviousCommands.size();

  //if this time is over the maximum amount, pop them off at the back
    while(pWorld->m_dTotalCommandTime > MAX_CONTROL_DELAY &&
        (pWorld->m_dTotalCommandTime - pWorld->m_lPreviousCommands.back().m_dT) > MAX_CONTROL_DELAY)
    {
        pWorld->m_dTotalCommandTime -= pWorld->m_lPreviousCommands.back().m_dT;
        pWorld->m_lPreviousCommands.pop_back();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands)
{
    //clamp the time delay to be > 0
    timeDelay = std::max(timeDelay,0.0);

    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    //lock the world to prevent changes
    boost::mutex::scoped_lock lock(*pWorld);
    CommandList& previousCommands = pWorld->m_lPreviousCommands;
    //if there is control delay, get commands from a previous time
    double currentDelay = 0;
    CommandList::iterator it = previousCommands.begin();
    ControlCommand* pCurrentCommand = &(*it);
    //currentDelay = (*it).m_dT;

    int count = 0;
    if(previousCommands.size() > 1) {
        it++; //move to the first element
        for(; it != previousCommands.end() ; it++) {
            count++;
            if( currentDelay + (*it).m_dT >= timeDelay ) {

              //interpolate between the current and next commands
              double r2 = (timeDelay - currentDelay)/(*it).m_dT;
              double r1 = 1-r2;

              delayedCommands.m_dForce = r1*pCurrentCommand->m_dForce + r2*(*it).m_dForce;
              delayedCommands.m_dCurvature = r1*pCurrentCommand->m_dCurvature + r2*(*it).m_dCurvature;
              delayedCommands.m_dPhi = r1*pCurrentCommand->m_dPhi + r2*(*it).m_dPhi;
              delayedCommands.m_dTorque = r1*pCurrentCommand->m_dTorque + r2*(*it).m_dTorque;

              it++;
              return;
            }else {
                pCurrentCommand = &(*it);
                currentDelay += pCurrentCommand->m_dT;

                if(currentDelay == timeDelay) {
                    delayedCommands = *pCurrentCommand;
                    return;
                  }
            }
        }
    }else if(previousCommands.size() > 0){
        ROS_WARN("Command history list size < 2, using first command.");
        delayedCommands = previousCommands.front();
    }else{
        ROS_WARN("Command history list size == 0. Passing empty command.");
        delayedCommands.m_dForce = 0;//pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
        delayedCommands.m_dCurvature = 0;
        delayedCommands.m_dPhi = 0;//pWorld->m_Parameters[CarParameters::SteeringOffset]*SERVO_RANGE;
        delayedCommands.m_dTorque << 0,0,0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
double MochaVehicle::GetCorrectedSteering(double& dCurvature, int index)
{
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dCurvature*pWorld->m_Parameters[CarParameters::WheelBase]);
    phi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
      SoftMaximum(phi,-pWorld->m_Parameters[CarParameters::MaxSteering],50),50);
      dCurvature  = (tan(phi)/pWorld->m_Parameters[CarParameters::WheelBase]);
    return phi;
}

/////////////////////////////////////////////////////////////////////////////////////////
double MochaVehicle::GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/)
{
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dcurvature*pWorld->m_Parameters[CarParameters::WheelBase])*steeringCoef;
    //double phi = 1.0/atan((1-powi(pWorld->m_Parameters[CarParameters::WheelBase]/2,2)*powi(dcurvature,2))/(powi(dcurvature,2)*powi(pWorld->m_Parameters[CarParameters::WheelBase],2)));

    dCorrectedCurvature  = (tan(phi)/pWorld->m_Parameters[CarParameters::WheelBase]);
    //dCorrectedCurvature = 1/sqrt(powi(pWorld->m_Parameters[CarParameters::WheelBase]/2,2) + powi(pWorld->m_Parameters[CarParameters::WheelBase],2)*powi(1.0/tan(phi),2));
    return phi;
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::UpdateParameters(const std::vector<RegressionParameter>& vNewParams)
{
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
      UpdateParameters(vNewParams,ii);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index) {
    UpdateParameters(vNewParams,GetWorldInstance(index));
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,BulletWorldInstance* pWorld)
{
    boost::mutex::scoped_lock lock(*pWorld);
    //update the parameter map and learning parameter list with the new params
    for(size_t ii = 0; ii < vNewParams.size() ; ii++) {
        //pWorld->m_vLearningParameters[ii].m_dVal = vNewParams[ii].m_dVal;
        pWorld->m_Parameters[vNewParams[ii].m_nKey] = vNewParams[ii].m_dVal;
        //DLOG(INFO) << "Updating parameter with key " << vNewParams[ii].m_nKey << " to " << pWorld->m_Parameters[vNewParams[ii].m_nKey];
    }
    _InternalUpdateParameters(pWorld);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_InternalUpdateParameters(BulletWorldInstance* pWorld)
{
    //DLOG(INFO) << "updating parameters to " << pWorld->m_vParameters;

    // boost::mutex::scoped_lock lock(*pWorld);

    pWorld->m_bParametersChanged = true;

    //dyanmic friction is slightly less
    pWorld->m_pVehicle->SetDynamicFrictionCoefficient(pWorld->m_Parameters[CarParameters::DynamicFrictionCoef]);
    //set side friction (for drifting)
    pWorld->m_pVehicle->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[CarParameters::StaticSideFrictionCoef]);
    pWorld->m_pVehicle->SetSlipCoefficient(pWorld->m_Parameters[CarParameters::SlipCoefficient]);
    pWorld->m_pVehicle->SetMagicFormulaCoefficients(pWorld->m_Parameters[CarParameters::MagicFormula_B],
      pWorld->m_Parameters[CarParameters::MagicFormula_C],
      pWorld->m_Parameters[CarParameters::MagicFormula_E]);

    //set the mass and wheelbase of the car
    //pWorld->m_Parameters[CarParameters::WheelBase] = pWorld->m_vParameters[eWheelBase];
    //pWorld->m_Parameters.m_dMass = pWorld->m_vParameters[eMass];

    // btCollisionObjectArray objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects before remove\n", objarr.size());

    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pVehicle->getRigidBody());


    // objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects after remove\n", objarr.size());

    //change rigid body dimensions
    btBoxShape *pBoxShape =  (btBoxShape *)pWorld->m_pVehicle->getRigidBody()->getCollisionShape();
    pBoxShape->setImplicitShapeDimensions(btVector3(pWorld->m_Parameters[CarParameters::WheelBase],pWorld->m_Parameters[CarParameters::Width],pWorld->m_Parameters[CarParameters::Height]));
    //calculate new inertia
    btVector3 localInertia(0,0,0);
    pBoxShape->calculateLocalInertia(pWorld->m_Parameters[CarParameters::Mass],localInertia);
    pWorld->m_pVehicle->getRigidBody()->setMassProps(pWorld->m_Parameters[CarParameters::Mass],localInertia);
    pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pVehicle->getRigidBody(),COL_CAR,COL_NOTHING);

    // objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // printf("%d collision objects after add\n", objarr.size());

    //change the position of the wheels
    pWorld->m_pVehicle->getWheelInfo(0).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(1).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(2).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[CarParameters::WheelBase]/2;
    pWorld->m_pVehicle->getWheelInfo(3).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[CarParameters::WheelBase]/2;

    for (size_t i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    {
        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
        // ROS_INFO("-- %d wheels %f %f %f", i, pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getZ());
        pWorld->m_pVehicle->updateWheelTransformsWS(wheel);
        pWorld->m_pVehicle->updateWheelTransform(i);
        // pWorld->m_pVehicle->updateWheelTransform(i,true);
        // ROS_INFO("-+ %d wheels %f %f %f", i, pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOrigin().getZ());
        wheel.m_suspensionRestLength1 = pWorld->m_Parameters[CarParameters::SuspRestLength];
        wheel.m_suspensionStiffness = pWorld->m_Parameters[CarParameters::Stiffness];
        wheel.m_wheelsDampingCompression = pWorld->m_Parameters[CarParameters::CompDamping];
        wheel.m_wheelsDampingRelaxation = pWorld->m_Parameters[CarParameters::ExpDamping];
    }

    pWorld->m_pVehicle->updateSuspension();

}

/////////////////////////////////////////////////////////////////////////////////////////
double MochaVehicle::GetTotalWheelFriction(int worldId, double dt)
  {
    double totalForce = 0;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
      totalForce += _CalculateWheelFriction((ii),pWorld,dt);
    }
    return totalForce;
}

/////////////////////////////////////////////////////////////////////////////////////////
// std::pair<double,double> MochaVehicle::GetSteeringRequiredAndMaxForce(const int nWorldId, const int nWheelId, const double dPhi, const double dt)
// {
    // BulletWorldInstance* pWorld = GetWorldInstance(nWorldId);
    // return pWorld->m_pVehicle->GetSteeringRequiredAndMaxForce(nWheelId,dPhi,dt);
// }

/////////////////////////////////////////////////////////////////////////////////////////
double MochaVehicle::_CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt)
{
    bool bDynamic;
    double maxImpulse = pInstance->m_pVehicle->CalculateMaxFrictionImpulse(wheelNum,dt,bDynamic);
    return maxImpulse / dt;
}

/////////////////////////////////////////////////////////////////////////////////////////
double MochaVehicle::GetTotalGravityForce(BulletWorldInstance* pWorld)
{
    btVector3 gravityForce(0,0,-9.8*pWorld->m_Parameters[CarParameters::Mass]);
    for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(ii);
        if(wheel.m_raycastInfo.m_isInContact)
        {
            gravityForce -= wheel.m_raycastInfo.m_contactNormalWS*wheel.m_wheelsSuspensionForce;
        }
    }
    //now get the component in the direction of the vehicle
    const btTransform& chassisTrans = pWorld->m_pVehicle->getChassisWorldTransform();
    btVector3 carFwd (
      chassisTrans.getBasis()[0][CAR_FORWARD_AXIS],
      chassisTrans.getBasis()[1][CAR_FORWARD_AXIS],
      chassisTrans.getBasis()[2][CAR_FORWARD_AXIS]);
    double force = gravityForce.dot(carFwd);
    return force;
}

/////////////////////////////////////////////////////////////////////////////////////////
// void MochaVehicle::UpdateState(  const int& worldId,
//   const ControlCommand command,
//   const double forceDt /*= -1*/,
//   const bool bNoDelay /* = false */,
//   const bool bNoUpdate /* = false */)
// {
//     ControlCommand delayedCommands;
//     BulletWorldInstance* pWorld = GetWorldInstance(worldId);

//     if(pWorld->m_dTime == -1 && forceDt == -1 )   {
//         pWorld->m_dTime = Tic();
//         return;
//     }

//     //calculate the time since the last iteration
//     double dT = Toc( pWorld->m_dTime );
//     pWorld->m_dTime = Tic();

//     if( forceDt != -1 ){
//         dT = forceDt;
//     }

//     delayedCommands = command;
//     delayedCommands.m_dT = dT;
//     if(bNoDelay == false){
//         PushDelayedControl(worldId,delayedCommands);
//         //get the delayed command
//         _GetDelayedControl(worldId, pWorld->m_Parameters[CarParameters::ControlDelay],delayedCommands);
//     }
//     //get the delayed commands for execution and remove the offsets
//     double dCorrectedForce = delayedCommands.m_dForce- pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
//     double dCorrectedPhi = delayedCommands.m_dPhi-pWorld->m_Parameters[CarParameters::SteeringOffset]*SERVO_RANGE;

//     //D.C. motor equations:
//     //torque = Pwm*Ts - slope*V
//     //TODO: make this velocity in the direction of travel
//     const double stallTorque = dCorrectedForce*pWorld->m_Parameters[CarParameters::StallTorqueCoef];
//     dCorrectedForce = sgn(stallTorque)*std::max(0.0,fabs(stallTorque) -
//     pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*fabs(pWorld->m_state.m_dV.norm()));

//     //now apply the offset and scale values to the force and steering commands
//     dCorrectedPhi = dCorrectedPhi/(pWorld->m_Parameters[CarParameters::SteeringCoef]*SERVO_RANGE);

//     //clamp the steering
//     dCorrectedPhi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
//       SoftMaximum(dCorrectedPhi,-pWorld->m_Parameters[CarParameters::MaxSteering],10),10);

//     //steering needs to be flipped due to the way RayCastVehicle works
//     dCorrectedPhi *= -1;

//     //rate-limit the steering
//     double dCurrentSteering = pWorld->m_pVehicle->GetAckermanSteering();
//     double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
//     //clamp the rate
//     dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[CarParameters::MaxSteeringRate]);
//     //apply the steering
//     dCorrectedPhi = dCurrentSteering+dRate*dT;

//     double wheelForce = dCorrectedForce/2;

//     int wheelIndex = 2;
//     pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);
//     wheelIndex = 3;
//     pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);

//     wheelIndex = 0;
//     //set the steering value
//     pWorld->m_pVehicle->SetAckermanSteering(dCorrectedPhi);

//     // Simulation mode -> this causes the car to move on the screen
//     // Experiment mode + SIL -> this causes the car to go +z forever and spin on it's y-axis (through the length of the car)
//     // if bNoUpdate is true, then car does not move in both modes
//     // printf("OK 0\n");
//     if (pWorld->m_pDynamicsWorld && bNoUpdate==false)
//     {
//         // boost::mutex::scoped_lock lock(*pWorld);
//         Eigen::Vector3d T_w = pWorld->m_state.m_dTwv.so3()*command.m_dTorque;
//         btVector3 bTorques( T_w[0], T_w[1], T_w[2] );
//         pWorld->m_pVehicle->getRigidBody()->applyTorque( bTorques );
//         //DLOG(INFO) << "Sending torque vector " << T_w.transpose() << " to car.";
//         // printf("OK 1\n");
//         pWorld->m_pDynamicsWorld->stepSimulation(dT,1,dT);
//     }
//     // printf("OK 2\n");

//     // update internal variables with produce of stepSim
//     //do this in a critical section
//     {
//     boost::mutex::scoped_lock lock(*pWorld);
//     //get chassis data from bullet
//     Eigen::Matrix4d Twv;
//     pWorld->m_pVehicle->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
//     pWorld->m_state.m_dTwv = Sophus::SE3d(Twv);

//     if(pWorld->m_state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
//       pWorld->m_state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
//       pWorld->m_state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
//     }

//     for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
//       //m_pVehicle->updateWheelTransform(ii,true);
//       pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform.getOpenGLMatrix(Twv.data());
//       pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
//       pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact;
//     }

//     //get the velocity
//     pWorld->m_state.m_dV << pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[2];
//     pWorld->m_state.m_dW << pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[2];

//     //set the steering
//     pWorld->m_state.m_dSteering = pWorld->m_pVehicle->GetAckermanSteering();
//     }

// }

void MochaVehicle::UpdateState(  const int& worldId,
  const ControlCommand command,
  const double forceDt /*= -1*/,
  const bool bNoDelay /* = false */,
  const bool bNoUpdate /* = false */)
{
    ControlCommand delayedCommands;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    if(pWorld->m_dTime == -1 && forceDt == -1 )   {
        pWorld->m_dTime = Tic();
        return;
    }

    //calculate the time since the last iteration
    double dT = Toc( pWorld->m_dTime );
    pWorld->m_dTime = Tic();

    if( forceDt != -1 ){
        dT = forceDt;
    }

    delayedCommands = command;
    delayedCommands.m_dT = dT;
    if(bNoDelay == false){
        PushDelayedControl(worldId,delayedCommands);
        //get the delayed command
        _GetDelayedControl(worldId, pWorld->m_Parameters[CarParameters::ControlDelay],delayedCommands);
    }

    // ROS_INFO("New command: f %.2f p %.2f dt %.2f t %.2f %.2f %.2f", delayedCommands.m_dForce, delayedCommands.m_dPhi, delayedCommands.m_dT, delayedCommands.m_dTorque[0], delayedCommands.m_dTorque[1], delayedCommands.m_dTorque[2]);

    ///////////////////////////////////////////////////////////////////////
/*
    //get the delayed commands for execution and remove the offsets
    // double dCorrectedForce = delayedCommands.m_dForce- pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
    // double dCorrectedPhi = delayedCommands.m_dPhi-pWorld->m_Parameters[CarParameters::SteeringOffset]*SERVO_RANGE;

    // //D.C. motor equations:
    // //torque = Pwm*Ts - slope*V
    // //TODO: make this velocity in the direction of travel
    // const double stallTorque = dCorrectedForce*pWorld->m_Parameters[CarParameters::StallTorqueCoef];
    // dCorrectedForce = sgn(stallTorque)*std::max(0.0,fabs(stallTorque) -
    // pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*fabs(pWorld->m_state.m_dV.norm()));

    // //now apply the offset and scale values to the force and steering commands
    // dCorrectedPhi = dCorrectedPhi/(pWorld->m_Parameters[CarParameters::SteeringCoef]*SERVO_RANGE);

    // //clamp the steering
    // dCorrectedPhi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
    //   SoftMaximum(dCorrectedPhi,-pWorld->m_Parameters[CarParameters::MaxSteering],10),10);

    // //steering needs to be flipped due to the way RayCastVehicle works
    // dCorrectedPhi *= -1;

    // //rate-limit the steering
    // double dCurrentSteering = pWorld->m_pVehicle->getEnabledSteeringAngle();
    // double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
    // //clamp the rate
    // dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[CarParameters::MaxSteeringRate]);
    // //apply the steering
    // dCorrectedPhi = dCurrentSteering+dRate*dT;

    // double wheelForce = dCorrectedForce/2;

    // pWorld->m_pVehicle->setEnabledAngularAcceleration(delayedCommands.m_dForce, delayedCommands.m_dT);
    // pWorld->m_pVehicle->setEnabledLinearVelocity(1);

    // pWorld->m_pVehicle->setEnabledSteeringAngle(delayedCommands.m_dPhi);
    // pWorld->m_pVehicle->setEnabledYawVelocity(delayedCommands.m_dCurvature);

    // btScalar wheelBase = pWorld->m_pVehicle->getWheelBase();
    btScalar wheelBase = 2*fabs( pWorld->m_pVehicle->getWheelInfo(0).m_chassisConnectionPointCS[pWorld->m_pVehicle->getForwardAxis()] );
    btScalar steering = delayedCommands.m_dPhi;
    btScalar currLinVel = pWorld->m_pVehicle->getCurrentSpeedKmHour()/3.6;
    // btScalar currLinVel = forwVelCS;
    btScalar yawVel = tan(steering)*currLinVel/wheelBase;

    btScalar forwVel = pWorld->m_pVehicle->getCurrentSpeedKmHour()/3.6+delayedCommands.m_dForce*delayedCommands.m_dT;

    // pWorld->m_pVehicle->setForwardAndYawVelocity(forwVel, yawVel);
    pWorld->m_pVehicle->SetAckermanSteering(delayedCommands.m_dPhi);

    // btTransform chassisTranWS = pWorld->m_pVehicle->getChassisWorldTransform();
    // for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    // {
    //     btWheel* wheel = pWorld->m_pVehicle->getWheel(i);

    //     // set wheel steering position instantaneously
    //     btTransform wheelTranWS = wheel->getBody()->getWorldTransform();
    //     btTransform wheelTranCS = chassisTranWS.inverse() * wheelTranWS;
    //     // ROS_INFO("wheel %d orig basis %f %f %f %f", i, wheelTranCS.getRotation()[0], wheelTranCS.getRotation()[1], wheelTranCS.getRotation()[2], wheelTranCS.getRotation()[3]);
    //     btTransform wheelTranCS_yaw(btQuaternion(0,0,0,1), wheelTranCS.getOrigin());
    //     wheelTranCS_yaw.getBasis().setEulerYPR(wheel->getSteeringAngle(),0,0);
    //     // ROS_INFO("yawed basis %f %f %f %f", i, wheelTranCS_yaw.getRotation()[0], wheelTranCS_yaw.getRotation()[1], wheelTranCS_yaw.getRotation()[2], wheelTranCS_yaw.getRotation()[3]);
    //     // wheelTranCS = wheelTranCS_yaw * btTransform(wheelTranCS.getBasis(),btVector3(0,0,0));
    //     wheelTranWS = chassisTranWS * wheelTranCS_yaw;
	// 	wheel->getBody()->setCenterOfMassTransform(wheelTranWS);
    // }

    // Simulation mode -> this causes the car to move on the screen
    // Experiment mode + SIL -> this causes the car to go +z forever and spin on it's y-axis (through the length of the car)
    // if bNoUpdate is true, then car does not move in both modes
    // printf("OK 0\n");
    if (pWorld->m_pDynamicsWorld && bNoUpdate==false)
    {
        // pWorld->m_pVehicle->setEnabledTorqueForce(command.m_dTorque);
        // pWorld->m_pVehicle->updateConstraints();
        // double t0 = Tic();
        pWorld->m_pDynamicsWorld->stepSimulation(dT,1,dT);
        // double t1 = Tic();
        // global_time += t1-t0;
        // ROS_INFO_THROTTLE(.2,"StepSim of dt %.2fs took %.2fs", dT, t1-t0);
    }
    // printf("OK 2\n");

    // update internal variables with produce of stepSim
    //do this in a critical section
    {
    boost::mutex::scoped_lock lock(*pWorld);
    pWorld->m_state.m_dTime = command.m_dTime+dT;

    //get chassis data from bullet
    Eigen::Matrix4d Twv;
    pWorld->m_pVehicle->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
    pWorld->m_state.m_dTwv = Sophus::SE3d(Twv);

    std::vector<CollisionEvent> collisions;
    getCollisions(pWorld->m_pDynamicsWorld, collisions);

    if(pWorld->m_state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
        pWorld->m_state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
        pWorld->m_state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
    }

    // for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
    //     //m_pVehicle->updateWheelTransform(ii,true);
    //     pWorld->m_pVehicle->getWheelTransformWS(ii).getOpenGLMatrix(Twv.data());
    //     pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
    // //     pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheel(ii)->isInContact(pWorld->m_pVehicleRayCaster);
    
    //     bool isInContact = false;
    //     for (uint i=0; i<collisions.size(); i++)
    //     {
    //         if (collisions[i].bodyA==pWorld->m_pVehicle->getWheel(ii)->getBody() || collisions[i].bodyB==pWorld->m_pVehicle->getWheel(ii)->getBody()) 
    //         {
    //             isInContact = true;
    //         }
    //     }
    //     pWorld->m_state.m_vWheelContacts[ii] = isInContact;
    // }
    pWorld->m_pVehicle->updateWheelContacts();
    // std::string prt;
    // for(size_t ii = 0; ii < pWorld->m_state.m_vWheelStates.size() ; ii++) {
    //     prt += (pWorld->m_state.m_vWheelContacts[ii] ? "true " : "false ");
    // }
    // std::cout << prt << std::endl;

    //get the velocity
    // pWorld->m_state.m_dV << pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[2];
    btVector3 linVel = pWorld->m_pVehicle->getChassisLinearVelocityWS();
    pWorld->m_state.m_dV << linVel[0], linVel[1], linVel[2];
    // pWorld->m_state.m_dW << pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[2];
    btVector3 angVel = pWorld->m_pVehicle->getChassisAngularVelocityWS();
    pWorld->m_state.m_dW << angVel[0], angVel[1], angVel[2];

    //set the steering
    pWorld->m_state.m_dSteering = pWorld->m_pVehicle->getSteeringAngle();
    }
*/
    ////////////////////////////////////////////////////////////////////////////


    //get the delayed commands for execution and remove the offsets
    double dCorrectedForce = delayedCommands.m_dForce;// - pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;
    double dCorrectedPhi = delayedCommands.m_dPhi;//-pWorld->m_Parameters[CarParameters::SteeringOffset];//*SERVO_RANGE;

    
    // til now accel, now make force
    dCorrectedForce *= pWorld->m_Parameters[CarParameters::Mass];
    // here Pwm = (torque+slope*V)/Ts
    dCorrectedForce = sgn(dCorrectedForce)* (fabs(dCorrectedForce) + pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*pWorld->m_state.m_dV.norm())/pWorld->m_Parameters[CarParameters::StallTorqueCoef];
    //D.C. motor equations:
    //torque = Pwm*Ts - slope*V
    //TODO: make this velocity in the direction of travel
    const double stallTorque = dCorrectedForce*pWorld->m_Parameters[CarParameters::StallTorqueCoef];
    dCorrectedForce = sgn(stallTorque)*std::max(0.0,fabs(stallTorque) -
        pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*fabs(pWorld->m_state.m_dV.norm()));

    //now apply the offset and scale values to the force and steering commands
    // dCorrectedPhi = dCorrectedPhi/(pWorld->m_Parameters[CarParameters::SteeringCoef]);//*SERVO_RANGE);

    //clamp the steering
    dCorrectedPhi = SoftMinimum(pWorld->m_Parameters[CarParameters::MaxSteering],
      SoftMaximum(dCorrectedPhi,-pWorld->m_Parameters[CarParameters::MaxSteering],10),10);

    //steering needs to be flipped due to the way RayCastVehicle works
    // dCorrectedPhi *= -1;
    // dCorrectedForce *= -1;

    //rate-limit the steering
    double dCurrentSteering = pWorld->m_pVehicle->GetAckermanSteering();
    double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
    //clamp the rate
    dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[CarParameters::MaxSteeringRate]);
    //apply the steering
    dCorrectedPhi = dCurrentSteering+dRate*dT;

    double wheelForce = dCorrectedForce/2;

    int wheelIndex = 2;
    pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);
    wheelIndex = 3;
    pWorld->m_pVehicle->applyEngineForce(wheelForce,wheelIndex);

    wheelIndex = 0;
    //set the steering value
    pWorld->m_pVehicle->SetAckermanSteering(dCorrectedPhi);

    // ROS_INFO("sending force %f phi %f", wheelForce, dCorrectedPhi);

    // Simulation mode -> this causes the car to move on the screen
    // Experiment mode + SIL -> this causes the car to go +z forever and spin on it's y-axis (through the length of the car)
    // if bNoUpdate is true, then car does not move in both modes
    // printf("OK 0\n");
    if (pWorld->m_pDynamicsWorld && bNoUpdate==false)
    {
      // boost::mutex::scoped_lock lock(*pWorld);
      Eigen::Vector3d T_w = pWorld->m_state.m_dTwv.so3()*command.m_dTorque;
      btVector3 bTorques( T_w[0], T_w[1], T_w[2] );
      pWorld->m_pVehicle->getRigidBody()->applyTorque( bTorques );
    //   ROS_INFO("Sending torque vector %f %f %f", T_w[0], T_w[1], T_w[2]);
      // printf("OK 1\n");
    //   double t0 = Tic();
      pWorld->m_pDynamicsWorld->stepSimulation(dT,1,dT);
    //   double t1 = Tic();
    //   ROS_INFO("StepSim (%fs) took %fs", dT, t1-t0);
    }
    // printf("OK 2\n");

    // update internal variables with produce of stepSim
    //do this in a critical section
    // {
    //   boost::mutex::scoped_lock lock(*pWorld);
    //   //get chassis data from bullet
    //   Eigen::Matrix4d Twv;
    //   pWorld->m_pVehicle->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
    //   pWorld->m_state.m_dTwv = Sophus::SE3d(Twv);

    //   pWorld->m_state.m_bChassisInCollision = pWorld->m_pVehicle->m_bChassisInCollision;
    // //   if (pWorld->m_state.m_bChassisInCollision)
    // //     ROS_INFO("* CHASSIS IN COLLISION ****");

    //   if(pWorld->m_state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
    //     pWorld->m_state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
    //     pWorld->m_state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
    //   }

    //   for(size_t ii = 0; ii < pWorld->m_pVehicle->getNumWheels() ; ii++) {
    //     //m_pVehicle->updateWheelTransform(ii,true);
    //     pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform.getOpenGLMatrix(Twv.data());
    //     pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
    //     pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact;
    //   }

    //   //get the velocity
    //   pWorld->m_state.m_dV << pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getLinearVelocity()[2];
    //   pWorld->m_state.m_dW << pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[0], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[1], pWorld->m_pVehicle->getRigidBody()->getAngularVelocity()[2];

    //   //set the steering
    //   pWorld->m_state.m_dSteering = pWorld->m_pVehicle->GetAckermanSteering();
    // }
    SyncStateToVehicle(pWorld);

    // if (worldId==9)
    // {
    //     _pubTFs(worldId);
    //     _pubVehicleMesh(nWorldId);
    //     // _pubPreviousCommands(nWorldId);
    // }

    // btVector3 force = pWorld->m_pVehicle->getRigidBody()->getTotalForce();
    // btVector3 force = pWorld->m_pVehicle->getRigidBody()->getWorldTransform().getRotation() * pWorld->m_pVehicle->getRigidBody()->getTotalForce();
    // ROS_INFO("New state: p %.2f %.2f %.2f v %.2f %.2f %.2f %.2f %.2f %.2f c %s %s %s %s", 
    //     pWorld->m_state.m_dTwv.translation().x(), pWorld->m_state.m_dTwv.translation().y(), pWorld->m_state.m_dTwv.translation().z(), 
    //     pWorld->m_state.m_dV[0], pWorld->m_state.m_dV[1], pWorld->m_state.m_dV[2],
    //     pWorld->m_state.m_dW[0], pWorld->m_state.m_dW[1], pWorld->m_state.m_dW[2],
    //     (pWorld->m_state.m_vWheelContacts[0]?"1":"0"), (pWorld->m_state.m_vWheelContacts[1]?"1":"0"), (pWorld->m_state.m_vWheelContacts[2]?"1":"0"), (pWorld->m_state.m_vWheelContacts[3]?"1":"0"));
        // force[0], force[1], force[2]
        // pWorld->m_state.m_dSteering
}

void MochaVehicle::SyncAllStatesToVehicles()
{
    for (uint i=0; i<GetNumWorlds(); i++)
    {
        SyncStateToVehicle(i);
    }
}

void MochaVehicle::SyncStateToVehicle(int worldId)
{
    BulletWorldInstance* world = GetWorldInstance(worldId);
    SyncStateToVehicle(world);
}

void MochaVehicle::SyncStateToVehicle(BulletWorldInstance* world)
{
    boost::mutex::scoped_lock lock(*world);
    //get chassis data from bullet
    Eigen::Matrix4d Twv;
    world->m_pVehicle->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
    world->m_state.m_dTwv = Sophus::SE3d(Twv);

    world->m_state.m_bChassisInCollision = world->m_pVehicle->m_bChassisInCollision;
//   if (world->m_state.m_bChassisInCollision)
//     ROS_INFO("* CHASSIS IN COLLISION ****");

    if(world->m_state.m_vWheelStates.size() != world->m_pVehicle->getNumWheels()) {
    world->m_state.m_vWheelStates.resize(world->m_pVehicle->getNumWheels());
    world->m_state.m_vWheelContacts.resize(world->m_pVehicle->getNumWheels());
    }

    for(size_t ii = 0; ii < world->m_pVehicle->getNumWheels() ; ii++) {
    //m_pVehicle->updateWheelTransform(ii,true);
    world->m_pVehicle->getWheelInfo(ii).m_worldTransform.getOpenGLMatrix(Twv.data());
    world->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
    world->m_state.m_vWheelContacts[ii] = world->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact;
    }

    //get the velocity
    world->m_state.m_dV << world->m_pVehicle->getRigidBody()->getLinearVelocity()[0], world->m_pVehicle->getRigidBody()->getLinearVelocity()[1], world->m_pVehicle->getRigidBody()->getLinearVelocity()[2];
    world->m_state.m_dW << world->m_pVehicle->getRigidBody()->getAngularVelocity()[0], world->m_pVehicle->getRigidBody()->getAngularVelocity()[1], world->m_pVehicle->getRigidBody()->getAngularVelocity()[2];

    //set the steering
    world->m_state.m_dSteering = world->m_pVehicle->GetAckermanSteering();
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d MochaVehicle::GetVehicleLinearVelocity(int worldId)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->m_pVehicle->getRigidBody()->getLinearVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d MochaVehicle::GetVehicleAngularVelocity(int worldId)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->m_pVehicle->getRigidBody()->getAngularVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d MochaVehicle::GetVehicleInertiaTensor(int worldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    btVector3 bVec = pWorld->m_pVehicle->getRigidBody()->getInvInertiaDiagLocal();
    Eigen::Vector3d res;
    for(int ii = 0 ; ii < 3 ; ii++) {
      res(ii) = (bVec[ii] == 0 ? 0 : 1/bVec[ii]);
    }
    return res;
}


/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::GetVehicleState(int worldId, VehicleState& stateOut)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    stateOut = pWorld->m_state;
    stateOut.m_dTwv.translation() -= GetBasisVector(stateOut.m_dTwv,pWorld->m_pVehicle->getUpAxis())*
    (pWorld->m_Parameters[CarParameters::SuspRestLength] +
      pWorld->m_Parameters[CarParameters::WheelRadius]+
      pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);

    // stateOut.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
    // stateOut.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
    // for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    // {
    //     Sophus::SE3d Twv;
    //     pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(Twv.data());
    //     stateOut.m_vWheelStates[i] = Sophus::SE3d(Twv);
    //     stateOut.m_vWheelContacts[i] = pWorld->m_pVehicle->getWheelInfo(i).m_raycastInfo.m_isInContact;
    // }
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::SetStateNoReset( BulletWorldInstance *pWorld , const Sophus::SE3d& Twv)
{
    btTransform trans;
    trans.setFromOpenGLMatrix(Twv.matrix().data());
    // btTransform rot_180_x(btQuaternion(1,0,0,0),btVector3(0,0,0));
    // trans = rot_180_x * trans * rot_180_x;
    pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setCenterOfMassTransform(trans);
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::SetState( int nWorldId, VehicleState& state /* in ned */ , bool raycast /* =false */)
{
    BulletWorldInstance *pWorld = GetWorldInstance(nWorldId);
    boost::mutex::scoped_lock lock(*pWorld);
    //load the backup onto the vehicle
    pWorld->m_vehicleBackup.LoadState(pWorld->m_pVehicle);

    //set the wheel positions and contact
    // for(size_t ii = 0; ii < state.m_vWheelStates.size() ; ii++) {
    //     Sophus::SE3d Twv;
    //     pWorld->m_pVehicle->getWheel(ii)->getWorldTransform().getOpenGLMatrix(Twv.data());
    //     pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
    //     pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheel(ii)->isInContact(pWorld->m_pVehicleRayCaster);
    // }

    //update the parameters since they will have been overwritten
    // _InternalUpdateParameters(pWorld);

    // if enabled, use raycast to get state when clamped to ground
    if (raycast)
    {
        // ROS_INFO("raycasting %s",convertEigenMatrix2String(state.m_dTwv.translation().transpose()).c_str());
        Eigen::Vector3d dIntersect;
        RayCast(state.m_dTwv.translation(), -GetBasisVector(state.m_dTwv,2)*0.2, dIntersect, true, nWorldId);
        state.m_dTwv.translation() = dIntersect;
        //  ROS_INFO("done raycasting %s",convertEigenMatrix2String(state.m_dTwv.translation().transpose()).c_str());
    }

    /////////////////////////////////////////////////////////////////////////////

/*
    // set Bullet vehicle chassis transform, offset to account for wheels
    Sophus::SE3d T = state.m_dTwv;
    T.translation() += GetBasisVector(T,2)*
      (pWorld->m_Parameters[CarParameters::SuspRestLength] +
      pWorld->m_Parameters[CarParameters::WheelRadius]+
      pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    SetStateNoReset(pWorld,T);
    pWorld->m_pVehicle->resetSuspension();

    pWorld->m_state = state;
    pWorld->m_state.m_dTwv = T;

    // set Bullet vehicle wheel transforms and update contacts
    for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    {
        // btWheel* wheel = pWorld->m_pVehicle->getWheel(i);

        // set wheel steering position instantaneously
		// btTransform wheelTranCS = btTransform(btQuaternion(0,0,0,1), wheel->getChassisConnection());
        // btTransform wheelTranCS = pWorld->m_pVehicle->getWheelTransformCS(i);
        // btTransform wheelTranCS = pWorld->m_pVehicle->m_worldTran
		btMatrix3x3 yaw;
		yaw.setEulerZYX(0, 0, pWorld->m_pVehicle->getSteeringAngle());
		wheelTranCS.setBasis(yaw);
		btTransform wheelTranWS = pWorld->m_pVehicle->getChassisWorldTransform() * wheelTranCS;
		// wheel->getBody()->setCenterOfMassTransform(wheelTranWS);
        // wheel->setWorldTransform(wheelTransWS);
        pWorld->m_pVehicle->setWheelTransformWS(i,wheelTransWS);

        // pWorld->m_state.m_vWheelContacts[i] = pWorld->m_pVehicle->getWheel(i)->isInContact(pWorld->m_pVehicleRaycaster);
    }
    pWorld->m_pVehicle->updateWheelContacts();

    // set Bullet vehicle chassis control to match state
    btVector3 vel(state.m_dV[0], state.m_dV[1], state.m_dV[2]);
    btVector3 w(state.m_dW[0], state.m_dW[1], state.m_dW[2]);
    // pWorld->m_pVehicle->getRigidBody()->setLinearVelocity(vel);
    pWorld->m_pVehicle->setChassisLinearVelocityWS(vel);
    // pWorld->m_pVehicle->getRigidBody()->setAngularVelocity(w);
    pWorld->m_pVehicle->setChassisAngularVelocityWS(w);

    // set Bullet vehicle wheel controls to match state
    btVector3 velCS = vel * pWorld->m_pVehicle->getChassisTransformWS().getBasis();
    btScalar forwVelCS = velCS[pWorld->m_pVehicle->getForwardAxis()];
    for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    {
        btWheel* wheel = pWorld->m_pVehicle->getWheel(i);

        btVector3 forwAngVelWS(0,0,0);
        forwAngVelWS[pWorld->m_pVehicle->getRightAxis()] = forwVelCS/wheel->getRadius();
        // forwAngVelWS = pWorld->m_pVehicle->getWheel(i)->getBody()->getWorldTransform().getBasis() * forwAngVelWS;
        // forwAngVelWS = pWorld->m_pVehicle->getWheelTransformWS(i).getBasis() * forwAngVelWS;
        forwAngVelWS = pWorld->m_pVehicle->getWheelInfo(i).m_worldTransform.getBasis() * forwAngVelWS;

        // ROS_INFO("forwAngVelWS %f %f %f",forwAngVelWS[0],forwAngVelWS[1],forwAngVelWS[2]);

        // wheel->getBody()->setLinearVelocity(vel);
        pWorld->m_pVehicle->setWheelLinearVelocity(i,vel);
        // wheel->getBody()->setAngularVelocity(forwAngVelWS);
        pWorld->m_pVehicle->setWheelAngularVelocity(i,forwAngVelWS);
    }
    

    // //set the linear velocity of the car
    // btVector3 vel(state.m_dV[0], state.m_dV[1], state.m_dV[2]);
    // btVector3 w(state.m_dW[0], state.m_dW[1], state.m_dW[2]);
    // pWorld->m_pVehicle->getRigidBody()->setLinearVelocity(vel);
    // pWorld->m_pVehicle->getRigidBody()->setAngularVelocity(w);

    // btVector3 velCS = vel * pWorld->m_pVehicle->getChassisWorldTransform().getBasis();
    // btScalar forwVelCS = velCS[pWorld->m_pVehicle->getForwardAxis()];
    // // pWorld->m_pVehicle->setEnabledLinearVelocity(forwVelCS);

    // // ROS_INFO("forwVelCS %f ",forwVelCS);

    // for (uint i=0; i<pWorld->m_pVehicle->getNumWheels(); i++)
    // {
    //     btWheel* wheel = pWorld->m_pVehicle->getWheel(i);

    //     btVector3 forwAngVelWS(0,0,0);
    //     forwAngVelWS[pWorld->m_pVehicle->getRightAxis()] = forwVelCS/wheel->getRadius();
    //     forwAngVelWS = pWorld->m_pVehicle->getWheel(i)->getBody()->getWorldTransform().getBasis() * forwAngVelWS;

    //     // ROS_INFO("forwAngVelWS %f %f %f",forwAngVelWS[0],forwAngVelWS[1],forwAngVelWS[2]);

    //     wheel->getBody()->setLinearVelocity(vel);
    //     wheel->getBody()->setAngularVelocity(forwAngVelWS);

    //     // set wheel steering position instantaneously
	// 	btTransform wheelTranCS = btTransform(btQuaternion(0,0,0,1), wheel->getChassisConnection());
	// 	btMatrix3x3 yaw;
	// 	yaw.setEulerZYX(0, 0, wheel->getSteeringAngle());
	// 	wheelTranCS.setBasis(yaw);
	// 	btTransform wheelTranWS = pWorld->m_pVehicle->getChassisWorldTransform() * wheelTranCS;
	// 	wheel->getBody()->setCenterOfMassTransform(wheelTranWS);

    //     // pWorld->m_state.m_vWheelContacts[i] = pWorld->m_pVehicle->getWheel(i)->isInContact(pWorld->m_pVehicleRaycaster);
    // }

    // btTransform wheelTranCS = pWorld->m_pVehicle->getChassisWorldTransform().inverse() * getWheel(0)->getWorldTransform();
    // btScalar wheelBase = 2*fabs( m_wheelInfo[0].m_chassisConnectionPointCS[pWorld->m_pVehicle->getForwardAxis()] );
    // btScalar wheelBase = pWorld->m_pVehicle->getWheelBase();
    btScalar wheelBase = 2*fabs( pWorld->m_pVehicle->getWorldInfo[0].m_chassisConnectionPointCS[pWorld->m_pVehicle->getForwardAxis()] );
    btScalar steering = state.m_dSteering;
    btScalar currLinVel = pWorld->m_pVehicle->getCurrentSpeedMS();
    // btScalar currLinVel = forwVelCS;
    btScalar yawVel = tan(steering)*currLinVel/wheelBase;
    pWorld->m_pVehicle->setForwardAndYawVelocityCS(forwVelCS, yawVel);

    //set the steering
    // pWorld->m_pVehicle->setEnabledSteeringAngle(state.m_dSteering);
    // pWorld->m_pVehicle->updateConstraints();

    // std::vector<CollisionEvent> collisions;
    // getCollisions(pWorld->m_pDynamicsWorld, collisions);

    // if(pWorld->m_state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
    //     pWorld->m_state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
    //     pWorld->m_state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
    // }

    // for(size_t ii = 0; ii < state.m_vWheelStates.size() ; ii++) {
    //     Sophus::SE3d Twv;
    //     pWorld->m_pVehicle->getWheel(ii)->getWorldTransform().getOpenGLMatrix(Twv.data());
    //     pWorld->m_state.m_vWheelStates[ii] = Sophus::SE3d(Twv);
    //     //     pWorld->m_state.m_vWheelContacts[ii] = pWorld->m_pVehicle->getWheel(ii)->isInContact(pWorld->m_pVehicleRayCaster);
    
    //     bool isInContact = false;
    //     for (uint i=0; i<collisions.size(); i++)
    //     {
    //         if (collisions[i].bodyA==pWorld->m_pVehicle->getWheel(ii)->getBody() || collisions[i].bodyB==pWorld->m_pVehicle->getWheel(ii)->getBody()) 
    //         {
    //             isInContact = true;
    //         }
    //     }
    //     pWorld->m_state.m_vWheelContacts[ii] = isInContact;
    // }
    // std::string prt;
    // for(size_t ii = 0; ii < pWorld->m_state.m_vWheelStates.size() ; ii++) {
    //     prt += (pWorld->m_state.m_vWheelContacts[ii] ? "true " : "false ");
    // }
    // std::cout << prt << std::endl;

    //raycast all wheels so they are correctly positioned
    //    for (int i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    //    {
    //        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
    //        pWorld->m_pVehicle->rayCast(wheel);
    //    }
*/
    //////////////////////////////////////////////////////////////////////////////////////

    if(state.m_vWheelStates.size() != pWorld->m_pVehicle->getNumWheels()) {
        state.m_vWheelStates.resize(pWorld->m_pVehicle->getNumWheels());
        state.m_vWheelContacts.resize(pWorld->m_pVehicle->getNumWheels());
    }

    // //set the wheel positions and contact
    for(size_t ii = 0; ii < state.m_vWheelStates.size() ; ii++) {
      //m_pVehicle->updateWheelTransform(ii,true);
    //   pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform.setFromOpenGLMatrix(state.m_vWheelStates[ii].data());
        pWorld->m_pVehicle->getWheelInfo(ii).m_worldTransform = btTransform(
            btQuaternion(state.m_vWheelStates[ii].unit_quaternion().x(), state.m_vWheelStates[ii].unit_quaternion().y(), state.m_vWheelStates[ii].unit_quaternion().z(),state.m_vWheelStates[ii].unit_quaternion().w()),
            btVector3(state.m_vWheelStates[ii].translation().x(), state.m_vWheelStates[ii].translation().y(), state.m_vWheelStates[ii].translation().z()));
        pWorld->m_pVehicle->getWheelInfo(ii).m_raycastInfo.m_isInContact = state.m_vWheelContacts[ii];
    //   ROS_INFO("Setting state %d w wheels %f %f %f", ii, state.m_vWheelStates[ii].translation().x(), state.m_vWheelStates[ii].translation().y(), state.m_vWheelStates[ii].translation().z());
    }

    // ROS_INFO("= Setting state w wheels %f %f %f, %f %f %f, %f %f %f, %f %f %f",
    //     state.m_vWheelStates[0].translation().x(), state.m_vWheelStates[0].translation().y(), state.m_vWheelStates[0].translation().z(),
    //     state.m_vWheelStates[1].translation().x(), state.m_vWheelStates[1].translation().y(), state.m_vWheelStates[1].translation().z(),
    //     state.m_vWheelStates[2].translation().x(), state.m_vWheelStates[2].translation().y(), state.m_vWheelStates[2].translation().z(),
    //     state.m_vWheelStates[3].translation().x(), state.m_vWheelStates[3].translation().y(), state.m_vWheelStates[3].translation().z());

    // ROS_INFO("- Setting state w wheels %f %f %f, %f %f %f, %f %f %f, %f %f %f",
    //     pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getZ());

    //update the parameters since they will have been overwritten
    // _InternalUpdateParameters(pWorld);
    btVector3 vel(state.m_dV[0], state.m_dV[1], state.m_dV[2]);
    btVector3 w(state.m_dW[0], state.m_dW[1], state.m_dW[2]);

    //set the state 4x4 matrix, however offset the body up to account for the wheel columns
    Sophus::SE3d T = state.m_dTwv;
    T.translation() += GetBasisVector(T,pWorld->m_pVehicle->getUpAxis())*
      (pWorld->m_Parameters[CarParameters::SuspRestLength] +
      pWorld->m_Parameters[CarParameters::WheelRadius]+
      pWorld->m_Parameters[CarParameters::SuspConnectionHeight]);
    SetStateNoReset(pWorld,T);

    pWorld->m_state = state;
    pWorld->m_state.m_dTwv = T;

    //set the linear velocity of the car
    pWorld->m_pVehicle->getRigidBody()->setLinearVelocity(vel);
    pWorld->m_pVehicle->getRigidBody()->setAngularVelocity(w);


    //set the steering
    pWorld->m_pVehicle->SetAckermanSteering(state.m_dSteering);


    //raycast all wheels so they are correctly positioned
    //    for (int i=0;i<pWorld->m_pVehicle->getNumWheels();i++)
    //    {
    //        WheelInfo& wheel = pWorld->m_pVehicle->getWheelInfo(i);
    //        pWorld->m_pVehicle->rayCast(wheel);
    //    }

    // ROS_INFO("+ Setting state w wheels %f %f %f, %f %f %f, %f %f %f, %f %f %f",
    //     pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(0).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(1).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(2).m_worldTransform.getOrigin().getZ(),
    //     pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getX(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getY(), pWorld->m_pVehicle->getWheelInfo(3).m_worldTransform.getOrigin().getZ());

    // if (nWorldId==9)
    // {
    //     _pubTFs(nWorldId);
    //     // _pubVehicleMesh(nWorldId);
    //     // _pubPreviousCommands(nWorldId);
    // }

    // ROS_INFO("New state: p %.2f %.2f %.2f  v %.2f %.2f %.2f  w %.2f %.2f %.2f c %s %s %s %s", 
    //     pWorld->m_state.m_dTwv.translation().x(), pWorld->m_state.m_dTwv.translation().y(), pWorld->m_state.m_dTwv.translation().z(), 
    //     pWorld->m_state.m_dV[0], pWorld->m_state.m_dV[1], pWorld->m_state.m_dV[2],
    //     pWorld->m_state.m_dW[0], pWorld->m_state.m_dW[1], pWorld->m_state.m_dW[2],
    //     (pWorld->m_state.m_vWheelContacts[0]?"1":"0"), (pWorld->m_state.m_vWheelContacts[1]?"1":"0"), (pWorld->m_state.m_vWheelContacts[2]?"1":"0"), (pWorld->m_state.m_vWheelContacts[3]?"1":"0"));
        // force[0], force[1], force[2]
        // pWorld->m_state.m_dSteering
}

/////////////////////////////////////////////////////////////////////////////////////////
btRigidBody*	MochaVehicle::_LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
      shape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    pWorld->m_pDynamicsWorld->addRigidBody(body,group,mask);

    return body;
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::_LocalRemoveRigidBody(BulletWorldInstance *pWorld, btCollisionShape* shape)
{
    std::cout << "removing rigid body" << std::endl;

    btRigidBody* body = dynamic_cast<btRigidBody*>(shape);
    // btCollisionObject* body = dynamic_cast<btCollisionObject*>(shape);
    // body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    pWorld->m_pDynamicsWorld->removeRigidBody(body);
}

/////////////////////////////////////////////////////////////////////////////////////////
std::vector<Sophus::SE3d> MochaVehicle::GetWheelTransforms(const int worldIndex){
    BulletWorldInstance *pWorld = GetWorldInstance(worldIndex);
    return pWorld->m_vWheelTransforms;
}

/////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::ResetCommandHistory(int worldId)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    pWorld->m_lPreviousCommands.clear();
    pWorld->m_dTotalCommandTime = 0;
}

  /////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::GetCommandHistory(int worldId,CommandList &previousCommandsOut)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    previousCommandsOut = pWorld->m_lPreviousCommands;
}

  /////////////////////////////////////////////////////////////////////////////////////////
CommandList& MochaVehicle::GetCommandHistoryRef(int worldId)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    return pWorld->m_lPreviousCommands;
}

  /////////////////////////////////////////////////////////////////////////////////////////
void MochaVehicle::SetCommandHistory(const int& worldId, const CommandList &previousCommands)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    boost::mutex::scoped_lock lock(*pWorld);
    //find out the total time of the commands
    pWorld->m_dTotalCommandTime = 0;
    for(const ControlCommand& command : previousCommands ){
        pWorld->m_dTotalCommandTime += command.m_dT;
    }

    pWorld->m_lPreviousCommands = previousCommands;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool MochaVehicle::RayCast(const Eigen::Vector3d& dSource,const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index /*= 0*/)
{
    btVector3 source(dSource[0],dSource[1],dSource[2]);
    btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
    btVector3 target = source + vec;
    BulletWorldInstance* pInstance = GetWorldInstance(index);

    btVehicleRaycaster::btVehicleRaycasterResult results,results2;
    if( biDirectional ){
        source = source - vec;
    }

    if(pInstance->m_pVehicleRayCaster->castRay(source,target,results) == 0){
        return false;
    }else{
        Eigen::Vector3d dNewSource(source[0],source[1],source[2]);
        dIntersect = dNewSource + results.m_distFraction* (biDirectional ? (Eigen::Vector3d)(dRayVector*2) : dRayVector);
        return true;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
/*

void BulletCarModel::_TerrainMeshPublisherFunc()
{
  while( ros::ok() )
  {
    BulletWorldInstance* pWorld = GetWorldInstance(0);
    time_t t0 = clock();

    _pubMesh(pWorld->m_pTerrainBody->getCollisionShape(), &(pWorld->m_pTerrainBody->getWorldTransform()), &m_terrainMeshPub);
    
    time_t t1 = clock();
    uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(pWorld->m_pTerrainBody->getCollisionShape())->getMeshInterface())->getNumTriangles();
    ROS_INFO("pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
     
    // btCollisionObjectArray objarr = pWorld->m_pDynamicsWorld->getCollisionObjectArray();
    // for(uint i=0; i<objarr.size(); i++)
    // {
    //   if(objarr[i] != pWorld->m_pCarChassis)
    //   {
    //     time_t t0 = clock();

    //     _pubMesh(objarr[i]->getCollisionShape(), &(objarr[i]->getWorldTransform()), &m_terrainMeshPub);
    //     
    //     time_t t1 = clock();
    //     uint num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(objarr[i]->getCollisionShape())->getMeshInterface())->getNumTriangles();
    //     ROS_INFO("pubbing mesh, %d triangles, %.2f sec", num_tri, std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    //   }
    // }
    
    ros::Rate(100).sleep();
  }
}

void BulletCarModel::_pubMesh(btCollisionShape* collisionShape, ros::Publisher* pub)
{
  btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  _pubMesh(collisionShape, parentTransform, pub);
}

void BulletCarModel::_pubMesh(btCollisionShape* collisionShape, btTransform* parentTransform, ros::Publisher* pub)
{
    // ROS_INFO("about to pub mesh with %d faces", dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(collisionShape)->getMeshInterface())->getNumTriangles());

    time_t t0 = std::clock();

    mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
    btTransform* transformedToRosParent = new btTransform(btQuaternion(1,0,0,0),btVector3(0,0,0)); // rot 180 x
    (*transformedToRosParent) *= (*parentTransform);
    convertCollisionShape2MeshMsg(collisionShape, transformedToRosParent, &mesh);

    mesh_msgs::TriangleMeshStamped* mesh_stamped = new mesh_msgs::TriangleMeshStamped();
    mesh_stamped->mesh = *mesh;
    mesh_stamped->header.frame_id = "world";
    mesh_stamped->header.stamp = ros::Time::now();

    time_t t1 = std::clock();

    // ROS_INFO("pubbing mesh, %d faces, %d vertices, %.2f sec", mesh->triangles.size(), mesh->vertices.size(), std::difftime(t1,t0)/CLOCKS_PER_SEC); 
    pub->publish(*mesh_stamped);
    ros::spinOnce();
    // ros::Rate(10).sleep();
}

#define CIMM_REPLACE 1
#define CIMM_APPEND 2
#define CARPLANNER_INPUT_MESH_MODE CIMM_REPLACE // CIMM_REPLACE, CIMM_APPEND
void BulletCarModel::_meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg)
{
  static tf::StampedTransform Twm;
  // static tf2_ros::TransformListener tflistener(m_tfbuffer);

  try
  {
    m_tflistener.waitForTransform("world", "infinitam", ros::Time::now(), ros::Duration(1.0));
    m_tflistener.lookupTransform("world", "infinitam", ros::Time(0), Twm);
    // geometry_msgs::TransformStamped Twm_gm = m_tfbuffer.lookupTransform("world", "infinitam", ros::Time(0));
    // geometry_msg2tf(Twm_gm,&Twm);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    usleep(10000);
    return;
  }

  tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
  Twm.setData(rot_180_x*Twm);

  time_t t0 = std::time(0);

  btCollisionShape* meshShape;// = new btBvhTriangleMeshShape(pTriangleMesh,true,true);
  convertMeshMsg2CollisionShape(new mesh_msgs::TriangleMeshStamped(*mesh_msg), &meshShape);
  // uint new_num_tri = dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(meshShape)->getMeshInterface())->getNumTriangles();

  // BulletWorldInstance* pWorld = GetWorldInstance(0);
  // boost::mutex::scoped_lock lock(*pWorld);


  for (uint i=0; i<m_nNumWorlds; i++)
  {
    BulletWorldInstance* pWorld = GetWorldInstance(i);
    boost::mutex::scoped_lock lock(*pWorld);

  // #if CARPLANNER_INPUT_MESH_MODE==CIMM_REPLACE
  //   if(pWorld->m_pTerrainBody != NULL)
  //   {
  //     pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
  //   }
  //   pWorld->m_pTerrainShape = meshShape;
  //   pWorld->m_pTerrainBody->setCollisionShape(pWorld->m_pTerrainShape);
  //   pWorld->m_pTerrainBody->setWorldTransform(btTransform(
  //     btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
  //     btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
  //   pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);

  // #elif CARPLANNER_INPUT_MESH_MODE==CIMM_APPEND
    if( pWorld->m_pDynamicsWorld->getCollisionWorld()->getNumCollisionObjects() !< 2 )
      continue;
    btTransform tr;
    tr.setIdentity();
    btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));
    btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,meshShape,btVector3(0,0,0));
    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);
    body->setWorldTransform(btTransform(
      btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
      btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
    pWorld->m_pDynamicsWorld->addRigidBody(body);

  // #else
  //   //error
  // #endif

  }

  time_t t1 = std::time(0);
  ROS_INFO("got mesh, %d faces, %d vertices, %.2f sec", mesh_msg->mesh.triangles.size(), mesh_msg->mesh.vertices.size(), difftime(t1,t0));

}

void BulletCarModel::replaceMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
  // btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  BulletWorldInstance* pWorld = GetWorldInstance(worldId);
  // boost::unique_lock<boost::mutex> lock(*pWorld);
  pWorld->lock();

  if(pWorld->m_pTerrainBody != NULL)
  {
    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pTerrainBody);
  }

  pWorld->m_pTerrainShape = meshShape;
  pWorld->m_pTerrainBody->setCollisionShape(pWorld->m_pTerrainShape);
  pWorld->m_pTerrainBody->setWorldTransform(btTransform(
    btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
    btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
  pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);
  
  // boost::unique_lock<boost::mutex> unlock(*pWorld);
  pWorld->unlock();
}

void BulletCarModel::appendMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm)
{
  btAssert((!meshShape || meshShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

  BulletWorldInstance* pWorld = GetWorldInstance(worldId);
  // boost::unique_lock<boost::mutex> lock(*pWorld);
  pWorld->lock();
  
  uint max_num_coll_objs = 3;
  if( pWorld->m_pDynamicsWorld->getCollisionWorld()->getNumCollisionObjects() >= max_num_coll_objs )
  {
    // boost::unique_lock<boost::mutex> unlock(*pWorld);
    pWorld->unlock();
    replaceMesh(worldId, meshShape, Twm);
    return;
  }

  btTransform tr;
  tr.setIdentity();
  btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
  
  btRigidBody::btRigidBodyConstructionInfo cInfo(0.0,myMotionState,meshShape,btVector3(0,0,0));
  btRigidBody* body = new btRigidBody(cInfo);
  body->setContactProcessingThreshold(BT_LARGE_FLOAT);
  body->setWorldTransform(btTransform(
    btQuaternion(Twm.getRotation().getX(),Twm.getRotation().getY(),Twm.getRotation().getZ(),Twm.getRotation().getW()),
    btVector3(Twm.getOrigin().getX(),Twm.getOrigin().getY(),Twm.getOrigin().getZ())));
  pWorld->m_pTerrainBody = body;
  pWorld->m_pDynamicsWorld->addRigidBody(pWorld->m_pTerrainBody);

  // boost::unique_lock<boost::mutex> unlock(*pWorld);
  pWorld->unlock();
}

*/

// void* DefaultVehicleRaycaster::castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
// {
// //	RayResultCallback& resultCallback;
//     btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

//     m_dynamicsWorld->rayTest(from, to, rayCallback);

//     if (rayCallback.hasHit()){
//         result.m_distFraction = 0;
//         const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
//         if (body && body->hasContactResponse()){
//             result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
//             result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
//             result.m_hitNormalInWorld.normalize();
//             result.m_distFraction = rayCallback.m_closestHitFraction;
//             return (void*)body;
//         }
//     }
//     return 0;
// }
