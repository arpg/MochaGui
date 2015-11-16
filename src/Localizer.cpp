#include "MochaGui/Localizer.h"

#include <math.h>
#include <unistd.h>

#include "CarPlanner/CarPlannerCommon.h"
#include "CarPlanner/MochaException.h"
#include "RpgUtils.h"

//////////////////////////////////////////////////////////////////
Eigen::Vector3d Quat2Euler( double *Q )
{
  Eigen::Vector3d X;

  X(2)   = atan2((Q[2] * Q[3] + Q[0] * Q[1]) * 2,
      (Q[1] * Q[1] + Q[2] * Q[2])*-2 + 1) ;

  X(1) = -asin((Q[0] * Q[2] - Q[1] * Q[3])*2);

  X(0)   = -atan2((Q[1] * Q[2] + Q[0] * Q[3])*2,
      (Q[2] * Q[2] + Q[3] * Q[3])*-2 + 1 ) ;

  return X;
}

//////////////////////////////////////////////////////////////////
Localizer::Localizer()
{
  m_bIsStarted = false;
  m_abStopLocalizer = false;
  m_pNode.reset(new node::node( false ));
  m_NodeReceiverName = "bushido";
  m_pNode->init(m_NodeReceiverName);
}

//////////////////////////////////////////////////////////////////
void Localizer::TrackObject(
    const std::string& sNodeName,
    const std::string& sObjectName,
    bool bRobotFrame /*= true*/
    ){
  TrackObject(sNodeName,sObjectName,Sophus::SE3d(),bRobotFrame);
}

//////////////////////////////////////////////////////////////////
void Localizer::TrackObject(
    const std::string& sNodeName,
    const std::string& sObjectName,
    Sophus::SE3d dToffset,
    bool bRobotFrame /*= true*/
    )
{


  TrackerObject* pObj = &m_mObjects[ sObjectName ];
  pObj->m_sUri = sNodeName + "/" + sObjectName;

  this->m_pNode->subscribe(pObj->m_sUri);
  pObj->m_dToffset = dToffset;
  pObj->m_bRobotFrame = bRobotFrame;
  pObj->m_pLocalizerObject = this;

}

//////////////////////////////////////////////////////////////////
Localizer::~Localizer()
{
  std::map< std::string, TrackerObject >::iterator it;
  for( it = m_mObjects.begin(); it != m_mObjects.end(); it++ ){
    delete( it->second.m_pLocalizerObject );
  }
// std::shared_ptr deletes itself.
}

//////////////////////////////////////////////////////////////////
void Localizer::Start()
{
  if( m_bIsStarted == true ) {
    throw MochaException("The Localizer thread has already started.");
  }

  m_pThread = new std::thread(Localizer::_ThreadFunction, this);
  m_bIsStarted = true;
}

//////////////////////////////////////////////////////////////////
void Localizer::Stop()
{
  if( m_bIsStarted == false ) {
    throw MochaException("No thread is running!"); //crh uncommented
    return;
  }

  m_pThread->join();

  m_bIsStarted = false;
}

//////////////////////////////////////////////////////////////////
Sophus::SE3d Localizer::GetPose( const std::string& sObjectName, bool blocking/* = false */,
                             double* time /*= NULL*/, double* rate /*= NULL*/)
{
  if( m_mObjects.find( sObjectName ) == m_mObjects.end() ){
    throw MochaException("Invalid object name.");
  }

  TrackerObject& obj = m_mObjects[sObjectName];
  std::unique_lock<std::mutex> lock(obj.m_Mutex);

  //if blocking wait until we have a signal that the pose for this object has been updated
  if(blocking && obj.m_bPoseUpdated == false){
    obj.m_PoseUpdated.wait(lock);
  }
  obj.m_bPoseUpdated = false;
  Sophus::SE3d pose = m_mObjects[sObjectName].m_dSensorPose;
  if(time != NULL){
    *time = m_mObjects[sObjectName].m_dTime;
  }
  if(rate != NULL){
    *rate = m_mObjects[sObjectName].m_dPoseRate;
  }
  return pose;
}

//////////////////////////////////////////////////////////////////
//
//Eigen::Matrix<double,6,1> Localizer::GetdPose( const std::string& sObjectName )
//{
//    if( m_mObjects.find( sObjectName ) == m_mObjects.end() ){
//        throw MochaException("Invalid object name.");
//    }

//    //lock the sensor pose for readout
//    lock();
//    Eigen::Matrix<double,6,1> pose = m_mObjects[sObjectName].m_dDSensorPose;
//    unlock();

//    return pose;
//}

//////////////////////////////////////////////////////////////////
eLocType Localizer::WhereAmI( Eigen::Vector3d P )
{
  return VT_AIR;
}

//////////////////////////////////////////////////////////////////
eLocType Localizer::WhereAmI( Eigen::Matrix<double, 6, 1 > P )
{
  Eigen::Vector3d Pv = P.block < 3, 1 > (0, 0);
  return WhereAmI( Pv );
}

//////////////////////////////////////////////////////////////////
void Localizer::_ThreadFunction(Localizer *pL)
{
  while (1) {
    std::map< std::string, TrackerObject >::iterator it;
    for( it = pL->m_mObjects.begin(); it != pL->m_mObjects.end(); it++ ) {
      ninjacar::PoseMsg msg;
      pL->m_pNode->receive(it->second.m_sUri,msg); //crh replaced vrpn_Tracker_Remote.mainloop()
      ninjacar::ReadPoseSE3(msg,&it->second.m_dSensorPose);
      std::cout << "Blocking call needed? Localizer.cpp l. 163." << std::endl;
      //boost::this_thread::interruption_point(); //crh replaced with next line
      if(pL->m_abStopLocalizer) { return; }

    }

    //small sleep to not eat up all the cpu
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

//////////////////////////////////////////////////////////////////
//void VRPN_CALLBACK Localizer::_MoCapVelHandler( void* uData, const vrpn_TRACKERVELCB tData )
//{
//  //TrackerObject* pObj = (TrackerObject*)uData;
//  //Localizer* pLocalizer = pObj->m_pLocalizerObject;

//  //pObj->m_dDSensorPose.block<3,1>(0,0) << tData.vel[0],tData.vel[1],tData.vel[2];
//  //double dQuats[4] = { tData.vel_quat[0], tData.vel_quat[1], tData.vel_quat[2], tData.vel_quat[3] };
//  //pObj->m_dDSensorPose.block < 3, 1 > (3, 0) = Quat2Euler( dQuats );
//}

//////////////////////////////////////////////////////////////////
//void VRPN_CALLBACK Localizer::_MoCapHandler( void* uData, const vrpn_TRACKERCB tData )
//{
//  TrackerObject* pObj = (TrackerObject*)uData;
//  //lock the sensor poses as we update them
//  {
//    std::unique_lock<std::mutex> lock(pObj->m_Mutex);

//    Eigen::Matrix4d T;
//    if(pObj->m_bRobotFrame){
//      T << 1, 0, 0, 0,
//          0, -1, 0, 0,
//          0, 0, -1, 0,
//          0, 0 , 0, 1;
//    }else{
//      T = Eigen::Matrix4d::Identity();
//    }
//    Eigen::Vector3d Pos;
//    Pos << tData.pos[0], tData.pos[1], tData.pos[2];

//    //get the pose and transform it as necessary
//    Sophus::SE3d Twc(Sophus::SO3d(Eigen::Quaterniond(tData.quat)),Pos);
//    pObj->m_dSensorPose = Sophus::SE3d(T) * (pObj->m_dToffset * Twc);

//    //now calculate the time derivative
//    double LocalizerTime = tData.msg_time.tv_sec + 1e-6*tData.msg_time.tv_usec;

//    //calculate metrics
//    if(pObj->m_dLastTime == -1){
//      pObj->m_dLastTime = LocalizerTime;
//      pObj->m_nNumPoses = 0;
//      pObj->m_dPoseRate = 0;
//    }else if((LocalizerTime - pObj->m_dLastTime) > 1){
//      pObj->m_dPoseRate = pObj->m_nNumPoses /(LocalizerTime - pObj->m_dLastTime) ;
//      pObj->m_dLastTime = LocalizerTime;
//      pObj->m_nNumPoses = 0;
//    }

//    pObj->m_nNumPoses++;

//    pObj->m_dTime = LocalizerTime;

//  }

//  //signal that the object has been update
//  pObj->m_bPoseUpdated = true;
//  pObj->m_PoseUpdated.notify_all();
//}
