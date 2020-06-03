/*
 * File:   MochaVehicle.h
 * Author: nima
 *
 * Created on May 7, 2012, 1:13 PM
 */

#ifndef MOCHAVEHICLE_H
#define	MOCHAVEHICLE_H

#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <carplanner_msgs/VehicleState.h>
#include <carplanner_msgs/MotionSample.h>
#include <carplanner_msgs/Command.h>
#include <carplanner_msgs/ResetMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <nav_msgs/Odometry.h>

#include <nodelet/nodelet.h>

#include "mesh_conversion_tools.hpp"
#include "tf_conversion_tools.hpp"
//#include "/home/ohrad/code/mochagui/mesh_conversion_tools.hpp"
//#include "/home/ohrad/code/mochagui/tf_conversion_tools.hpp"
// #include <mochagui/conversion_tools.h>

#include <actionlib/server/simple_action_server.h>
#include <carplanner_msgs/ApplyVelocitiesAction.h>
// #include <carplanner_msgs/UpdateStateAction.h>
// #include <carplanner_msgs/GetGravityCompensationAction.h>
// #include <carplanner_msgs/GetFrictionCompensationAction.h>
#include <carplanner_msgs/GetControlDelayAction.h>
#include <carplanner_msgs/GetInertiaTensorAction.h>
#include <carplanner_msgs/SetNoDelayAction.h>
#include <carplanner_msgs/RaycastAction.h>

// #include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
// #include "ExampleBrowser/CollisionShape2TriangleMesh.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

#include <boost/thread.hpp>
#include <queue>
#include <CarPlanner/threadpool.hpp>
#include <CarPlanner/RpgUtils.h>

// #include <CarPlanner/GLDebugDrawer.h>

#include <boost/thread.hpp>
#include <boost/signals2/mutex.hpp>

#include <CarPlanner/CarParameters.h>
#include <CarPlanner/RaycastVehicle.h>
#include "sophus/se3.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include <string.h>
#include <unistd.h>
// #include <HAL/Messages/Command.h>
// #include <HAL/Messages/Matrix.h>
// #include <HAL/Messages/Pose.h>
#include <exception>
#include <ctime>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <google/protobuf/message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl_lite.h>


#define CAR_UP_AXIS 2   //this is the index for the bullet Z axis
#define CAR_FORWARD_AXIS 0   //this is the index for the bullet X axis
#define CAR_RIGHT_AXIS 1   //this is the index for the bullet Y axis
#define CUBE_HALF_EXTENTS 1

#define BULLET_MODEL_GRAVITY 9.81

#define MAX_SERVO_ANGLE 45


//collision filtering
#define COL_NOTHING 0
#define COL_RAY 1   //this is the "default" filter that we need to include for raycasting (on the ground only)
#define COL_CAR 2
#define COL_GROUND 4

//vehicle parameter ordering
#define VEHICLE_NUM_PARAMS 5
#define VEHICLE_WIDTH 0.21
#define VEHICLE_WHEEL_BASE 0.27
#define VEHICLE_WHEEL_RADIUS 0.04
#define VEHICLE_WHEEL_WIDTH 0.025
#define MIN_CONTROL_DELAY 0.0
#define MAX_CONTROL_DELAY 0.3

/// Structure to hold the steering, acceleration and reaction wheel caommands that are sent to the vehicle
class ControlCommand
{
public:
    ControlCommand(): m_dForce(0), m_dCurvature(0), m_dT(0),m_dPhi(0), m_dTorque(0,0,0),m_dTime(0)
    {

    }
    ControlCommand(const double& force,const double& curvature,
                    const Eigen::Vector3d& torques, const double& dt, const double& dPhi){

        m_dForce = force;
        //m_dControlAccel = accel;
        m_dCurvature = curvature;
        m_dTorque = torques;
        m_dT = dt;
        m_dPhi = dPhi;
    }

    carplanner_msgs::Command toROS()
    {
        carplanner_msgs::Command msg;

        msg.force = m_dForce;
        msg.curvature = m_dCurvature;
        msg.dt = m_dT;
        msg.dphi = m_dPhi;
        msg.torques.clear();
        msg.torques.push_back(m_dTorque[0]);
        msg.torques.push_back(m_dTorque[1]);
        msg.torques.push_back(m_dTorque[2]);
        msg.time = m_dTime;

        return msg;
    }

    void fromROS(carplanner_msgs::Command& msg)
    {
        m_dForce = msg.force;
        m_dCurvature = msg.curvature;
        m_dT = msg.dt;
        m_dPhi = msg.dphi;
        m_dTorque[0] = msg.torques[0];
        m_dTorque[1] = msg.torques[1];
        m_dTorque[2] = msg.torques[2];
        m_dTime = msg.time;

        return;
    }

    //double m_dControlAccel;
    double m_dForce;
    double m_dCurvature;
    double m_dT;
    double m_dPhi;
    Eigen::Vector3d m_dTorque;
    double m_dTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

/// List holding the previous commands sent to the model (Newest commands at the front, oldest at the back)
typedef std::list<ControlCommand> CommandList;

struct VehicleState
{
    VehicleState()
    {
        m_dTwv = Sophus::SE3d();
        m_dV.setZero();
        m_dW.setZero();
        m_dSteering = 0;
        m_dCurvature = 0 ;
        m_dTime = 0;
    }

    VehicleState(const Sophus::SE3d& dTwv, const double dV = 1, double dCurvature = 0)
    {
        m_dCurvature = dCurvature;
        m_dTwv = dTwv;
        Eigen::Vector3d vecV;
        vecV << dV,0,0;
        m_dV = m_dTwv.so3()*vecV;
        m_dW << 0,0,0;
        m_dSteering = 0;
    }

    // VehicleState& operator= (const VehicleState& other)
    // {
    //     m_dTwv = other.m_dTwv;
    //     m_dV = other.m_dV;
    //     m_dW = other.m_dW;
    //     m_dSteering = other.m_dSteering;
    //     m_dCurvature = other.m_dCurvature;
    //     m_dTime = other.m_dTime;

    //     m_vWheelStates.resize(other.m_vWheelStates.size());
    //     for (uint i=0; i<m_vWheelStates.size(); i++)
    //     {
    //         m_vWheelStates[i] = other.m_vWheelStates[i];
    //     }

    //     m_vWheelContacts.resize(other.m_vWheelContacts.size());
    //     for (uint i=0; i<m_vWheelContacts.size(); i++)
    //     {
    //         m_vWheelContacts[i] = other.m_vWheelContacts[i];
    //     }

    //     return *this;
    // }

    static VehicleState GetInterpolatedState(const std::vector<VehicleState>& vStates,
                                             const int nStartIndex,
                                             const double& time,
                                             int& nIndex)
    {
        VehicleState stateOut = vStates.back();
        double currTime = vStates.front().m_dTime;
        for( size_t ii = nStartIndex; ii < vStates.size()-1 ; ii++){
            nIndex = ii;
            if(vStates[ii+1].m_dTime >= time){
                double interpolation = (time-currTime)/(vStates[ii+1].m_dTime-currTime);
                const VehicleState& state2 = vStates[ii+1];
                const VehicleState& state1 = vStates[ii];
                stateOut.m_dCurvature = interpolation*state2.m_dCurvature + (1-interpolation)*state1.m_dCurvature;
                stateOut.m_dSteering = interpolation*state2.m_dSteering + (1-interpolation)*state1.m_dSteering;
                stateOut.m_dV = interpolation*state2.m_dV + (1-interpolation)*state1.m_dV;
                stateOut.m_dW = interpolation*state2.m_dW + (1-interpolation)*state1.m_dW;
                Eigen::Vector3d trans = interpolation*state2.m_dTwv.translation() + (1-interpolation)*state1.m_dTwv.translation();
                Eigen::Quaterniond rot = state1.m_dTwv.so3().unit_quaternion().slerp(interpolation,state2.m_dTwv.so3().unit_quaternion());
                stateOut.m_dTwv = Sophus::SE3d(rot,trans);
                break;
            }else{
                currTime = vStates[ii+1].m_dTime;
            }
        }
        return stateOut;
    }

    void UpdateWheels(const std::vector<Sophus::SE3d>& vWheelTransforms){
        Sophus::SE3d bodyT = m_dTwv;
        m_vWheelContacts.resize(4);
        m_vWheelStates.resize(4);
        for(size_t ii = 0 ; ii < m_vWheelContacts.size() ; ii++){
            //position the wheels with respect to the body
            fflush(stdout);
            Sophus::SE3d T = bodyT* vWheelTransforms[ii];
            m_vWheelStates[ii] = T;
        }
    }

    static void AlignWithVelocityVector(Sophus::SE3d& Twv, const Eigen::Vector3d& dV)
    {
        Eigen::Vector3d vel = dV.normalized();
        Eigen::Matrix3d R = Twv.so3().matrix();
        R.block<3,1>(0,0) = vel;
        R.block<3,1>(0,2) = R.block<3,1>(0,0).cross(R.block<3,1>(0,1)).normalized();
        R.block<3,1>(0,1) = R.block<3,1>(0,2).cross(R.block<3,1>(0,0)).normalized();
        Twv.so3() = Sophus::SO3d(R);
    }

    void AlignWithVelocityVector()
    {
        VehicleState::AlignWithVelocityVector(m_dTwv,m_dV);
    }

    /// Checks all ground contact points, and returns true if all wheels are off the ground
    bool IsAirborne() const
    {
        bool ground = false;
        if(m_vWheelContacts.size() == 4 )
        {
            for(unsigned int ii = 0 ; ii < 4 ; ii++){
                if(m_vWheelContacts[ii] == true) {
                    ground = true;
                }
            }
        }
        return !ground;
    }

    double GetTheta() const
    {
        //calculate theta
        Eigen::Vector3d down,right,forward, trueForward;
        down << 0,0,1;
        trueForward << 1,0,0;
        forward = GetBasisVector(m_dTwv,0); //get the forward
        return atan2(forward[1],forward[0]);
    }

    /// This function gets a vehicle state and returns a 5d pose vector which is parametrized as
    /// [x,y,t,k,v] where t is the 2d angle, k is the path curvature and v is the velocity
    static Eigen::Vector6d VehicleStateToXYZTCV(const VehicleState& state //< The given state to construct the pose from
                                              )
    {
        //Eigen::Vector6d dPose = mvl::T2Cart(state.m_dTwv);
        //angle = dPose[5];
        Eigen::Vector6d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   state.GetTheta(),
                   state.m_dCurvature,
                   state.m_dV.norm();
        return poseOut;
    }

    /// Uses the VehicleStateToXYZTCV function to convert the
    Eigen::Vector6d ToXYZTCV(){
        return VehicleStateToXYZTCV(*this);
    }

    // x y z roll pitch yaw
    static Eigen::Vector6d VehicleStateToXYZRPY(const VehicleState& state)
    {
        // Eigen::Vector7d poseTmp = VehicleStateToXYZQuat(state);
        Eigen::Quaterniond quatTmp = state.m_dTwv.unit_quaternion();
        // Eigen::Quaterniond quatTmp(state.m_dTwv.unit_quaternion());
        // Eigen::Vector3d yprTmp = quatTmp.toYawPitchRoll();
        auto rpyTmp = quatTmp.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector6d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   rpyTmp[0],
                   rpyTmp[1],
                   rpyTmp[2];
        return poseOut;
    }

    Eigen::Vector6d ToXYZRPY(){
        return VehicleStateToXYZRPY(*this);
    }

    // x y z qx qy qz qw
    static Eigen::Vector7d VehicleStateToXYZQuat(const VehicleState& state)
    {
        Eigen::Vector7d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   state.m_dTwv.unit_quaternion().x(),
                   state.m_dTwv.unit_quaternion().y(),
                   state.m_dTwv.unit_quaternion().z(),
                   state.m_dTwv.unit_quaternion().w();
        return poseOut;
    }

    Eigen::Vector7d ToXYZQuat(){
        return VehicleStateToXYZQuat(*this);
    }

    static Sophus::SE3d VehicleStateToSE3d(const VehicleState& state)
    {
        Sophus::SE3d poseOut;
        Eigen::Vector7d state_vec = VehicleStateToXYZQuat(state);
        poseOut.translation() = *(new Eigen::Vector3d(state_vec[0], state_vec[1], state_vec[2]));
        poseOut.setQuaternion(Eigen::Quaterniond(state_vec[6], state_vec[3], state_vec[4], state_vec[5]));
        return poseOut;
    }

    Sophus::SE3d ToSE3d()
    {
        return VehicleStateToSE3d(*this);
    }

    // This is used to flip between the coord convention between gl/viz/bullet and physical/ros
    VehicleState FlipCoordFrame()
    {
        Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));
        m_dTwv = rot_180_x * (*this).m_dTwv * rot_180_x;

        // (*this).m_dV = (*this).m_dTwv.so3() * (*this).m_dV;
        m_dV = (rot_180_x * Sophus::SE3d(Eigen::Quaterniond(1,0,0,0), (*this).m_dV) * rot_180_x).translation();

        return *this;
    }

    double GetNormalVelocity(const VehicleState& state)
    {
      return state.m_dV.norm();
    }

    double GetNormalVelocity()
    {
      return GetNormalVelocity(*this);
    }

    double GetTime() const
    {
       return m_dTime;
    }

    Eigen::Vector6d GetVels() const
    {
      Eigen::Vector6d vels;
      vels << (*this).m_dV[0],
              (*this).m_dV[1],
              (*this).m_dV[2],
              (*this).m_dW[0],
              (*this).m_dW[1],
              (*this).m_dW[2];
      return vels;
    }

    Eigen::Vector7d GetPose() const
    {
      Sophus::SE3d state = m_dTwv;
      Eigen::Vector3d trans = state.translation();
      Eigen::Quaterniond quat = state.unit_quaternion();

      Eigen::Vector7d pose;
      pose << trans[0],
              trans[1],
              trans[2],
              quat.x(),
              quat.y(),
              quat.z(),
              quat.w();

      return pose;
    }

    std::vector<Eigen::Vector7d> GetWheelPoses() const
    {
      std::vector<Eigen::Vector7d> poses;

      for(uint ii=0; ii<m_vWheelStates.size(); ii++)
      {
        Sophus::SE3d state = m_vWheelStates[ii];
        Eigen::Vector3d trans = state.translation();
        Eigen::Quaterniond quat = state.unit_quaternion();

        Eigen::Vector7d pose;
        pose << trans[0],
                trans[1],
                trans[2],
                quat.x(),
                quat.y(),
                quat.z(),
                quat.w();

        poses.push_back(pose);
      }

      return poses;
    }

    std::vector<bool> GetWheelContacts() const
    {
      std::vector<bool> contacts;

      for(uint ii=0; ii<m_vWheelContacts.size(); ii++)
      {
        bool contact = m_vWheelContacts[ii];
        contacts.push_back(contact);
      }

      return contacts;
    }

    double GetCurvature() const
    {
       return m_dCurvature;
    }

    double GetSteering() const
    {
       return m_dSteering;
    }

    // void fromROS(nav_msgs::Odometry& msg)
    // {
    //     m_dForce = msg.force;
    //     m_dCurvature = msg.curvature;
    //     m_dT = msg.dt;
    //     m_dPhi = msg.dphi;
    //     m_dTorque[0] = msg.torques[0];
    //     m_dTorque[1] = msg.torques[1];
    //     m_dTorque[2] = msg.torques[2];
    //     m_dTime = msg.time;

    //     return;
    // }

    inline static VehicleState& OdomMsg2VehicleState(const nav_msgs::Odometry& odom_msg, double steer=0, double curv=0)
    {
        VehicleState* state = new VehicleState();
        state->m_dTime                       = odom_msg.header.stamp.sec + (double)odom_msg.header.stamp.nsec*(double)1e-9;

        state->m_dTwv.translation()[0]       = odom_msg.pose.pose.position.x;
        state->m_dTwv.translation()[1]       = odom_msg.pose.pose.position.y;
        state->m_dTwv.translation()[2]       = odom_msg.pose.pose.position.z;

        state->m_dTwv.setQuaternion(Sophus::Quaterniond(odom_msg.pose.pose.orientation.w,
                                                       odom_msg.pose.pose.orientation.x,
                                                       odom_msg.pose.pose.orientation.y,
                                                       odom_msg.pose.pose.orientation.z));
        // state->m_dTwv.unit_quaternion().x() = odom_msg.pose.pose.orientation.x;
        // state->m_dTwv.unit_quaternion().y() = odom_msg.pose.pose.orientation.y;
        // state->m_dTwv.unit_quaternion().z() = odom_msg.pose.pose.orientation.z;
        // state->m_dTwv.unit_quaternion().w() = odom_msg.pose.pose.orientation.w;

        state->m_dV[0]                       = odom_msg.twist.twist.linear.x;
        state->m_dV[1]                       = odom_msg.twist.twist.linear.y;
        state->m_dV[2]                       = odom_msg.twist.twist.linear.z;

        state->m_dV = state->m_dTwv.so3() * state->m_dV;

        // Eigen::Vector3d linvel_cs(odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.linear.z);
        // Eigen::Vector3d linvel_ws = state->m_dTwv.so3() * linvel_cs;
        // state->m_dV = linvel_ws;

        state->m_dW[0]                       = odom_msg.twist.twist.angular.x;
        state->m_dW[1]                       = odom_msg.twist.twist.angular.y;
        state->m_dW[2]                       = odom_msg.twist.twist.angular.z;

        state->m_dCurvature = curv;
        state->m_dSteering = steer;

        return (*state);
    }

    static VehicleState tf2VehicleState(tf::StampedTransform tf, double steer=0, double curv=0)
    {
        static VehicleState last_state;
        VehicleState state;
        state.m_dTime                       = tf.stamp_.sec + (double)tf.stamp_.nsec*(double)1e-9;

        state.m_dTwv.translation()[0]       = tf.getOrigin().getX();
        state.m_dTwv.translation()[1]       = tf.getOrigin().getY();
        state.m_dTwv.translation()[2]       = tf.getOrigin().getZ();
        // state.m_dTwv.unit_orientation().x() = tf.getRotation().getX();
        // state.m_dTwv.unit_orientation().y() = tf.getRotation().getY();
        // state.m_dTwv.unit_orientation().z() = tf.getRotation().getZ();
        // state.m_dTwv.unit_orientation().w() = tf.getRotation().getW();
        state.m_dTwv.setQuaternion(Eigen::Quaterniond(tf.getRotation().getW(),
                                                      tf.getRotation().getX(),
                                                      tf.getRotation().getY(),
                                                      tf.getRotation().getZ()));

        state.m_dV[0]                       = (state.m_dTwv.translation()[0] - last_state.m_dTwv.translation()[0])/(state.m_dTime - last_state.m_dTime);
        state.m_dV[1]                       = (state.m_dTwv.translation()[1] - last_state.m_dTwv.translation()[1])/(state.m_dTime - last_state.m_dTime);
        state.m_dV[2]                       = (state.m_dTwv.translation()[2] - last_state.m_dTwv.translation()[2])/(state.m_dTime - last_state.m_dTime);

        Eigen::Quaterniond dq((state.m_dTwv.unit_quaternion().w() - last_state.m_dTwv.unit_quaternion().w())/(state.m_dTime - last_state.m_dTime),
                              (state.m_dTwv.unit_quaternion().x() - last_state.m_dTwv.unit_quaternion().x())/(state.m_dTime - last_state.m_dTime),
                              (state.m_dTwv.unit_quaternion().y() - last_state.m_dTwv.unit_quaternion().y())/(state.m_dTime - last_state.m_dTime),
                              (state.m_dTwv.unit_quaternion().z() - last_state.m_dTwv.unit_quaternion().z())/(state.m_dTime - last_state.m_dTime));
        // // roll (x-axis rotation)
        // double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        // double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        // state.m_dW[0] = std::atan2(sinr_cosp, cosr_cosp);
        // // pitch (y-axis rotation)
        // double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        // if (std::abs(sinp) >= 1)
        //     state.m_dW[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        // else
        //     state.m_dW[1] = std::asin(sinp);
        // // yaw (z-axis rotation)
        // double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        // double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        // state.m_dW[2] = std::atan2(siny_cosp, cosy_cosp);

        // state.m_dW[0] = Eigen::Roll(q);
        // state.m_dW[1] = Eigen::Pitch(q);
        // state.m_dW[2] = Eigen::Yaw(q);

        state.m_dW = Eigen::Matrix3d(dq).eulerAngles(0,1,2);

        state.m_dCurvature = curv;
        state.m_dSteering = steer;

        last_state = state;
        return state;
    }

    carplanner_msgs::VehicleState toROS() const
    {
    //   Sophus::SE3d rot_180_y(Eigen::Quaterniond(0,0,1,0),Eigen::Vector3d(0,0,0)); // Quat(w,x,y,z) , Vec(x,y,z)
    //   Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));

      carplanner_msgs::VehicleState state_msg;
      // state_msg.header.stamp.sec = (*this).GetTime();
      // state_msg.header.frame_id = "world";

    //   Sophus::SE3d Twv = rot_180_x*(*this).m_dTwv*rot_180_x;
      Sophus::SE3d Twv = (*this).m_dTwv;
      state_msg.pose.header.stamp.sec = (*this).GetTime();
      state_msg.pose.header.frame_id = "world";
      state_msg.pose.child_frame_id = "base_link";
      state_msg.pose.transform.translation.x = Twv.translation()[0];
      state_msg.pose.transform.translation.y = Twv.translation()[1];
      state_msg.pose.transform.translation.z = Twv.translation()[2];
      state_msg.pose.transform.rotation.w = Twv.unit_quaternion().w();
      state_msg.pose.transform.rotation.x = Twv.unit_quaternion().x();
      state_msg.pose.transform.rotation.y = Twv.unit_quaternion().y();
      state_msg.pose.transform.rotation.z = Twv.unit_quaternion().z();

      for( unsigned int i=0; i<(*this).m_vWheelStates.size(); i++ )
      {
          geometry_msgs::TransformStamped tf;
          tf.header.stamp = ros::Time::now();
          tf.header.frame_id = "world";
          tf.child_frame_id = "wheel_link" + std::to_string(i);

          Sophus::SE3d Twv = (*this).m_vWheelStates[i];
          tf.transform.translation.x = Twv.translation()[0];
          tf.transform.translation.y = Twv.translation()[1];
          tf.transform.translation.z = Twv.translation()[2];
          tf.transform.rotation.w = Twv.unit_quaternion().w();
          tf.transform.rotation.x = Twv.unit_quaternion().x();
          tf.transform.rotation.y = Twv.unit_quaternion().y();
          tf.transform.rotation.z = Twv.unit_quaternion().z();

          state_msg.wheel_poses.push_back( tf );
      }

      std::vector<bool> wheel_contacts = (*this).GetWheelContacts();
      for(uint ii=0; ii<wheel_contacts.size(); ii++)
      {
        bool contact = wheel_contacts[ii];
        state_msg.wheel_contacts.push_back(contact);
      }

      Eigen::Vector6d vels = (*this).GetVels();
      state_msg.lin_vel.x = vels[0];
      state_msg.lin_vel.y = vels[1];
      state_msg.lin_vel.z = vels[2];
      state_msg.ang_vel.x = vels[3];
      state_msg.ang_vel.y = vels[4];
      state_msg.ang_vel.z = vels[5];

      state_msg.curvature = (*this).GetCurvature();

      state_msg.steering = (*this).GetSteering();

      return state_msg;
    }

    void fromROS(carplanner_msgs::VehicleState msg)
    {
        (*this).m_dTwv.translation() = Eigen::Vector3d(
            msg.pose.transform.translation.x,
            msg.pose.transform.translation.y,
            msg.pose.transform.translation.z);
        (*this).m_dTwv.setQuaternion(Eigen::Quaterniond(
            msg.pose.transform.rotation.w,
            msg.pose.transform.rotation.x,
            msg.pose.transform.rotation.y,
            msg.pose.transform.rotation.z));

        for( unsigned int i=0; i<(*this).m_vWheelStates.size(); i++ )
        {
            (*this).m_vWheelStates[i].translation() = Eigen::Vector3d(
                msg.wheel_poses[i].transform.translation.x,
                msg.wheel_poses[i].transform.translation.y,
                msg.wheel_poses[i].transform.translation.z);
            (*this).m_vWheelStates[i].setQuaternion(Eigen::Quaterniond(
                msg.wheel_poses[i].transform.rotation.w,
                msg.wheel_poses[i].transform.rotation.x,
                msg.wheel_poses[i].transform.rotation.y,
                msg.wheel_poses[i].transform.rotation.z));
        }

        for(uint i=0; i<m_vWheelContacts.size(); i++)
        {
            m_vWheelContacts[i] = msg.wheel_contacts[i];
        }

        (*this).m_dV[0] = msg.lin_vel.x;
        (*this).m_dV[1] = msg.lin_vel.y;
        (*this).m_dV[2] = msg.lin_vel.z;
        (*this).m_dW[0] = msg.ang_vel.x;
        (*this).m_dW[1] = msg.ang_vel.y;
        (*this).m_dW[2] = msg.ang_vel.z;

        (*this).m_dCurvature = msg.curvature;

        (*this).m_dSteering = msg.steering;
    }


    Sophus::SE3d m_dTwv;                     //< 4x4 matrix denoting the state of the car
    std::vector<Sophus::SE3d> m_vWheelStates;   //< 4x4 matrices which denote the pose of each wheel
    std::vector<bool> m_vWheelContacts;         //< Angular velocity of the vehicle in world coordinates

    Eigen::Vector3d m_dV;                       //< Linear velocity of the vehicle in world coordinates
    Eigen::Vector3d m_dW;                       //< Angular velocity of the vehicle in world coordinates

    double m_dCurvature;                        //< The curvature at this point in the path
    double m_dSteering;                         //< The steering command given to the car at this point (used to reset rate limitations)
    double m_dTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class BulletVehicleState
{
public:

    BulletVehicleState() {}
    ~BulletVehicleState() {}

    void LoadState(RaycastVehicle *pVehicle)
    {
        //copy back the data
        *pVehicle = m_pVehicleBuffer;
        memcpy( (void*)pVehicle->getRigidBody(), m_pChassisBuffer,sizeof(RaycastVehicle));
    }

    void SaveState(RaycastVehicle *pVehicle)
    {
        //make a backup of the vhicle
        m_pVehicleBuffer = *pVehicle;
        memcpy(m_pChassisBuffer, (void*)pVehicle->getRigidBody(),sizeof(btRigidBody));
    }

private:
    unsigned char m_pChassisBuffer[sizeof(btRigidBody)];
    RaycastVehicle m_pVehicleBuffer;
};

struct BulletWorldInstance : public boost::mutex
{
    BulletWorldInstance()
    {
        m_pCarChassis = NULL;
        m_pVehicle = NULL;
        m_pTerrainShape = NULL;
        m_pHeightfieldData = NULL;
        m_dTime = -1;
        m_bParametersChanged = false;
        m_dTotalCommandTime = 0;
        m_bEnableGUI = false;
    }

    ~BulletWorldInstance()
    {
        if( m_pTerrainShape != NULL) {
            delete m_pTerrainShape;
        }
    }

    void enableGUI(bool do_enable_gui)
    {
      m_bEnableGUI = do_enable_gui;
    }

    std::vector<Sophus::SE3d> m_vWheelTransforms;
    double m_dTime;

    bool m_bEnableGUI;

    btScalar *m_pHeightfieldData;
    btCollisionShape *m_pTerrainShape;
    btRigidBody *m_pTerrainBody;
    RaycastVehicle::btVehicleTuning	m_Tuning;
    btVehicleRaycaster*	m_pVehicleRayCaster;
    RaycastVehicle*	m_pVehicle;
    btCollisionShape* m_pVehicleChassisShape;

    btAlignedObjectArray<btCollisionShape*> m_vCollisionShapes;
    btAlignedObjectArray<btCollisionShape*> m_vVehicleCollisionShapes;
    class btDefaultCollisionConfiguration* m_pCollisionConfiguration;
    class btCollisionDispatcher*	m_pDispatcher;

    class btBroadphaseInterface*	m_pOverlappingPairCache;
    class btConstraintSolver*	m_pConstraintSolver;
    class btDiscreteDynamicsWorld* m_pDynamicsWorld;

    btRigidBody* m_pCarChassis;
    // GLDebugDrawer	m_DebugDrawer;
    BulletVehicleState m_vehicleBackup;
    VehicleState m_state;

    CommandList m_lPreviousCommands;    //< List holding the previous commands sent to the model (Newest commands at the front, oldest at the back)
    double m_dTotalCommandTime;

    //static parameters
    CarParameterMap m_Parameters;

    //CarParameters m_Parameters;
    //std::vector<RegressionParameter> m_vLearningParameters;

    int m_nIndex;
    bool m_bParametersChanged;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ControlSample
{
public:
    double m_dSpeed;
    double m_dSteering;
    double m_Dt;
};

struct MotionSample
{
    std::vector<VehicleState> m_vStates;
    std::vector<ControlCommand> m_vCommands;

    carplanner_msgs::MotionSample toROS()
    {
        carplanner_msgs::MotionSample sample_msg;
        for(uint i=0; i<m_vStates.size(); i++) {
            sample_msg.states.push_back(m_vStates[i].toROS());
        }
        for(uint i=0; i<m_vCommands.size(); i++) {
            sample_msg.commands.push_back(m_vCommands[i].toROS());
        }
        return sample_msg;
    }

    void fromROS(carplanner_msgs::MotionSample sample_msg)
    {
        (*this).m_vStates.clear();
        (*this).m_vStates.resize(sample_msg.states.size());
        for(uint i=0; i<sample_msg.states.size(); i++) {
            (*this).m_vStates[i].fromROS(sample_msg.states[i]);
        }

        (*this).m_vCommands.clear();
        (*this).m_vCommands.resize(sample_msg.commands.size());
        for(uint i=0; i<sample_msg.commands.size(); i++) {
            (*this).m_vCommands[i].fromROS(sample_msg.commands[i]);
        }

        return;
    }

    double MaxCommandCurvature() const
    {
        double dMax = 0;//DBL_MIN;
        for(size_t ii = 0 ; ii < m_vCommands.size() ; ii++){
            //dMax = std::max(dMax,m_vCommands[ii].m_dCurvature);
            dMax += m_vCommands[ii].m_dCurvature;
        }
        return dMax;
    }

    CommandList GetDelayedCommandList(const double& delay, const int& nMotionStartIndex)
    {
        CommandList prevCommands;
        double totalDelay = delay;
        for(int kk = nMotionStartIndex ; kk >= 0 && totalDelay > 0 ; kk--) {
            prevCommands.push_back(m_vCommands[kk]);
            totalDelay -= m_vCommands[kk].m_dT;
        }
        return prevCommands;
    }

    const VehicleState& GetLastPose() const { return m_vStates.back(); }

    std::vector<Sophus::SE3d> GetMotionSample() const
    {
        std::vector<Sophus::SE3d> vPoses;
        vPoses.reserve(m_vStates.size());
        for(const VehicleState& state : m_vStates){
            vPoses.push_back(state.m_dTwv);
        }
        return vPoses;
    }

    double GetBadnessCost() const
    {
        double cost = 0;

        // if(m_vCommands.size() != 0){
        //     const ControlCommand* pPrevCommand = &m_vCommands.front();
        //     for(size_t ii = 1; ii < m_vStates.size() ; ii++){
        //         //const VehicleState& state = m_vStates[ii];
        //         const ControlCommand& command = m_vCommands[ii];
        //         //cost = std::max(state.m_dV.norm() * state.m_dW.norm(),cost);
        //         //cost += fabs(state.m_dV.norm() * state.m_dW[2]) - fabs(m_vCommands[ii].m_dCurvature);
        //         cost += fabs(command.m_dPhi - pPrevCommand->m_dPhi);
        //         pPrevCommand = &m_vCommands[ii];
        //         //cost += fabs(state.m_dSteering);
        //     }
        //     cost /= GetDistance();
        // }

        for(size_t ii = 1; ii < m_vStates.size() ; ii++){
            const VehicleState& state = m_vStates[ii];
            // cost += fabs(state.m_dW[0]*state.m_dW[1]);
            cost += fabs(state.m_dW[0]);
        }
        cost /= GetDistance();

        return cost;
    }

    double GetDistance() const
    {
        double dist = 0;
        if(m_vStates.empty() == false){
            Eigen::Vector3d lastPos = m_vStates[0].m_dTwv.translation();
            for(const VehicleState& state : m_vStates){
                dist += (state.m_dTwv.translation()-lastPos).norm();
                lastPos = state.m_dTwv.translation();
            }
        }
        return dist;
    }

    ////////////////////////////////////////////////////////////////
    static bool FixSampleIndexOverflow(const std::vector<MotionSample>& segmentSamples, int& segmentIndex, int& sampleIndex, bool loop = true)
    {
        bool overFlow = false;
        bool underFlow = false;

        //if this index is beyond the bounds, we move to the next segment
        while(sampleIndex >= (int)segmentSamples[segmentIndex].m_vStates.size()) {
            sampleIndex -= (int)segmentSamples[segmentIndex].m_vStates.size();
            segmentIndex++;

            //if we have reached the end of the segments, we can loop back
            if(segmentIndex >= (int)segmentSamples.size()) {
                if(loop){
                    //loop around
                    segmentIndex = 0;
                }else{
                    //do not loop around
                    segmentIndex = segmentSamples.size()-1;
                    sampleIndex = (int)segmentSamples[segmentIndex].m_vStates.size();
                }
            }
            overFlow = true;
        }

        while(sampleIndex < 0) {
            segmentIndex--;

            //if we have reached the beginning of the segments, we can loop back
            if(segmentIndex < 0) {
                segmentIndex = segmentSamples.size()-1;
            }

            sampleIndex += (int)segmentSamples[segmentIndex].m_vStates.size();
            underFlow = true;
        }

        return overFlow || underFlow;
    }

    void Clear(){
        m_vStates.clear();
        m_vCommands.clear();
    }
};



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ControlPlan
{
public:

    double m_dStartTime;
    double m_dEndTime;
    double m_dNorm;
    MotionSample m_Sample;

    Sophus::SE3d m_dStartPose;
    Sophus::SE3d m_dEndPose;

    int m_nStartSegmentIndex;   //the segment at which this plan starts
    int m_nStartSampleIndex; //the sample in the segment at which this control plan starts

    int m_nEndSegmentIndex;   //the segment at which this plan ends
    int m_nEndSampleIndex; //the sample in the segment at which this control plan ends
    int m_nPlanId;


    VehicleState m_StartState;
    Eigen::Vector3d m_dStartTorques;
    VehicleState m_GoalState;

    void Clear() {
        m_Sample.Clear();
    }

    ~ControlPlan(){
        //DLOG(INFO) << "Deleting control plan.";
    }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MochaVehicle
{
public:
    struct Config{
        std::string params_file="/home/mike/code/mochagui/learning_params.csv",
            terrain_mesh_file="/home/mike/code/mochagui/labLoop.ply",
            car_mesh_file="/home/mike/code/mochagui/herbie/herbie.blend",
            wheel_mesh_file="/home/mike/code/mochagui/herbie/wheel.blend";
        enum Mode{ Simulation=0, Experiment=1 } mode=Mode::Simulation;
    } m_config;

    MochaVehicle(ros::NodeHandle&, ros::NodeHandle&);
    ~MochaVehicle();

    ros::NodeHandle* m_private_nh;
    ros::NodeHandle* m_nh;
    // tf::TransformBroadcaster m_tfbr;
    tf::TransformListener m_tflistener;
    // tf2_ros::Buffer m_tfbuffer;
    // tf2_ros::TransformListener m_tflistener(m_tfbuffer);
    ros::Publisher m_statePub;
    ros::Publisher m_terrainMeshPub;
    ros::Publisher m_chassisMeshPub;
    ros::Subscriber m_terrainMeshSub;
    bool reset_mesh_frame;
    // ros::ServiceServer m_resetmeshSrv;
    // ros::Subscriber m_meshSub2;
    // std::string m_meshSubTopic = "/infinitam/mesh";
    // std::string m_meshSubTopic = "/fake_mesh_publisher/mesh";
    // ros::Publisher m_posePub;
    // ros::Subscriber m_commandSub;

    // GUIHelperInterface* m_guiHelper;

    void InitROS();
    void _PublisherFunc();
    // void _StatePublisherFunc();
    void _TerrainMeshPublisherFunc();
    // void _pubState();
    // void _pubState(VehicleState&);
    void _pubMesh();
    void _pubMesh(btCollisionShape*, ros::Publisher*);
    void _pubMesh(btCollisionShape*, btTransform*, ros::Publisher*);
    // void _meshCB(const mesh_msgs::TriangleMeshStamped::ConstPtr&);
    // bool ResetMesh(carplanner_msgs::ResetMesh::Request&, carplanner_msgs::ResetMesh::Response&);
    // void _PoseThreadFunc();
    // void _CommandThreadFunc(const carplanner_msgs::Command::ConstPtr&);

    void ApplyVelocities(const VehicleState& startingState,
                                              std::vector<ControlCommand>& vCommands,
                                              std::vector<VehicleState>& vStatesOut,
                                              const int iMotionStart,
                                              const int iMotionEnd,
                                              const int nWorldId,
                                              const bool bNoCompensation /*= false (bUsingBestSolution)*/,
                                              const CommandList *pPreviousCommands /*= NULL*/);

    VehicleState ApplyVelocities(const VehicleState& startState,
                                                        MotionSample& sample,
                                                        int nWorldId /*= 0*/,
                                                        bool noCompensation /*= false*/);

    void RaycastService(const carplanner_msgs::RaycastGoalConstPtr&);
    actionlib::SimpleActionServer<carplanner_msgs::RaycastAction> m_actionRaycast_server;

    // void CreateServerService(const carplanner_msgs::CreateServerGoalConstPtr&);
    // actionlib::SimpleActionServer<carplanner_msgs::CreateServerAction> m_actionCreateSimpleServer_server;

    // std::vector<actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction>> servers_;

    // void ApplyVelocities(carplanner_msgs::ApplyVelocitiesGoalConstPtr&);
    void ApplyVelocitiesService(const carplanner_msgs::ApplyVelocitiesGoalConstPtr&);
    // actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server0;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server1;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server2;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server3;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server4;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server5;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server6;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server7;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server8;
    actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> m_actionApplyVelocities_server9;

    // void UpdateStateService(const carplanner_msgs::UpdateStateGoalConstPtr&);
    // actionlib::SimpleActionServer<carplanner_msgs::UpdateStateAction>* m_actionUpdateState_server;

    // void GetGravityCompensationService(const carplanner_msgs::GetGravityCompensationGoalConstPtr&);
    // actionlib::SimpleActionServer<carplanner_msgs::GetGravityCompensationAction>* m_actionGetGravityCompensation_server;

    void GetControlDelayService(const carplanner_msgs::GetControlDelayGoalConstPtr&);
    actionlib::SimpleActionServer<carplanner_msgs::GetControlDelayAction>* m_actionGetControlDelay_server;

    void GetInertiaTensorService(const carplanner_msgs::GetInertiaTensorGoalConstPtr&);
    actionlib::SimpleActionServer<carplanner_msgs::GetInertiaTensorAction> m_actionGetInertiaTensor_server;

    void SetNoDelayService(const carplanner_msgs::SetNoDelayGoalConstPtr&);
    actionlib::SimpleActionServer<carplanner_msgs::SetNoDelayAction> m_actionSetNoDelay_server;

    void meshCb(const mesh_msgs::TriangleMeshStamped::ConstPtr&);

    void replaceMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm);
    void appendMesh(uint worldId, btCollisionShape* meshShape, tf::StampedTransform& Twm);

    static btVector3 GetUpVector(int upAxis,btScalar regularValue,btScalar upValue);
    /////////////////////////////////////////////////////////////////////////////////////////
    static void GenerateStaticHull(const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh , btVector3& dMin, btVector3& dMax);
    void Init(btCollisionShape *pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, CarParameterMap &parameters, unsigned int numWorlds, bool real=false );
    void Init(const struct aiScene *pAIScene,CarParameterMap& parameters, unsigned int numWorlds, bool real=false );
    void Init();
    void DebugDrawWorld(int worldId);

    std::pair<double, double> GetSteeringRequiredAndMaxForce(const int nWorldId, const int nWheelId, const double dPhi, const double dt);
    double GetTotalGravityForce(BulletWorldInstance* pWorld);
    double GetTotalWheelFriction(int worldId, double dt);
    double _CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt);
    /////////////////////////////////////////////////////////////////////////////////////////
    void UpdateState(const int &worldId,
                     const ControlCommand command,
                     const double forceDt = -1,
                     const bool bNoDelay = false,
                     const bool bNoUpdate  = false );
    /////////////////////////////////////////////////////////////////////////////////////////
    virtual void GetVehicleState(int worldId, VehicleState &stateOut);
    //virtual VehicleState GetVehicleStateAsync(int worldId);
    Eigen::Vector3d GetVehicleLinearVelocity(int worldId);
    Eigen::Vector3d GetVehicleAngularVelocity(int worldId);
    Eigen::Vector3d GetVehicleInertiaTensor(int worldId);
    virtual void SetState(int nWorldId,  const VehicleState& state );
    //virtual void SetState( int worldId,  const Eigen::Matrix4d& vState  );
    virtual void SetStateNoReset(BulletWorldInstance *pWorld, const Sophus::SE3d &Twv );
    BulletWorldInstance *GetWorldInstance(int id){ return m_vWorlds[id]; }
    const BulletWorldInstance *GetWorldInstance(int id) const { return m_vWorlds[id]; }
    const int GetWorldCount() const { return m_vWorlds.size(); }

    double GetCorrectedSteering(double& dCurvature, int index);
    double  GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/);
    std::vector<Sophus::SE3d> GetWheelTransforms(const int worldIndex);

    //getter functions
    Eigen::Vector3d     GetGravity() { return m_dGravity; }
    //HeightMap*          GetHeightMap() { return m_pHeightMap; }
    void                GetCommandHistory(int worldId,CommandList &previousCommandsOut);
    void                ResetCommandHistory(int worldId);
    CommandList&        GetCommandHistoryRef(int worldId);
    void                PushDelayedControl(int worldId, ControlCommand& delayedCommands);
    CarParameterMap& GetParameters(int index) { return GetWorldInstance(index)->m_Parameters; }
    const CarParameterMap& GetParameters(int index) const  { return GetWorldInstance(index)->m_Parameters; }
    void                SetCommandHistory(const int &worldId, const CommandList &previousCommands);
    bool RayCast(const Eigen::Vector3d& dSource, const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool &biDirectional, int index = 0);

    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams, BulletWorldInstance *pWorld);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index);
    void _InternalUpdateParameters(BulletWorldInstance* pWorld);

    unsigned int GetNumWorlds(){ return m_nNumWorlds; }

    CommandList& GetPreviousCommand() { return m_lPreviousCommands; }
    void SetPreviousCommands(const CommandList& list) { m_lPreviousCommands = list;}
    void ResetPreviousCommands() { return m_lPreviousCommands.clear(); }
    bool SetNoDelay(bool bNoDelay){ return (m_bNoDelay = bNoDelay); }

protected:
    // MochaVehicle *m_pCarModel;
    Eigen::Vector3d m_dInitTorques;
    CommandList m_lPreviousCommands;
    bool m_bNoDelay;

    void _GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands);
    btRigidBody* _LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask);
    // btRigidBody* _LocalAddRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape);
    void _LocalRemoveRigidBody(BulletWorldInstance *, btCollisionShape* );
    void _InitVehicle(BulletWorldInstance* pWorld, CarParameterMap& parameters);
    void _InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh);

    std::vector<BulletWorldInstance*> m_vWorlds;
    //HeightMap *m_pHeightMap;
    // boost::thread* m_pPoseThread;
    // boost::thread* m_pCommandThread;
    boost::thread* m_pPublisherThread;
    // boost::thread* m_pStatePublisherThread;
    boost::thread* m_pTerrainMeshPublisherThread;

    Eigen::Vector3d m_dGravity;
    unsigned int m_nNumWorlds;

    static int GetNumWorldsRequired(const int nOptParams) { return nOptParams*2+2; }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


#endif	/* MOCHAVEHICLE_H */
