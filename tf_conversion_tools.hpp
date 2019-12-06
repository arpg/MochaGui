#ifndef _TF_CONVERSION_TOOLS
#define	_TF_CONVERSION_TOOLS

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf/tf.h"

// tf::Transform rot_270_x(tf::Quaternion(-0.707,0,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_90_y(tf::Quaternion(0,0.707,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_180_y(tf::Quaternion(0,1,0,0),tf::Vector3(0,0,0));
// tf::Transform rot_270_y(tf::Quaternion(0,-0.707,0,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_90_z(tf::Quaternion(0,0,0.707,0.707),tf::Vector3(0,0,0));
// tf::Transform rot_180_z(tf::Quaternion(0,0,1,0),tf::Vector3(0,0,0));
// tf::Transform rot_270_z(tf::Quaternion(0,0,-0.707,0.707),tf::Vector3(0,0,0));
// tf::Transform link_to_optical(tf::Quaternion(-0.5,0.5,-0.5,0.5),tf::Vector3(0,0,0));

inline void tf2geometry_msg(tf::Transform& tf, geometry_msgs::Transform* gm)
{
	gm->translation.x = tf.getOrigin().getX(); 
	gm->translation.y = tf.getOrigin().getY();
	gm->translation.z = tf.getOrigin().getZ();
	gm->rotation.x = tf.getRotation().getX();
	gm->rotation.y = tf.getRotation().getY();
	gm->rotation.z = tf.getRotation().getZ();
	gm->rotation.w = tf.getRotation().getW();
}

inline void tf2geometry_msg(tf::StampedTransform& tf, geometry_msgs::TransformStamped* gm)
{
	geometry_msgs::Transform gm_temp;
	tf2geometry_msg(tf, &gm_temp);
	gm->transform = gm_temp;
	gm->header.frame_id = tf.frame_id_;
	gm->child_frame_id = tf.child_frame_id_;
	gm->header.stamp = tf.stamp_;
}

inline void geometry_msg2tf(geometry_msgs::Transform& gm, tf::Transform* tf)
{
	tf->setOrigin(tf::Vector3(gm.translation.x, gm.translation.y, gm.translation.z));
	tf->setRotation(tf::Quaternion(gm.rotation.x, gm.rotation.y, gm.rotation.z, gm.rotation.w));
}

inline void geometry_msg2tf(geometry_msgs::TransformStamped& gm, tf::StampedTransform* tf)
{
	tf::Transform tf_temp;
	geometry_msg2tf(gm.transform, &tf_temp);
	tf->setData(tf_temp);
	tf->frame_id_ = gm.header.frame_id;
	tf->child_frame_id_ = gm.child_frame_id;
	tf->stamp_ = gm.header.stamp;
}

#endif
