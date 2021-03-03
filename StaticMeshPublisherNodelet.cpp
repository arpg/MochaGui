#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <glog/logging.h>

#include <carplanner_msgs/mesh_conversion_tools.hpp>
#include <tf/transform_listener.h>

#include "MochaVehicle.h"


namespace mochapc {

/* StaticMeshPublisherNodelet
 * Nodelet that periodically publishes provided mesh 
 * to test memory sharing. 
 */
class StaticMeshPublisherNodelet : public nodelet::Nodelet
{
public:

    virtual inline void onInit()
    {
        NODELET_INFO("Initializing...\n");

        ros::NodeHandle& nh = this->getMTNodeHandle();
        ros::NodeHandle& private_nh = this->getMTPrivateNodeHandle();

        private_nh.param("terrain_mesh_file", m_meshFile, m_meshFile);
        private_nh.param("terrain_mesh_topic", m_meshTopic, m_meshTopic);
        private_nh.param("publish_timeout", m_timeout_s, m_timeout_s);
        m_meshPub = nh.advertise<mesh_msgs::TriangleMeshStamped>(m_meshTopic, 1);

        m_scene.reset(aiImportFile( m_meshFile.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals ));
        if ( !m_scene )
        {
            NODELET_INFO("Invalid scene. Quitting...\n");
            return;
        }

        ROS_INFO_NAMED("[StatiMeshPublisherNodelet]", "Initialized, ready to publish to %s", m_meshTopic.c_str());
        m_mesh = m_scene->mMeshes[m_scene->mRootNode->mMeshes[0]];
        m_loopTimer = nh.createTimer(ros::Duration(m_timeout_s), [ this ](const ros::TimerEvent &time) {
            mesh_msgs::TriangleMeshStampedPtr meshStampedMsg(new mesh_msgs::TriangleMeshStamped);
            
            mesh_msgs::TriangleMesh* conversion_ptr;
            convertAssimpMeshToMeshMsg(m_mesh, &conversion_ptr);

            meshStampedMsg->header.stamp = ros::Time::now();
            meshStampedMsg->header.frame_id = "mesh";

            meshStampedMsg->mesh = *conversion_ptr;
            DLOG(INFO) << "Publishing Mesh at " << meshStampedMsg.get();
            
            NODELET_INFO("Publishing mesh with %d faces on %s\n", meshStampedMsg->mesh.triangles.size(), m_meshTopic.c_str());
            m_meshPub.publish(meshStampedMsg);

            delete conversion_ptr;
        });
    }

private:
    ros::Timer m_loopTimer;

    ros::Publisher m_meshPub;
    std::string m_meshFile{ ros::package::getPath("mochapc") + "/labLoop.ply" };
    std::string m_meshTopic{ "vehicle/input_terrain_mesh" };
    float m_timeout_s{ 1.0f };

    boost::shared_ptr<const aiScene> m_scene;
    aiMesh* m_mesh;
};

} // namespace mochapc

PLUGINLIB_EXPORT_CLASS(mochapc::StaticMeshPublisherNodelet, nodelet::Nodelet);
