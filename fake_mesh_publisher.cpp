#include <glog/logging.h>
#include <gflags/gflags.h>

#include "/home/mike/code/MochaGui_ros/conversion_tools.h"
#include <tf/transform_listener.h>

DEFINE_string(mesh, "~/code/MochaGui_ros/labLoop.ply", "File for terrain mesh.");

int main( int argc, char* argv[] )
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    if(FLAGS_mesh.empty()) {
        LOG(FATAL) << "Mesh of environment not provided. Use parameter -mesh";
        return -1;
    }
    else
    {
        printf("%sLoading mesh: %s\n","[fake_mesh_publisher] ",FLAGS_mesh.c_str());
    }

    ros::init(argc, argv, "fake_mesh_publisher");

    ros::NodeHandle nh;
    ros::Publisher mesh_pub = nh.advertise<mesh_msgs::TriangleMeshStamped>("/infinitam/mesh",1);

    const aiScene *pScene = aiImportFile( FLAGS_mesh.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    if( !pScene )
    {
      printf("%sInvalid scene. Quitting...\n","[fake_mesh_publisher] ");
      return 0;
    }

    aiMesh* aMesh = pScene->mMeshes[pScene->mRootNode->mMeshes[0]];

    printf("%sInitialized.\n","[fake_mesh_publisher] ");

    while(ros::ok())
    {
      mesh_msgs::TriangleMeshStamped* mesh_stamped = new mesh_msgs::TriangleMeshStamped();
      mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
      // mesh_msgs::TriangleMesh* mesh = &((*mesh_stamped).mesh);
      // mesh = &(mesh_stamped->mesh);
      mesh_stamped->header.stamp = ros::Time::now();
      mesh_stamped->header.frame_id = "map";
      convertAssimpMeshToMeshMsg(aMesh, &mesh);
      // mesh_stamped->mesh);
      mesh_stamped->mesh = *mesh;

      printf("%sPublishing mesh with %d faces.\n","[fake_mesh_publisher] ",mesh_stamped->mesh.triangles.size());
      // printf("%sPublishing mesh with %d faces.\n","[fake_mesh_publisher] ",mesh->triangles.size());
      // printf("%sPublishing mesh with %d faces.\n","[fake_mesh_publisher] ",aMesh->mNumFaces);
      mesh_pub.publish(*mesh_stamped);

      ros::spinOnce();
      ros::Rate(100).sleep();
    }
}
