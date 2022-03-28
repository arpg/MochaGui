#ifndef _MESH_CONVERSION_TOOLS
#define	_MESH_CONVERSION_TOOLS

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <carplanner_msgs/TriangleMeshStamped.h>

#include <Eigen/Eigen>

#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>

#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"

#include "tf/tfMessage.h"
#include "tf/tf.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void CollisionShape2TriangleMesh(btCollisionShape* collisionShape, const btTransform& parentTransform, btAlignedObjectArray<btVector3>* vertexPositions, btAlignedObjectArray<btVector3>* vertexNormals, btAlignedObjectArray<int>* indicesOut)
{
// CollisionShape2TriangleMesh(mesh, parentTransform, vertexPositions, vertexNormals, indicesOut);
  switch (collisionShape->getShapeType())
  {
    case SOFTBODY_SHAPE_PROXYTYPE:
    {
      // printf("%s SOFTBODY_SHAPE_PROXYTYPE\n", "[CollisionShape2TriangleMsg]");
      //skip the soft body collision shape for now
      break;
    }
    case STATIC_PLANE_PROXYTYPE:
    {
      // printf("%s STATIC_PLANE_PROXYTYPE\n", "[CollisionShape2TriangleMsg]");
      //draw a box, oriented along the plane normal
      const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(collisionShape);
      btScalar planeConst = staticPlaneShape->getPlaneConstant();
      const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
      btVector3 planeOrigin = planeNormal * planeConst;
      btVector3 vec0, vec1;
      btPlaneSpace1(planeNormal, vec0, vec1);
      btScalar vecLen = 100.f;
      btVector3 verts[4];

      verts[0] = planeOrigin + vec0 * vecLen + vec1 * vecLen;
      verts[1] = planeOrigin - vec0 * vecLen + vec1 * vecLen;
      verts[2] = planeOrigin - vec0 * vecLen - vec1 * vecLen;
      verts[3] = planeOrigin + vec0 * vecLen - vec1 * vecLen;

      int startIndex = vertexPositions->size();
      indicesOut->push_back(startIndex + 0);
      indicesOut->push_back(startIndex + 1);
      indicesOut->push_back(startIndex + 2);
      indicesOut->push_back(startIndex + 0);
      indicesOut->push_back(startIndex + 2);
      indicesOut->push_back(startIndex + 3);

      btVector3 triNormal = parentTransform.getBasis() * planeNormal;

      for (int i = 0; i < 4; i++)
      {
        btVector3 vtxPos;
        btVector3 pos = parentTransform * verts[i];
        vertexPositions->push_back(pos);
        vertexNormals->push_back(triNormal);
      }
      break;
    }
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
    {
      // printf("%s TRIANGLE_MESH_SHAPE_PROXYTYPE\n", "[CollisionShape2TriangleMsg]");
      btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*)collisionShape;
      btVector3 trimeshScaling = trimesh->getLocalScaling();
      btStridingMeshInterface* meshInterface = trimesh->getMeshInterface();
      btAlignedObjectArray<btVector3> vertices;
      btAlignedObjectArray<int> indices;

      for (int partId = 0; partId < meshInterface->getNumSubParts(); partId++)
      {
        const unsigned char* vertexbase = 0;
        int numverts = 0;
        PHY_ScalarType type = PHY_INTEGER;
        int stride = 0;
        const unsigned char* indexbase = 0;
        int indexstride = 0;
        int numfaces = 0;
        PHY_ScalarType indicestype = PHY_INTEGER;
        //PHY_ScalarType indexType=0;

        btVector3 triangleVerts[3];
        meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts, type, stride, &indexbase, indexstride, numfaces, indicestype, partId); // *vertexbase is all zeros in gpu mode
        btVector3 aabbMin, aabbMax;

        // printf("%s %d faces\n", "[CollisionShape2TriangleMesh]", numfaces);

        for (int triangleIndex = 0; triangleIndex < numfaces; triangleIndex++)
        {
          unsigned int* gfxbase = (unsigned int*)(indexbase + triangleIndex * indexstride);

          for (int j = 2; j >= 0; j--)
          {
            int graphicsindex = indicestype == PHY_SHORT ? ((unsigned short*)gfxbase)[j] : gfxbase[j];
            if (type == PHY_FLOAT)
            {
              float* graphicsbase = (float*)(vertexbase + graphicsindex * stride);
              triangleVerts[j] = btVector3(
                graphicsbase[0] * trimeshScaling.getX(),
                graphicsbase[1] * trimeshScaling.getY(),
                graphicsbase[2] * trimeshScaling.getZ());
            }
            else
            {
              double* graphicsbase = (double*)(vertexbase + graphicsindex * stride);
              triangleVerts[j] = btVector3(btScalar(graphicsbase[0] * trimeshScaling.getX()),
                             btScalar(graphicsbase[1] * trimeshScaling.getY()),
                             btScalar(graphicsbase[2] * trimeshScaling.getZ()));
            }
          }
          usleep(10);
          indices.push_back(vertices.size());
          vertices.push_back(triangleVerts[0]);
          indices.push_back(vertices.size());
          vertices.push_back(triangleVerts[1]);
          indices.push_back(vertices.size());
          vertices.push_back(triangleVerts[2]);

          btVector3 triNormal = (triangleVerts[1] - triangleVerts[0]).cross(triangleVerts[2] - triangleVerts[0]);
          btScalar dot = triNormal.dot(triNormal);

          // printf("%d indices before cull\n", indices.size());

          //cull degenerate triangles
          if (dot >= SIMD_EPSILON * SIMD_EPSILON)
          {
            triNormal /= btSqrt(dot);
            for (int v = 0; v < 3; v++)
            {
              btVector3 pos = parentTransform * triangleVerts[v];
              indicesOut->push_back(vertexPositions->size());
              vertexPositions->push_back(pos);
              vertexNormals->push_back(triNormal);
            }
          }

          // printf("%d indices after cull\n", indicesOut->size());
        }
      }
      // uint num_ind = indicesOut->size(); 
      // printf("%d indices after convert\n", num_ind);
      break;
    }
    default:
    {
      // printf("%s Unknown Shape ProxyType\n", "[CollisionShape2TriangleMsg]");
      if (collisionShape->isConvex())
      {
        btConvexShape* convex = (btConvexShape*)collisionShape;
        {
          const btConvexPolyhedron* pol = 0;
          if (convex->isPolyhedral())
          {
            btPolyhedralConvexShape* poly = (btPolyhedralConvexShape*)convex;
            pol = poly->getConvexPolyhedron();
          }

          if (pol)
          {
            for (int v = 0; v < pol->m_vertices.size(); v++)
            {
              vertexPositions->push_back(pol->m_vertices[v]);
              btVector3 norm = pol->m_vertices[v];
              norm.safeNormalize();
              vertexNormals->push_back(norm);
            }
            for (int f = 0; f < pol->m_faces.size(); f++)
            {
              for (int ii = 2; ii < pol->m_faces[f].m_indices.size(); ii++)
              {
                indicesOut->push_back(pol->m_faces[f].m_indices[0]);
                indicesOut->push_back(pol->m_faces[f].m_indices[ii - 1]);
                indicesOut->push_back(pol->m_faces[f].m_indices[ii]);
              }
            }
          }
          else
          {
            btShapeHull* hull = new btShapeHull(convex);
            hull->buildHull(0.0, 1);

            {
              //int strideInBytes = 9*sizeof(float);
              //int numVertices = hull->numVertices();
              //int numIndices =hull->numIndices();

              for (int t = 0; t < hull->numTriangles(); t++)
              {
                btVector3 triNormal;

                int index0 = hull->getIndexPointer()[t * 3 + 0];
                int index1 = hull->getIndexPointer()[t * 3 + 1];
                int index2 = hull->getIndexPointer()[t * 3 + 2];
                btVector3 pos0 = parentTransform * hull->getVertexPointer()[index0];
                btVector3 pos1 = parentTransform * hull->getVertexPointer()[index1];
                btVector3 pos2 = parentTransform * hull->getVertexPointer()[index2];
                triNormal = (pos1 - pos0).cross(pos2 - pos0);
                triNormal.safeNormalize();

                for (int v = 0; v < 3; v++)
                {
                  int index = hull->getIndexPointer()[t * 3 + v];
                  btVector3 pos = parentTransform * hull->getVertexPointer()[index];
                  indicesOut->push_back(vertexPositions->size());
                  vertexPositions->push_back(pos);
                  vertexNormals->push_back(triNormal);
                }
              }
            }
            delete hull;
          }
        }
      }
      else
      {
        if (collisionShape->isCompound())
        {
          btCompoundShape* compound = (btCompoundShape*)collisionShape;
          for (int i = 0; i < compound->getNumChildShapes(); i++)
          {
            btTransform childWorldTrans = parentTransform * compound->getChildTransform(i);
            CollisionShape2TriangleMesh(compound->getChildShape(i), childWorldTrans, vertexPositions, vertexNormals, indicesOut);
          }
        }
        else
        {
          if (collisionShape->getShapeType() == SDF_SHAPE_PROXYTYPE)
          {
            //not yet
          }
          else
          {
            btAssert(0);
          }
        }
      }
    }
  };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, const btTransform* parentTransform, mesh_msgs::TriangleMesh** meshMsg)
{
  // const btTransform parentTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  btAlignedObjectArray<btVector3> vertexPositions;
  btAlignedObjectArray<btVector3> vertexNormals;
  btAlignedObjectArray<int> indicesOut;

  // printf("%s about to pub mesh with %d faces\n", "[convertCollisionShape2MeshMsg]", dynamic_cast<btTriangleMesh*>(dynamic_cast<btBvhTriangleMeshShape*>(collisionShape)->getMeshInterface())->getNumTriangles());

  CollisionShape2TriangleMesh(collisionShape, *parentTransform, &vertexPositions, &vertexNormals, &indicesOut);

  // printf("%s %d indices, %d vertices\n", "[convertCollisionShape2MeshMsg]", indicesOut.size(), vertexPositions.size());

  uint numVerts = indicesOut.size();
  (*meshMsg)->vertices = std::vector<geometry_msgs::Point>(numVerts);
  for( uint iv=0; iv<numVerts; iv++)
  {
    btVector3 vertexPosition = vertexPositions[iv];
    geometry_msgs::Point sVertex;
    sVertex.x = vertexPosition[0];
    sVertex.y = vertexPosition[1];
    sVertex.z = vertexPosition[2];
    (*meshMsg)->vertices[iv] = sVertex;
  }

  uint numTris = indicesOut.size()/3;
  (*meshMsg)->triangles = std::vector<mesh_msgs::TriangleIndices>(numTris);
  for( uint it=0; it<numTris; it++)
  {
    for( uint ii=0; ii<3; ii++)
    {
      (*meshMsg)->triangles[it].vertex_indices[ii] = indicesOut[it*3+ii];
    }
  }

  // printf("%s %d faces\n", "[convertCollisionShape2MeshMsg]", (*meshMsg)->triangles.size());

  return;
}

inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, mesh_msgs::TriangleMesh** meshMsg)
{
  const btTransform* parentTransform = new btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0));
  convertCollisionShape2MeshMsg(collisionShape, parentTransform, meshMsg);
  return;
}

inline void convertCollisionShape2MeshMsg(btCollisionShape* collisionShape, mesh_msgs::TriangleMeshStamped** mesh_stamped)
{
  mesh_msgs::TriangleMesh* mesh = new mesh_msgs::TriangleMesh();
  convertCollisionShape2MeshMsg(collisionShape, &mesh);
  (*mesh_stamped)->mesh = *mesh;
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertMeshMsg2CollisionShape(const mesh_msgs::TriangleMesh* meshMsg, btCollisionShape*& collisionShape)
{
  btTriangleMesh* triangleMesh = new btTriangleMesh();
  for( uint it=0; it<meshMsg->triangles.size(); it++)
  {
    // geometry_msgs::Point vertex0 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]]; // works
    // geometry_msgs::Point vertex1 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]];
    // geometry_msgs::Point vertex2 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]];

    // triangleMesh->addTriangle(btVector3(vertex0.x, vertex0.y, vertex0.z),
    //                           btVector3(vertex1.x, vertex1.y, vertex1.z),
    //                           btVector3(vertex2.x, vertex2.y, vertex2.z),
    //                           it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);

    const btScalar* v0x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].x)); // works
    const btScalar* v0y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].y));
    const btScalar* v0z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].z));
    const btScalar* v1x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].x));
    const btScalar* v1y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].y));
    const btScalar* v1z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].z));
    const btScalar* v2x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].x));
    const btScalar* v2y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].y));
    const btScalar* v2z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].z));

    triangleMesh->addTriangle(btVector3( *v0x, *v0y, *v0z ),
                              btVector3( *v1x, *v1y, *v1z ),
                              btVector3( *v2x, *v2y, *v2z ),
                              it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);
  }

  collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);

  return;
}

// inline void convertMeshMsg2CollisionShape(mesh_msgs::TriangleMesh* meshMsg, btCollisionShape*& collisionShape)
// {
//   btTriangleMesh* triangleMesh = new btTriangleMesh();
//   triangleMesh->preallocateVertices(meshMsg->vertices.size());
//   triangleMesh->preallocateIndices(meshMsg->triangles.size()*3);
//   for( uint it=0; it<meshMsg->triangles.size(); it++)
//   {
//     geometry_msgs::Point vertex0 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]];
//     geometry_msgs::Point vertex1 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]];
//     geometry_msgs::Point vertex2 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]];

//     triangleMesh->addTriangle(btVector3(vertex0.x, vertex0.y, vertex0.z),
//                               btVector3(vertex1.x, vertex1.y, vertex1.z),
//                               btVector3(vertex2.x, vertex2.y, vertex2.z),
//                               it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);

//     // const btScalar* v0x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].x)); 
//     // const btScalar* v0y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].y));
//     // const btScalar* v0z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]].z));
//     // const btScalar* v1x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].x));
//     // const btScalar* v1y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].y));
//     // const btScalar* v1z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]].z));
//     // const btScalar* v2x(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].x));
//     // const btScalar* v2y(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].y));
//     // const btScalar* v2z(&(meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]].z));

//     // triangleMesh->addTriangle(btVector3( *v0x, *v0y, *v0z ),
//     //                           btVector3( *v1x, *v1y, *v1z ),
//     //                           btVector3( *v2x, *v2y, *v2z ),
//     //                           it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);
//   }

//   collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);

//   return;
// }

// inline void convertMeshMsg2CollisionShape(mesh_msgs::TriangleMeshStamped* meshMsg, btCollisionShape*& collisionShape)
// {
//   mesh_msgs::TriangleMesh* mesh = &(meshMsg->mesh);
//   convertMeshMsg2CollisionShape(mesh, collisionShape);
//   return;
// }

inline void convertMeshMsg2CollisionShape(const mesh_msgs::TriangleMesh::ConstPtr& meshMsg, btCollisionShape*& collisionShape)
{
  btTriangleMesh* triangleMesh = new btTriangleMesh();
  for( uint it=0; it<meshMsg->triangles.size(); it++)
  {
    geometry_msgs::Point vertex0 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[0]];
    geometry_msgs::Point vertex1 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[1]];
    geometry_msgs::Point vertex2 = meshMsg->vertices[meshMsg->triangles[it].vertex_indices[2]];

    triangleMesh->addTriangle(btVector3(vertex0.x, vertex0.y, vertex0.z),
                              btVector3(vertex1.x, vertex1.y, vertex1.z),
                              btVector3(vertex2.x, vertex2.y, vertex2.z),
                              it==meshMsg->triangles.size()-1 /*last triangle*/ ? true : false);
  }

  collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);

  return;
}

inline void convertMeshMsg2CollisionShape(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg, btCollisionShape*& collisionShape)
{
  mesh_msgs::TriangleMesh::ConstPtr mesh = mesh_msgs::TriangleMesh::ConstPtr(&(meshMsg->mesh));
  convertMeshMsg2CollisionShape(mesh, collisionShape);
  return;
}

///////////////////////////////////////////////////////////////////////////////////////

inline void convertMeshMsg2CollisionShape_Shared(const mesh_msgs::TriangleMesh& meshMsg, btCollisionShape*& collisionShape)
{
  btAlignedObjectArray<btVector3>* triangleVertices = new btAlignedObjectArray<btVector3>();
  btAlignedObjectArray<unsigned int>* triangleIndices = new btAlignedObjectArray<unsigned int>();

  // Reserve memory and expand container to desired size
  triangleVertices->resizeNoInitialize(meshMsg.vertices.size());
  triangleIndices->resizeNoInitialize(meshMsg.triangles.size() * 3);

  for (unsigned int i = 0; i < meshMsg.vertices.size(); ++i) {
    btVector3 vertex(meshMsg.vertices[i].x, meshMsg.vertices[i].y, meshMsg.vertices[i].z);
    triangleVertices->at(i) = vertex;
  }

  for (unsigned int i = 0; i < meshMsg.triangles.size(); ++i) {
    triangleIndices->at(i * 3) = meshMsg.triangles[i].vertex_indices[0];
    triangleIndices->at(i * 3 + 1) = meshMsg.triangles[i].vertex_indices[1];
    triangleIndices->at(i * 3 + 2) = meshMsg.triangles[i].vertex_indices[2];
  }


  btTriangleIndexVertexArray* triangleMesh = new btTriangleIndexVertexArray(meshMsg.triangles.size(), (int *)&triangleIndices->at(0), 3 * sizeof(int), meshMsg.vertices.size(), (btScalar *)&triangleVertices->at(0), sizeof(btVector3));
  collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);

  return;
}

inline void convertMeshMsg2CollisionShape_Shared(const mesh_msgs::TriangleMeshStamped::ConstPtr& meshMsg, btCollisionShape*& collisionShape)
{
  convertMeshMsg2CollisionShape_Shared(meshMsg->mesh, collisionShape);
  return;
}

inline void convertMeshMsg2CollisionShape_Shared(const carplanner_msgs::TriangleMeshStamped::ConstPtr& meshMsg, btCollisionShape*& collisionShape)
{  
	btIndexedMesh mesh;

	mesh.m_numTriangles = meshMsg->triangleIndices.size() / 3;
	mesh.m_triangleIndexBase = (const unsigned char*)meshMsg->triangleIndices.data();
	mesh.m_triangleIndexStride = sizeof(int) * 3;
	mesh.m_numVertices = meshMsg->triangleVertices.size() / 3;
	mesh.m_vertexBase = (const unsigned char*)meshMsg->triangleVertices.data();
	mesh.m_vertexStride = sizeof(float) * 3;
  mesh.m_vertexType = PHY_FLOAT;

  btTriangleIndexVertexArray* triangleMesh = new btTriangleIndexVertexArray();
	triangleMesh->addIndexedMesh(mesh);

  collisionShape = new btBvhTriangleMeshShape(triangleMesh,true,true);
}

inline std::vector<std::pair<uint32_t, uint32_t>> getMeshBorder(const btTriangleIndexVertexArray* mesh) 
{
  const btIndexedMesh& rawMesh = mesh->getIndexedMeshArray()[0];
  
  // Border vertices have at least one outgoing edge that does not belong to two triangles. 
  // Count the number of triangles that use each edge
  std::map<std::pair<uint32_t, uint32_t>, int> edgeCounts;
  for (unsigned i = 0; i < rawMesh.m_numTriangles; ++i) {
    const uint32_t* currentTriangle = (uint32_t *)(rawMesh.m_triangleIndexBase + (rawMesh.m_triangleIndexStride * i));
    uint32_t indexOne = *currentTriangle;
    uint32_t indexTwo = *(currentTriangle + sizeof(uint32_t));
    uint32_t indexThree = *(currentTriangle + 2 * sizeof(uint32_t));
    // Sort indexOne indexTwo and indexThree for consistent ordering in the edge list
    if (indexOne > indexTwo) std::swap(indexOne, indexTwo);
    if (indexTwo > indexThree) std::swap(indexTwo, indexThree);
    if (indexOne > indexTwo) std::swap(indexOne, indexTwo);

    const auto edgeOne = std::make_pair<const uint32_t&, const uint32_t&>(indexOne, indexTwo);
    const auto edgeTwo = std::make_pair<const uint32_t&, const uint32_t&>(indexOne, indexThree);
    const auto edgeThree = std::make_pair<const uint32_t&, const uint32_t&>(indexTwo, indexThree);

    edgeCounts[edgeOne] += 1;
    edgeCounts[edgeTwo] += 1;
    edgeCounts[edgeThree] += 1;
  }

  std::vector<std::pair<uint32_t, uint32_t>> borderEdges;
  for (const auto& [edge, count]: edgeCounts) {
    if (count == 1) {
        borderEdges.push_back(edge);
    }
  }

  return borderEdges;
}

inline void mergeMeshes(const btTriangleIndexVertexArray* oldMesh, const btTriangleIndexVertexArray* newMesh)
{
  const std::vector<std::pair<uint32_t, uint32_t>> newBorder = getMeshBorder(newMesh);

  const auto pointInsideBorder = [](const std::vector<std::pair<uint32_t, uint32_t>>& border, const btTriangleIndexVertexArray* oldMesh, float x, float y) {
    const btIndexedMesh& rawMesh = oldMesh->getIndexedMeshArray()[0];
    // Draw a line in the +x direction from the (x, y) point, counting the number of edges it crosses. 
    // The point is inside the border polygon iff it crosses an odd number of edges
    int crosses = 0;
    const float tol = 1e-5;
    for (const auto& [indexOne, indexTwo]: border) {
      float* vertexBaseOne = (float *)(rawMesh.m_vertexBase + rawMesh.m_vertexStride * indexOne);
      float* vertexBaseTwo = (float *)(rawMesh.m_vertexBase + rawMesh.m_vertexStride * indexTwo);

      float x0 = *vertexBaseOne;
      float y0 = *(vertexBaseOne + sizeof(float));

      float x1 = *vertexBaseTwo;
      float y1 = *(vertexBaseTwo + sizeof(float));

      // If the line is horizontal, there is no intersection
      if (std::abs(y1 - y0) > tol) {
        float t = (y - y0) / (y1 - y0);
        // Line is between ymin and ymax
        if (t >= 0 && t <= 1) {
          float xt = x0 + (x1 - x0) * t;
          // Line intersects with edge
          if (xt >= x) {
            crosses += 1;
          }
        }
      }
    }
    return crosses % 2 == 1;
  };

  const auto deleteInsideBorder = [&pointInsideBorder](const std::vector<std::pair<uint32_t, uint32_t>>& border, 
                                     btTriangleIndexVertexArray* oldMesh, 
                                     const btTriangleIndexVertexArray* newMesh) {
    btIndexedMesh& rawMesh = oldMesh->getIndexedMeshArray()[0];
    int previousTriangles = rawMesh.m_numTriangles; 

    // Consider getting vertex counts so we know when to delete vertices. 
    for (unsigned i = 0; i < rawMesh.m_numTriangles; ++i) {
      uint32_t* currentTriangle = (uint32_t *)(rawMesh.m_triangleIndexBase + (rawMesh.m_triangleIndexStride * i));
      const uint32_t indexOne = *currentTriangle;
      const uint32_t indexTwo = *(currentTriangle + sizeof(uint32_t));
      const uint32_t indexThree = *(currentTriangle + 2 * sizeof(uint32_t));

      std::vector<float*> vertexBases; 
      vertexBases.push_back((float *)(rawMesh.m_vertexBase + rawMesh.m_vertexStride * indexOne));
      vertexBases.push_back((float *)(rawMesh.m_vertexBase + rawMesh.m_vertexStride * indexTwo));
      vertexBases.push_back((float *)(rawMesh.m_vertexBase + rawMesh.m_vertexStride * indexThree));
      // if any vertex is inside the x-y projection of the mesh border polygon, remove the triangle
      for (const auto& vertexBase: vertexBases) {
        float x = *vertexBase;
        float y = *(vertexBase + sizeof(float));

        if (pointInsideBorder(border, oldMesh, x, y)) {
          // Implementation: swap with last element and repeat this iteration. 
          // Removes the need to copy most triangles we're keeping
          uint32_t* lastTriangleBase = (uint32_t *)(rawMesh.m_triangleIndexBase + rawMesh.m_triangleIndexStride * rawMesh.m_numTriangles);
          std::swap(*lastTriangleBase, *currentTriangle);

          uint32_t* current = (uint32_t *)(currentTriangle + sizeof(uint32_t));
          uint32_t* last = (uint32_t *)(lastTriangleBase + sizeof(uint32_t));
          std::swap(*current, *last);

          current = (uint32_t *)(currentTriangle + 2 * sizeof(uint32_t));
          last = (uint32_t *)(lastTriangleBase + 2 * sizeof(uint32_t));
          std::swap(*current, *last);
          rawMesh.m_numTriangles -= 1;
          --i;
          break;
        }
      }
    }
  };
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertAssimpMeshToMeshMsg(aiMesh* sMesh, mesh_msgs::TriangleMesh** mesh)
{
  mesh_msgs::TriangleMesh* dMesh = new mesh_msgs::TriangleMesh();

  // printf("converting aiMesh w %d faces ",sMesh->mNumFaces);

  // copy vertices
  dMesh->vertices = std::vector<geometry_msgs::Point>(sMesh->mNumVertices);
  for( uint iv=0; iv<sMesh->mNumVertices; iv++)
  {
    aiVector3D aVertex = sMesh->mVertices[iv];
    geometry_msgs::Point rVertex;
    rVertex.x = aVertex[0];
    rVertex.y = aVertex[1];
    rVertex.z = aVertex[2];
    dMesh->vertices[iv] = rVertex;
  }

  // copy faces
  dMesh->triangles = std::vector<mesh_msgs::TriangleIndices>(sMesh->mNumFaces);
  for( uint it=0; it<sMesh->mNumFaces; it++)
  {
    aiFace* aFace = &(sMesh->mFaces[it]);
    mesh_msgs::TriangleIndices* rFace = &(dMesh->triangles[it]);
    for( uint ii=0; ii<aFace->mNumIndices; ii++)
    {
      rFace->vertex_indices[ii] = aFace->mIndices[ii];
    }
  }

  (*mesh) = dMesh;
  // printf("to mesh_msg w %d faces\n",(*mesh)->triangles.size());
}

inline void convertAssimpMeshToMeshMsg(aiMesh* sMesh, mesh_msgs::TriangleMeshStamped** mesh)
{
  mesh_msgs::TriangleMesh* tmpmesh = &((*mesh)->mesh);
  convertAssimpMeshToMeshMsg(sMesh, &tmpmesh);
  // printf("convert to mesh_stamped w %d faces",(*mesh)->mesh.triangles.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMesh*& sMesh, aiMesh*& mesh)
{
  aiMesh* dMesh = new aiMesh();

  // dMesh->mVertices = std::vector<aiVector3D>(sMesh->vertices.size());
  std::vector<aiVector3D> vVertices(sMesh->vertices.size());
  dMesh->mVertices = &(vVertices[0]);
  for( uint iv=0; iv<sMesh->vertices.size(); iv++)
  {
    geometry_msgs::Point sVertex = sMesh->vertices[iv];
    dMesh->mVertices[iv] = aiVector3D(sVertex.x, sVertex.y, sVertex.z);
  }

  std::vector<aiFace> vFaces(sMesh->triangles.size());
  dMesh->mFaces = &(vFaces[0]);
  for( uint it=0; it<sMesh->triangles.size(); it++)
  {
    mesh_msgs::TriangleIndices* sFace = &(sMesh->triangles[it]);
    aiFace* dFace = &(dMesh->mFaces[it]);
    for( uint ii=0; ii<sFace->vertex_indices.size(); ii++)
    {
      dFace->mIndices[ii] = sFace->vertex_indices[ii];
    }
  }

  mesh = dMesh;
}

inline void convertMeshMsgToAssimpMesh(mesh_msgs::TriangleMeshStamped*& sMesh, aiMesh*& mesh)
{
  mesh_msgs::TriangleMesh* tmpmesh = &(sMesh->mesh);
  convertMeshMsgToAssimpMesh(tmpmesh, mesh);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertRPY2Quat(const Eigen::Vector3d& rpy, Eigen::Quaterniond* quat)
{
  *quat = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

inline void convertRPY2Quat(const Eigen::Vector3d& rpy, Eigen::Vector4d* quat_xyzw)
{
  Eigen::Quaterniond quat = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
                          * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
  *quat_xyzw << quat.x(), quat.y(), quat.z(), quat.w();
}

//////////////////////////////////////////////////////////////////////////////////////////////

// inline std::string print_tf_rpy(const tf::Transform& t)
// { 
//   tf::Quaternion q = t.getRotation();
//   double r, p, y;
//   tf::Matrix3x3(q).getRPY(r,p,y);
//   return sprintf("%.2f %.2f %.2f %.2f %.2f %.2f", 
//     t.getOrigin().getX(), t.getOrigin().getY(), t.getOrigin().getZ(), r, p, y);
// }

#endif
