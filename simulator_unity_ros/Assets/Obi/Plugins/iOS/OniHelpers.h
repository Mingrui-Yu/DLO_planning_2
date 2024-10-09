/*
 *  OniHelpers.h
 *  Oni
 *
 *  Created by José María Méndez González on 21/9/15.
 *  Copyright (c) 2015 ArK. All rights reserved.
 *
 */

#ifndef OniHelpers_
#define OniHelpers_

#include "Dense"
#include "HalfEdgeMesh.h"

#if defined(__APPLE__) || defined(ANDROID) || defined(__linux__)
    #define EXPORT __attribute__((visibility("default")))
#else
    #define EXPORT __declspec(dllexport)
#endif

namespace Oni
{
    extern "C"
    {
        
		EXPORT int MakePhase(int group, int flags);
        EXPORT int GetGroupFromPhase(int phase);
        
        /**
         * Calculates the rest bend factor for a bending constraint between 3 particles.
         * @param coordinates an array of 9 floats: x,y,z of the first particle, x,y,z of the second particle, x,y,z of the third (central) particle.
         */
		EXPORT float BendingConstraintRest(float* coordinates);
        
		EXPORT HalfEdgeMesh* CreateHalfEdgeMesh();
        
		EXPORT void DestroyHalfEdgeMesh(HalfEdgeMesh* mesh);
		EXPORT void GetHalfEdgeMeshInfo(HalfEdgeMesh* mesh, HalfEdgeMesh::MeshInformation* mesh_info);
        
        EXPORT void CalculatePrimitiveCounts(HalfEdgeMesh* mesh,
                                             Eigen::Vector3f* vertices,
                                             int* triangles,
                                             int vertex_count,
                                             int triangle_count);
        
		EXPORT void Generate(HalfEdgeMesh* mesh,
                             Eigen::Vector3f* vertices,
                             int* triangles,
                             int vertex_count,
                             int triangle_count,
                             float* scale);
        
		EXPORT void SetHalfEdges(HalfEdgeMesh* mesh,HalfEdgeMesh::HalfEdge* half_edges, int count);
		EXPORT void SetVertices(HalfEdgeMesh* mesh,HalfEdgeMesh::Vertex* vertices, int count);
		EXPORT void SetFaces(HalfEdgeMesh* mesh,HalfEdgeMesh::Face* faces, int count);
        
        EXPORT void SetNormals(HalfEdgeMesh* mesh,Eigen::Vector3f* normals);
        EXPORT void SetTangents(HalfEdgeMesh* mesh,Vector4fUnaligned* tangents);
        EXPORT void SetInverseOrientations(HalfEdgeMesh* mesh,QuaternionfUnaligned* orientations);
        EXPORT void SetVisualMap(HalfEdgeMesh* mesh,int* map);
        
		EXPORT int GetHalfEdgeCount(HalfEdgeMesh* mesh);
		EXPORT int GetVertexCount(HalfEdgeMesh* mesh);
		EXPORT int GetFaceCount(HalfEdgeMesh* mesh);
        
        EXPORT void GetPointCloudAnisotropy(Eigen::Vector3f* points,int count,float max_anisotropy,float radius, const Eigen::Vector3f& hint_normal, Eigen::Vector3f& centroid, QuaternionfUnaligned& orientation,Eigen::Vector3f& principal_values);
    }
    
}

#endif
