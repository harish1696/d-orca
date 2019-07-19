#ifndef GCOP_PQPMESH_H
#define GCOP_PQPMESH_H

#include "PQP.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <tf/LinearMath/Transform.h>

/**
 * Load a STL Mesh into PQP Model
 * Currently Uses tf but can be replaced by Eigen vectors
*/



class PqpMesh{
  public:
    /** Constructor with STL file name. Creates an internal PQP Model 
     * @param filename File name for loading stl model
     * @param scale Scale for scaling vertices of the mesh
     */
    PqpMesh(const char *filename, const double *scale_ = 0);
    /** Default constructor with  model being two triangles forming the head of an arrow
      * @param cr   collision radius
     */
    PqpMesh(double cr);
    /** Default Constructor does not do anything
    */
    PqpMesh()
    {
    }
    /** Get current state of mesh as a transform (origin and rotation)
    */
    tf::Transform GetState();
    virtual ~PqpMesh();

    PQP_Model *pm;     ///< Model
    PQP_REAL pt[3];    ///< Origin
    PQP_REAL pR[3][3]; ///< Rotation Matrix
    double scale[3];   ///< Scale of STL file
};

PqpMesh::PqpMesh(double cr)
{
  pm = new PQP_Model;

  PQP_REAL p1[3], p2[3], p3[3], p4[3];

  pm->BeginModel();
  p1[0] = 0; p1[1] = cr; p1[2] = 0;
  p2[0] = 0; p2[1] = -cr; p2[2] = 0;
  p3[0] = cr; p3[1] = 0; p3[2] = 0;
  pm->AddTri(p1, p2, p3, 0);  
  p1[0] = 0; p1[1] = 0; p1[2] = cr;
  p2[0] = 0; p2[1] = 0; p2[2] = -cr;
  p3[0] = cr; p3[1] = 0; p3[2] = 0;
  pm->AddTri(p1, p2, p3, 1);
  //#TODO Do we need more triangles which are opposite to these triangles in coordinatespace??
  pm->EndModel();
  pm->MemUsage(1);


  //Set Initial mesh origin (0,0,0) and Rotation as Identity
  pt[0] = pt[1] = pt[2] = 0;
  pR[0][0] = pR[1][1] = pR[2][2] = 1.0;
  pR[0][1] = pR[1][0] = pR[2][0] = 0.0;
  pR[0][2] = pR[1][2] = pR[2][1] = 0.0;

  scale[0] = scale[1] = scale[2] = 0;
}
  
PqpMesh::PqpMesh(const char *filename, const double *scale_)
{
  pm = new PQP_Model;

  if(scale_ != 0)
  {
    memcpy(scale,scale_,3*sizeof(double));
  }
  else
  {
    scale[0] = scale[1] = scale[2] = 1.0;//Set scale 1
  }
  pm->BeginModel();

  //Loading File:
  FILE* file = fopen(filename,"rb");
  if(file)
  {
    int size=0;
    if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
    {
      printf("Error: Cannot access file to determine size of %s\n", filename);
    } 
    else
    {
      if (size)
      {
        printf("Open STL file of %d bytes\n",size);
        char* memoryBuffer = new char[size+1];
        int actualBytesRead = fread(memoryBuffer,1,size,file);
        if (actualBytesRead!=size)
        {
          printf("Error reading from file %s",filename);
        } 
        else
        {
          int numTriangles = *(int*)&memoryBuffer[80];

          if (numTriangles)
          {
            {
              //perform a sanity check instead of crashing on invalid triangles/STL files
              int expectedBinaryFileSize = numTriangles* 50 + 84;
              if (expectedBinaryFileSize != size)
              {
                return;
              }

            }

            //Looping through the triangles and setting the triangles in PQP
            {
              int i;
              float *vert_temp = (float *)malloc(3*3*numTriangles*sizeof(float));
              PQP_REAL p1[3], p2[3], p3[3];//Points of Triangle in PQP Mesh
              //tf::Vector3 edge1, edge2, edge3, unit_normal;

#pragma omp parallel for private(i)
              for (i=0;i<numTriangles;i++)
              {
                memcpy(&vert_temp[9*i],&memoryBuffer[96+i*50],36);//9 floats of 4 bytes each (3 loats per vertex)
                //Cast Vertices into PQP Reals
                p1[0] = scale[0]*vert_temp[9*i];   p1[1] = scale[1]*vert_temp[9*i+1]; p1[2] = scale[2]*vert_temp[9*i+2];
                p2[0] = scale[0]*vert_temp[9*i+3]; p2[1] = scale[1]*vert_temp[9*i+4]; p2[2] = scale[2]*vert_temp[9*i+5];
                p3[0] = scale[0]*vert_temp[9*i+6]; p3[1] = scale[1]*vert_temp[9*i+7]; p3[2] = scale[2]*vert_temp[9*i+8];
                //Add Triangle to PQP
                pm->AddTri(p1, p2, p3, i);
                /*
                // Add Normal to Model:
                edge1.setX(p2[0] - p1[0]); edge1.setY(p2[1] - p1[1]); edge1.setZ(p2[2] - p1[2]);
                edge2.setX(p3[0] - p2[0]); edge2.setY(p3[1] - p2[1]); edge2.setZ(p3[2] - p2[2]);
                //edge3.setX(p1[0] - p3[0]); edge3.setY(p1[1] - p3[1]); edge3.setZ(p1[2] - p3[2]);
                unit_normal = (edge1.cross(edge2)).normalized();
                normals.push_back(unit_normal);
                 */
                //#DEBUG 
                /*printf("Normal: %f,%f,%f\n",unit_normal.x(),unit_normal.y(),unit_normal.z());
                printf("Vertices1: %f,%f,%f\n",p1[0],p1[1],p1[2]);
                printf("Vertices2: %f,%f,%f\n",p2[0],p2[1],p2[2]);
                printf("Vertices3: %f,%f,%f\n",p3[0],p3[1],p3[2]);
                */
              }
            }
          }
          delete[] memoryBuffer;
        }
      }
      fclose(file);
    }
  }

  pm->EndModel();
  pm->MemUsage(1);

  pt[0] = pt[1] = pt[2] = 0;
  pR[0][0] = pR[1][1] = pR[2][2] = 1.0;
  pR[0][1] = pR[1][0] = pR[2][0] = 0.0;
  pR[0][2] = pR[1][2] = pR[2][1] = 0.0;
}

tf::Transform PqpMesh::GetState()
{
  tf::Transform t;
  t.setOrigin(tf::Vector3(pt[0],pt[1],pt[2]));

  tf::Matrix3x3 rotation;
  rotation[0][0] = pR[0][0]; rotation[0][1] = pR[0][1]; rotation[0][2] = pR[0][2];
  rotation[1][0] = pR[1][0]; rotation[1][1] = pR[1][1]; rotation[1][2] = pR[0][2];
  rotation[2][0] = pR[2][0]; rotation[2][1] = pR[2][1]; rotation[2][2] = pR[2][2];

  t.setBasis(rotation);

  return t;
}

PqpMesh::~PqpMesh()
{
  delete pm;
}

  

#endif

