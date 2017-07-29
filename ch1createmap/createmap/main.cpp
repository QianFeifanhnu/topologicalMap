#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"

using namespace std;
using namespace g2o;
int main()
{
    vector<VertexSE3*> vertices;
    vector<EdgeSE3*> edges;
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double,6,6>::Identity();
    int id = 0;
    //add x0,x1,x2,x3
    //x0=0
    Eigen::AngleAxisd rotz0(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot0 = rotz0.toRotationMatrix();
    Eigen::Isometry3d t0;
    t0 = rot0;
    t0.translation() = Eigen::Vector3d(0, 0, 0);
    VertexSE3* v0 = new VertexSE3;
    v0->setId(id++);
    v0->setEstimate(t0);
    vertices.push_back(v0);
    //x1=0
    Eigen::AngleAxisd rotz1(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot1 = rotz1.toRotationMatrix();
    Eigen::Isometry3d t1;
    t1 = rot1;
    t1.translation() = Eigen::Vector3d(0, 0, 0);
    VertexSE3* v1 = new VertexSE3;
    v1->setId(id++);
    v0->setEstimate(t1);
    vertices.push_back(v1);
    //x2=1
    Eigen::AngleAxisd rotz2(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot2 = rotz2.toRotationMatrix();
    Eigen::Isometry3d t2;
    t2 = rot2;
    t2.translation() = Eigen::Vector3d(0, 0, 1);
    VertexSE3* v2 = new VertexSE3;
    v2->setId(id++);
    v2->setEstimate(t2);
    vertices.push_back(v2);
    //x3=0.2
    Eigen::AngleAxisd rotz3(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rot3 = rotz3.toRotationMatrix();
    Eigen::Isometry3d t3;
    t3 = rot3;
    t3.translation() = Eigen::Vector3d(0, 0, 0.2);
    VertexSE3* v3 = new VertexSE3;
    v3->setId(id++);
    v3->setEstimate(t3);
    vertices.push_back(v3);
    //e01=0
    VertexSE3* prev01 = vertices[0];
    VertexSE3* cur01  = vertices[1];
    Eigen::AngleAxisd r01(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d r01m = r01.toRotationMatrix();
    Eigen::Isometry3d t01;
    t01 = r01m;
    t01.translation() = Eigen::Vector3d(0, 0, 0);
    EdgeSE3* e01 = new EdgeSE3;
    e01->setVertex(0, prev01);
    e01->setVertex(1, cur01);
    e01->setMeasurement(t01);
    e01->setInformation(information);
    edges.push_back(e01);

    //e12=1
    VertexSE3* prev12 = vertices[1];
    VertexSE3* cur12  = vertices[2];
    Eigen::AngleAxisd r12(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d r12m = r12.toRotationMatrix();
    Eigen::Isometry3d t12;
    t12 = r12m;
    t12.translation() = Eigen::Vector3d(0, 0, 1);
    EdgeSE3* e12 = new EdgeSE3;
    e12->setVertex(0, prev12);
    e12->setVertex(1, cur12);
    e12->setMeasurement(t12);
    e12->setInformation(information);
    edges.push_back(e12);

    //e23=-0.8
    VertexSE3* prev23= vertices[2];
    VertexSE3* cur23  = vertices[3];
    Eigen::AngleAxisd r23(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d r23m = r23.toRotationMatrix();
    Eigen::Isometry3d t23;
    t23 = r23m;
    t23.translation() = Eigen::Vector3d(0, 0, -0.8);
    EdgeSE3* e23 = new EdgeSE3;
    e23->setVertex(0, prev23);
    e23->setVertex(1, cur23);
    e23->setMeasurement(t23);
    e23->setInformation(information);
    edges.push_back(e23);

    //e31=0
    VertexSE3* prev31= vertices[3];
    VertexSE3* cur31  = vertices[1];
    Eigen::AngleAxisd r31(0, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d r31m = r31.toRotationMatrix();
    Eigen::Isometry3d t31;
    t31 = r31m;
    t31.translation() = Eigen::Vector3d(0, 0,0);
    EdgeSE3* e31 = new EdgeSE3;
    e31->setVertex(0, prev31);
    e31->setVertex(1, cur31);
    e31->setMeasurement(t31);
    e31->setInformation(information);
    edges.push_back(e31);

      // write output
    ofstream fileOutputStream;
    string outFilename="./ori.g2o";
    cout<<outFilename<<endl;
    fileOutputStream.open(outFilename.c_str());
    //CommandArgs arg;
    //arg.param("o", outFilename, "-", "output filename");
    string vertexTag = Factory::instance()->tag(vertices[0]);
    string edgeTag = Factory::instance()->tag(edges[0]);
    
    //ostream& fout = outFilename != "./out.g2o" ? fileOutputStream : cout;
    ostream& fout=fileOutputStream;
    for (size_t i = 0; i < vertices.size(); ++i) {
    VertexSE3* v = vertices[i];
    fout << vertexTag << " " << v->id() << " ";
    v->write(fout);
    fout << endl;
    }
    for (size_t i = 0; i < edges.size(); ++i) {
    EdgeSE3* e = edges[i];
    VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
    VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
    fout << edgeTag << " " << from->id() << " " << to->id() << " ";
    e->write(fout);
    fout << endl;
    }
    return 0;
}

