#pragma once
#ifndef _SplineCurve_H_
#define _SplineCurve_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Program.h"

#include "MatrixStack.h"

#include "CpsNode.h"


class SplineCurve {
public:
   SplineCurve();
   virtual ~SplineCurve();

   void draw(MatrixStack *P,MatrixStack *MV, Program *prog );

   float s2u(float s);
   void buildTable();
   void translatePoint(int index, Eigen::Vector3f trans);
   void addPoint();
   void end();
   
   int getNumCps();
   int getSelectedSegment();

   Eigen::Matrix4f getExtraRot(int k, float u);

   std::vector<std::pair<float,float> > usTable;   
   
   std::vector<CpsNode*> cps;
   std::vector<bool> flippedBi;


   Eigen::Matrix4f Bcr, Bb;

   std::vector<float> extraRot;
   Eigen::Vector3f lastPoint;
   
   int highlightedPoint;
private:


};

#endif