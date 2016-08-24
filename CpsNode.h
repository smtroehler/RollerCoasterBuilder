#pragma once
#ifndef _CpsNode_H_
#define _CpsNode_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class CpsNode {
public:
   CpsNode();
   virtual ~CpsNode();
   Eigen::Vector3f pos;
   bool flippedNormal;
   float extraRot;
private:
};

#endif