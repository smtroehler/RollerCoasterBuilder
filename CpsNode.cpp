#include "CpsNode.h"

CpsNode::CpsNode() {
   pos = Eigen::Vector3f(0.0f,0.0f,0.0f);
   flippedNormal = false;
   extraRot = 0;
}

CpsNode::~CpsNode() {
   
}