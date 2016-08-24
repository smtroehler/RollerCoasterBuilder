#include "SplineCurve.h"
#include "ShapeObj.h"
#include "Camera.h"
class RCTrack {


public:
   RCTrack();

   void draw(MatrixStack *P,MatrixStack *MV, Program *prog);
   ShapeObj trackModel;
   ShapeObj supportModel;
   SplineCurve curve;
   void useRCViewMatrix(MatrixStack *MV, Camera camera);
   void TravelAlong(MatrixStack *MV, ShapeObj *toDraw);
   void toFile(char *fileName);
   void loadFile(char *fileName);
private:
   Eigen::Matrix4f getRotation(float u, Eigen::Matrix4f B, Eigen::MatrixXf G, int k);
   Eigen::Quaternionf getTangentQuat(int k);
   
};