#include "RCTrack.h" 
#include <iostream>
#include <fstream>

enum SplineType
{
   CATMULL_ROM = 0,
   BASIS,
   SPLINE_TYPE_COUNT
};

extern SplineType type;
extern float smax;
extern float t;
RCTrack::RCTrack() {
   
}


float getAngle(Eigen::Vector3f a, Eigen::Vector3f b ) {
   std::cout << a.cross(b) << std::endl;
   float cr = (a.cross(b)).norm();
   float angle = asin(cr / (a.norm() * b.norm()));
   return angle;
}

Eigen::Matrix4f RCTrack::getRotation(float u, Eigen::Matrix4f B, Eigen::MatrixXf G, int k) {

   Eigen::Vector3f T, p_1d;

   Eigen::Vector4f u_1d;
   u_1d << 0, 1, 2 * u, 3 * u * u;
   p_1d = G * B * u_1d;
   T = p_1d /p_1d.norm();


   Eigen::Vector3f Bi, p_2d;

   Eigen::Vector4f u_2d;
   
   if(curve.cps[k]->flippedNormal) {
      p_2d = Eigen::Vector3f(0.0f,-1.0f,0.0f);
   }
   else {
      p_2d = Eigen::Vector3f(0.0f,1.0f,0.0f);
   }
   Bi = p_1d.cross(p_2d) / p_1d.cross(p_2d).norm();
  
   Eigen::Vector3f N = Bi.cross(T);
   
   T.normalize();
   Bi.normalize();
   N.normalize();

   Eigen::Matrix4f mat;
   mat = Eigen::Matrix4f::Identity();
   mat.block<3,1>(0,0) = T;
   mat.block<3,1>(0,1) = N;
   mat.block<3,1>(0,2) = Bi;

   return mat;
}

// mutex so that it isn't running while new tracks are being loaded in
bool isLoading = false;
bool isDrawing = false;
void RCTrack::draw(MatrixStack *P,MatrixStack *MV, Program *prog) {
   if(isLoading)
      return;
   isDrawing = true;
   curve.draw(P,MV,prog);

   int ncps = (int)curve.cps.size();
   if(ncps >= 5) {
      
      int trackTotal = (int)(smax/1.8); // curve discretization

      Eigen::MatrixXf G(3,ncps);
      for(int i = 0; i < ncps; ++i) {
         G.block<3,1>(0,i) = curve.cps[i]->pos;
      }

      for(int trackIndex = 0; trackIndex < trackTotal; ++trackIndex) {
         Eigen::MatrixXf Gk(3,4);
         Eigen::Matrix4f B = (type == CATMULL_ROM ? curve.Bcr : curve.Bb);      
      
         float tNorm = fmod(trackIndex,trackTotal) / trackTotal;
         float sNorm = tNorm;
         float s = smax * sNorm;
         float u = curve.s2u(s);
         float kfloat;
         u = std::modf(u, &kfloat);
         int k = (int)std::floor(kfloat);
         
         

         Gk = G.block<3,4>(0,k);
        
         Eigen::Vector4f uVec(1.0f, u, u*u, u*u*u);
         Eigen::Vector3f Point = Gk * B * uVec;

         Eigen::Vector3f color;
         
         glUniform3fv(prog->getUniform("kd"),  1, color.data());
         color =  Eigen::Vector3f(0.8,0.2,0.2);

         if(trackIndex % 10 == 1) {
            float offset = 0.0;
            MV->pushMatrix();
            MV->translate(Point);
            while((Point(1) - offset) > 0.0) {
               MV->translate(Eigen::Vector3f(0.0f, -1.0f, 0.0));
               supportModel.draw(MV);
               offset += 1.0f;
            }
            MV->popMatrix();
         }

         MV->pushMatrix();

         
         if(k + 2== curve.highlightedPoint) {
            color = (1 - u *u) * Eigen::Vector3f(0.8,0.2,0.2) + u * u * Eigen::Vector3f(0.2,0.2,0.8) ;
            glUniform3fv(prog->getUniform("kd"),  1, color.data());
         }
         else if(k + 1 == curve.highlightedPoint) {
            color = u * Eigen::Vector3f(0.8,0.2,0.2) + (1 - u) * Eigen::Vector3f(0.2,0.2,0.8) ;
            glUniform3fv(prog->getUniform("kd"),  1, color.data());
         }
         

         Eigen::Matrix4f rot = getRotation(u, B,G.block<3,4>(0,k), k);
         rot =  rot * curve.getExtraRot(k,u);
         MV->translate(Point);
         MV->multMatrix(rot);
         trackModel.draw(MV);

         MV->popMatrix();
      }
   }
   isDrawing = false;
}
extern float tmax ;

Eigen::Vector3f v(6.0, 0.0, 0.0);
float x = 0.0;
float prevypos = 10.0;
void RCTrack::useRCViewMatrix(MatrixStack *MV, Camera camera) {
   if(isLoading)
      return;
   isDrawing = true;
   int ncps = (int)curve.cps.size();
   if(ncps >= 5) {

      Eigen::Matrix4f B = (type == CATMULL_ROM ? curve.Bcr : curve.Bb);      
      Eigen::MatrixXf G(3,ncps);
      Eigen::MatrixXf Gk(3,4);



      for(int i = 0; i < ncps; ++i) {
         G.block<3,1>(0,i) = curve.cps[i]->pos;
      }

      

      float tNorm = fmod(x,smax) / smax;
      float sNorm = tNorm;
      float s = smax * sNorm;
      float u = curve.s2u(s);
      float kfloat;
      u = std::modf(u, &kfloat);
      int k = (int)std::floor(kfloat);

      Gk = G.block<3,4>(0,k);
     
      // Compute spline point at u
      Eigen::Vector4f uVec(1.0f, u, u*u, u*u*u);
      Eigen::Vector3f Point = Gk * B * uVec;

      MV->rotate(camera.getRot()(1), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
      MV->rotate(camera.getRot()(0), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
      MV->translate(Eigen::Vector3f(0.0f,-2.0f,0.0f));
      MV->multMatrix((getRotation(u, B,G.block<3,4>(0,k), k) * curve.getExtraRot(k,u)).inverse());
      MV->translate(-Point);
   }
   isDrawing = false;
}


void updateCarPosition(float u, Eigen::MatrixXf G, Eigen::Matrix4f B) {

   Eigen::Vector3f T, p_1d, Y;
   Eigen::Vector4f u_1d;
   float m = 100.0f, h = .15;

   u_1d << 0, 1, 2 * u, 3 * u * u;
   p_1d = G * B * u_1d;
   T = p_1d /p_1d.norm();
   Y = Eigen::Vector3f(0.0f,1.0f,0.0f);
   float angle = (T.dot(Y)) / (T.norm() * Y.norm());
   //gets the force of gravity that acts in the tangent of the car
   Eigen::Vector3f fgtan =  m * angle * Eigen::Vector3f(-9.8f,0.0f,0.0f);

   //fgtan is also the normal force between the track and the car
   float mu = .01;
   Eigen::Vector3f ffric = -0.3f * fgtan ;

   Eigen::Vector3f fbrake(0.0f,0.0f,0.0f);
   if(smax - fmod(x, smax) < 25.0f) {   
      fbrake = m * Eigen::Vector3f(-8.0f, 0.0, 0.0);
   }

   v = v + h * (1/m)*fgtan + h * (1/m) * ffric + h * (1/m) * fbrake; 

   if(v(0) < 6.0f) {
      v(0) = 6.0f;
   }  
}

void RCTrack::TravelAlong(MatrixStack *MV, ShapeObj *toDraw) {
   if(isLoading)
      return;
   isDrawing = true;
   int ncps = (int)curve.cps.size();
   if(ncps >= 5) {
      

      Eigen::MatrixXf G(3,ncps);
      for(int i = 0; i < ncps; ++i) {
         G.block<3,1>(0,i) = curve.cps[i]->pos;
            
      }

      float h = .01;
      x = x + h * v(0);

      Eigen::MatrixXf Gk(3,4);
      Eigen::Matrix4f B = (type == CATMULL_ROM ? curve.Bcr : curve.Bb);      
   
      float tNorm = fmod(x,smax) / smax;
      float sNorm = tNorm;
      float s = smax * sNorm;
      float u = curve.s2u(s);
      float kfloat;
      u = std::modf(u, &kfloat);
      int k = (int)std::floor(kfloat);
      
      

      Gk = G.block<3,4>(0,k);
     
      // Compute spline point at u
      Eigen::Vector4f uVec(1.0f, u, u*u, u*u*u);
      Eigen::Vector3f Point = Gk * B * uVec;

      Eigen::MatrixXf Gr(4,ncps);
      
      
     // MV->translate(Eigen::Vector3f(0.0f,0.5f,0.0f));
      MV->pushMatrix();
      
      MV->translate(Point);
      
      MV->multMatrix(getRotation(u, B,G.block<3,4>(0,k), k));
      MV->multMatrix(curve.getExtraRot(k,u));
      MV->translate(Eigen::Vector3f(0.0f,.5f,0.0f));
      
      
      toDraw->draw(MV);
      MV->popMatrix();

      updateCarPosition(u, G.block<3,4>(0,k), B);
    //  MV->multMatrix(getRotation(u, B,G.block<3,4>(0,k), k));
      
   }
   isDrawing = false;
}

void RCTrack::toFile(char *fileName) {
   std::ofstream myFile;
   myFile.open(fileName);
   int ncps = (int)curve.cps.size();
   myFile << ncps << std::endl;
   for(int i = 0; i < ncps; i++) {
      myFile << curve.cps[i]->pos(0) << " " <<
                curve.cps[i]->pos(1) << " " << curve.cps[i]->pos(2) << " " <<
                curve.cps[i]->flippedNormal << " " << 
                curve.cps[i]->extraRot << std::endl;
   }

   myFile.close();
}


void RCTrack::loadFile(char *fileName) {
   std::ifstream readFrom; 

   readFrom.open(fileName);
   isLoading = true;
   while(isDrawing) {

   }
   if(readFrom.is_open()) {
      char buffer[100];
      readFrom >> buffer;      
      int ncps = std::atoi(buffer);
      curve.cps.clear();
      for(int i = 0; i < ncps; i++) {
         curve.cps.push_back(new CpsNode());
         readFrom >> curve.cps[i]->pos(0);
         readFrom >> curve.cps[i]->pos(1);
         readFrom >> curve.cps[i]->pos(2);
         readFrom >> curve.cps[i]->flippedNormal;
         readFrom >> curve.cps[i]->extraRot;
      }
      curve.buildTable();
   }
   else {
      std::cout << "File not Found"  << std::endl;
   }
   isLoading = false;
   readFrom.close();
}
