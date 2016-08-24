#include "SplineCurve.h"

using namespace std;

enum SplineType
{
   CATMULL_ROM = 0,
   BASIS,
   SPLINE_TYPE_COUNT
};

extern SplineType type;
extern bool keyToggles[256];
float tmax = 5.0;
float smax = 0.0;

float randf2PI() {
   return fmod(rand() / 100000.0, 2 *  M_PI);
}
float randf0to1() {
   return fmod(rand() / 100000.0, 1.0);
}




SplineCurve::SplineCurve() {


   Bcr << 0.0f, -1.0f,  2.0f, -1.0f,
         2.0f,  0.0f, -5.0f,  3.0f,
         0.0f,  1.0f,  4.0f, -3.0f,
         0.0f,  0.0f, -1.0f,  1.0f;
   Bcr *= 0.5;
   
   Bb << 1.0f, -3.0f,  3.0f, -1.0f,
        4.0f,  0.0f, -6.0f,  3.0f,
        1.0f,  3.0f,  3.0f, -3.0f,
        0.0f,  0.0f,  0.0f,  1.0f;
   Bb /= 6.0f;

   cps.push_back(new CpsNode());
   cps.push_back(new CpsNode());
   cps.push_back(new CpsNode());
   cps.push_back(new CpsNode());
   translatePoint(0,Eigen::Vector3f(0.0f,10.0f, 0.0f));
   translatePoint(2,Eigen::Vector3f(10.0f,10.0f, 0.0f));

   lastPoint = Eigen::Vector3f(10.0f,10.0f,0.0f);
   buildTable();
}

void SplineCurve::buildTable()
{
   usTable.clear();
   int ncps = (int)cps.size();
   if(ncps >= 4) {
      Eigen::MatrixXf G(3,ncps);
      Eigen::MatrixXf Gk(3,4);
      Eigen::Matrix4f B = (type == CATMULL_ROM ? Bcr : Bb);

      Eigen::Vector3f xi = Eigen::Vector3f(-std::sqrt(3.0/5.0), 0, std::sqrt(3.0/5.0));
      Eigen::Vector3f wi = Eigen::Vector3f(5.0/9.0, 8.0/9.0, 5.0/9.0);
      float s = 0.0;
      for(int i = 0; i < ncps; ++i) {
         G.block<3,1>(0,i) = cps[i]->pos;
      }
      usTable.push_back(make_pair(0, 0));
      for(int k = 0; k < ncps - 3; ++k) {
         int n = 6; // curve discretization
         // Gk is the 3x4 block starting at column k
         Gk = G.block<3,4>(0,k);
         
         for(int i = 1; i < n; ++i) {
            
            float summat = 0.0;
            float ua = (i - 1) / (n - 1.0f);
            float ub = i / (n - 1.0f);

            for(int j = 0; j < 3; j++) {
               float u = (((ub - ua) / 2.0) * xi(j)) + ((ua + ub)  / 2.0);
               // Compute spline point at u
               Eigen::Vector4f uVec(0.0f, 1.0f, 2.0*u, 3.0*u*u);
               Eigen::Vector3f Ppri = Gk * B * uVec;               
               summat += wi(j) * Ppri.norm();

            }
            s += ((ub - ua)/ 2.0) * summat;   
            usTable.push_back(make_pair((i / (n - 1.0f)) + k, s));
         }  
      }
      smax = s;
   }
}

SplineCurve::~SplineCurve() 
{

}

int SplineCurve::getNumCps() {
   return (int)cps.size();

}

int SplineCurve::getSelectedSegment() {
   return highlightedPoint;
}

float SplineCurve::s2u(float s)
{
   float u0, u1, s0, s1, u, alpha, temp;
   pair<float,float> row0, row1;
   int i = -1;
   do {

      row1 = usTable[++i];
      temp = row1.second;
      
      
   } while(s > temp);
   // have to decrease by 1 because check overshoots it except when s is 0
   if(i != 0)
      --i;
   row0 = usTable[i];
   row1 = usTable[i + 1];
   s1 = row1.second;
   s0 = row0.second;
   
   alpha = (s - s0) / (s1 - s0);
   
   u1 = row1.first;
   u0 = row0.first;
   u = (1 - alpha) * u0 + (alpha * u1);


   return u;
}

extern float t;


void SplineCurve::draw(MatrixStack *P,MatrixStack *MV, Program *prog) {

      // Setup the projection matrix
   
   int ncps = (int)cps.size();

   if(keyToggles['o']) {
      prog->unbind();
      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadMatrixf(P->topMatrix().data());
      
      // Setup the modelview matrix
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glLoadMatrixf(MV->topMatrix().data());
      
      // Draw the curve
      
      // Draw the control points
      glPointSize(5.0f);
      glColor3f(0.0f, 1.0f, 0.0f);
      glBegin(GL_POINTS);
      for(int i = 0; i < ncps; ++i) {
         if(highlightedPoint == i) 
            glColor3f(1.0f, 0.0f, 0.0f);
         else
            glColor3f(0.0f, 1.0f, 0.0f);

         glVertex3fv(cps[i]->pos.data());
      }
      glEnd();



      if(ncps >= 4) {
         Eigen::MatrixXf G(3,ncps);
         Eigen::MatrixXf Gk(3,4);
         Eigen::Matrix4f B = (type == CATMULL_ROM ? Bcr : Bb);
         for(int i = 0; i < ncps; ++i) {
            G.block<3,1>(0,i) = cps[i]->pos;
         }
      
         glLineWidth(3.0f);
         for(int k = 0; k < ncps - 3; ++k) {
            int n = 32; // curve discretization
            // Gk is the 3x4 block starting at column k
            Gk = G.block<3,4>(0,k);
            glBegin(GL_LINE_STRIP);
            for(int i = 0; i < n; ++i) {
               prog->unbind();
               if(i/(n/2) % 2 == 0) {
                  // First half color
                  glColor3f(0.0f, 1.0f, 0.0f);
               } else {
                  // Second half color
                  glColor3f(0.0f, 0.0f, 1.0f);
               }
               // u goes from 0 to 1 within this segment
               float u = i / (n - 1.0f);
               // Compute spline point at u
               Eigen::Vector4f uVec(1.0f, u, u*u, u*u*u);
               Eigen::Vector3f Point = Gk * B * uVec;
               glVertex3fv(Point.data());
            }
            glEnd();
         }
      }
         // Pop modelview matrix
      glPopMatrix();
      
      // Pop projection matrix
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      prog->bind();
   }
}

void SplineCurve::translatePoint(int index, Eigen::Vector3f trans) {
   if(index >= 2 && index < cps.size() - 2) {
      cps[index]->pos += trans;
      buildTable();
   }
   else if(index >= cps.size() - 2) {
      cps[cps.size() - 2]->pos += trans;
      cps[cps.size() - 1]->pos += trans;
      buildTable();
   }
   else if(index < 2) {
      cps[0]->pos += trans;
      cps[1]->pos += trans;
      buildTable();
   }
}

void SplineCurve::addPoint() {
   Eigen::Vector3f toPos = cps[cps.size() - 1]->pos + Eigen::Vector3f(5.0f,0.0f,0.0f);
   cps.push_back(new CpsNode);
   cps[cps.size() - 1]->pos = toPos;
   cps[cps.size() - 1]->extraRot = cps[cps.size() - 3]->extraRot;

   cps[cps.size() - 2]->pos = cps[cps.size() - 1]->pos;
   cps[cps.size() - 2]->extraRot = cps[cps.size() - 3]->extraRot;
   
   buildTable();
}

void SplineCurve::end() {
   cps.push_back(cps[0]);
   cps.push_back(cps[1]);
   buildTable();
}

Eigen::Matrix4f SplineCurve::getExtraRot(int k, float u) {
   Eigen::Matrix4f mat2 = Eigen::Matrix4f::Identity();


   float rot = (1 - u) * cps[k]->extraRot + u * cps[k+1]->extraRot;
   mat2(1,1) = cos(rot);
   mat2(1,2) = -sin(rot);
   mat2(2,1) = sin(rot);
   mat2(2,2) = cos(rot);
   return mat2;
}