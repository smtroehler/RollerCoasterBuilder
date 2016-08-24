#ifdef __APPLE__
#include <GLUT/glut.h>
#endif
#ifdef __unix__
#include <GL/glut.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "GLSL.h"
#include "Program.h"
#include "Camera.h"
#include "MatrixStack.h"
#include "ShapeObj.h"

#include "RCTrack.h"
using namespace std;

bool keyToggles[256] = {false};


float t = 0.0f;
float tPrev = 0.0f;
int width = 1;
int height = 1;



Program prog;
Camera camera;

enum SplineType
{
   CATMULL_ROM = 0,
   BASIS,
   SPLINE_TYPE_COUNT
};


SplineType type = CATMULL_ROM;

Eigen::Matrix4f Bcr, Bb;

RCTrack track;
ShapeObj ground;
ShapeObj coasterCar;



void loadScene()
{
	t = 0.0f;
	keyToggles['c'] = true;
	prog.setShaderNames("phong_vert.glsl", "phong_frag.glsl");
   
   track.trackModel.load("coasterTrack.obj");
   
   ground.load("square.obj");
   track.supportModel.load("support.obj");
   coasterCar.load("coasterCar.obj");
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
   
}

void initGL()
{
	//////////////////////////////////////////////////////
	// Initialize GL for the whole scene
	//////////////////////////////////////////////////////
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	//////////////////////////////////////////////////////
	// Intialize the shapes
	//////////////////////////////////////////////////////
	
	track.trackModel.init();
   track.supportModel.init();
   ground.init();
   coasterCar.init();
	//////////////////////////////////////////////////////
	// Intialize the shaders
	//////////////////////////////////////////////////////
	
	prog.init();
	prog.addUniform("P");
	prog.addUniform("MV");
	prog.addAttribute("vertPos");
	prog.addAttribute("vertNor");

   prog.addUniform("ka");
   prog.addUniform("kd");
   prog.addUniform("ks");
   prog.addUniform("s");
      
   prog.addUniform("light");
   prog.addUniform("inten");

   track.trackModel.bindProg(&prog);
   track.supportModel.bindProg(&prog);
   ground.bindProg(&prog);
   coasterCar.bindProg(&prog);
	//////////////////////////////////////////////////////
	// Final check for errors
	//////////////////////////////////////////////////////
	GLSL::checkVersion();

}

void reshapeGL(int w, int h)
{
	// Set view size
	width = w;
	height = h;
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	camera.setAspect((float)width/(float)height);
}

Eigen::Matrix4f getRotateX(float radians)
{
   Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
   float s = std::sin(radians);
   float c = std::cos(radians);
   M(1,1) = c;
   M(1,2) = -s;
   M(2,1) = s;
   M(2,2) = c;
   return M;
}

Eigen::Matrix4f getRotateY(float radians)
{
   Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
   float s = std::sin(radians);
   float c = std::cos(radians);
   M(0,0) = c;
   M(0,2) = s;
   M(2,0) = -s;
   M(2,2) = c;
   return M;
}



void drawGL()
{
	// Elapsed time
	float tCurr = 0.001f*glutGet(GLUT_ELAPSED_TIME); // in seconds
	if(keyToggles[' ']) {
		t += (tCurr - tPrev);
	}
	tPrev = tCurr;
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyToggles['c']) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyToggles['l']) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	//////////////////////////////////////////////////////
	// Create matrix stacks
	//////////////////////////////////////////////////////
	
	MatrixStack P, MV;

	// Apply camera transforms
	P.pushMatrix();
	camera.applyProjectionMatrix(&P);
	MV.pushMatrix();
   if(keyToggles['v'])
      track.useRCViewMatrix(&MV,camera);
   else
	   camera.applyViewMatrix(&MV);
	
	//////////////////////////////////////////////////////
	// Draw origin frame using old-style OpenGL
	// (before binding the program)
	//////////////////////////////////////////////////////
	
   prog.bind();
   
   glUniformMatrix4fv(prog.getUniform("P"), 1, GL_FALSE, P.topMatrix().data());
   glUniformMatrix4fv(prog.getUniform("MV"), 1, GL_FALSE, MV.topMatrix().data());

   glUniform3fv(prog.getUniform("ka"),  1, Eigen::Vector3f(0.1,0.1,0.1).data());
   glUniform3fv(prog.getUniform("kd"),  1, Eigen::Vector3f(0.8,0.2,0.2).data());
   glUniform3fv(prog.getUniform("ks"), 1, Eigen::Vector3f(.9,.9,.9).data());
   glUniform1f(prog.getUniform("s"), 20.0);

   glUniform1f(prog.getUniform("inten"), 0.8);
   glUniform3fv(prog.getUniform("light"), 1, Eigen::Vector3f(2.0,3.0,2.0).data());
   if(keyToggles[' ']) {
	   track.TravelAlong(&MV, &coasterCar);
   }
   track.draw(&P, &MV, &prog);
   //   trackModel.draw();
   MV.pushMatrix();

   MV.scale(Eigen::Vector3f(1000.0f, 1.0f, 1000.0f));
   MV.translate(Eigen::Vector3f(-0.5f, 0.0f, 0.5f));
   MV.rotate(-90.0 * M_PI / 180.0, Eigen::Vector3f(1.0f,0.0f, 0.0f));
   
   glUniform3fv(prog.getUniform("kd"),  1, Eigen::Vector3f(0.2,0.8,0.2).data());
   ground.draw(&MV);

   MV.popMatrix();




   prog.unbind();
	//////////////////////////////////////////////////////
	// Now draw the shape using modern OpenGL
	//////////////////////////////////////////////////////
	
	// Bind the program


	//////////////////////////////////////////////////////
	// Cleanup
	//////////////////////////////////////////////////////
	
	// Pop stacks
	MV.popMatrix();
	P.popMatrix();
	
	// Swap buffer
	glutSwapBuffers();
}

void mouseGL(int button, int state, int x_, int y_)
{
	int modifier = glutGetModifiers();
	bool shift = modifier & GLUT_ACTIVE_SHIFT;
	bool ctrl  = modifier & GLUT_ACTIVE_CTRL;
	bool alt   = modifier & GLUT_ACTIVE_ALT;
	camera.mouseClicked(x_, y_, shift, ctrl, alt);
}

void mouseMotionGL(int x, int y)
{
	camera.mouseMoved(x, y);
}
int curPoint = 0; 

void keyboardGL(unsigned char key, int x, int y)
{
	keyToggles[key] = !keyToggles[key];
	switch(key) {
		case 27:
			// ESCAPE
			exit(0);
			break;
		case 't':
			t = 0.0f;
			break;
      case 'N':
         track.curve.addPoint();
         curPoint = track.curve.cps.size() - 2;
         track.curve.highlightedPoint = curPoint;
         break;
      case 'x':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(-5.0f,0.0f,0.0f));
         break;
      case 'X':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(5.0f,0.0f,0.0f));
         break;
      case 's':
         curPoint++;
         curPoint = curPoint % track.curve.cps.size();
         if(curPoint == 0)
            curPoint++;
         track.curve.highlightedPoint = curPoint;
         break;
      case 'S':
         curPoint--;
         if(curPoint == 0) {
            curPoint = track.curve.cps.size() - 1;
         }
         track.curve.highlightedPoint = curPoint;
         break;
      case 'y':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(0.0f,-5.0f,0.0f));
         break;
      case 'Y':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(0.0f,5.0f,0.0f));
         break;
      case 'z':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(0.0f,0.0f,-5.0f));
         break;
      case 'Z':
         track.curve.translatePoint(curPoint, Eigen::Vector3f(0.0f,0.0f,5.0f));
         break;
      case 'r':
         track.curve.cps[curPoint-1]->extraRot += .1;
         break;
      case 'R':
         track.curve.cps[curPoint-1]->extraRot -= .1;
         break;
      case 'e':
         track.curve.end();
         break;
      case 'f':
         track.curve.cps[curPoint - 1]->flippedNormal = true;
         break;
      case 'F':
         track.curve.cps[curPoint - 1]->flippedNormal = false;
         break;
      case 'D':
         track.curve = SplineCurve();
         break;
      case '/':
         cout << "enter in an output file name\n";
         char inName[50];
         cin >> inName;
         if(inName != NULL) {
            track.toFile(inName);
         }
         break;
      case '?':
         cout << "enter in the file name you want read\n";
         char outName[50];
         cin >> outName;
         if(outName != NULL) {
            track.loadFile(outName);
         }

         break;
	}
}

void timerGL(int value)
{
	glutPostRedisplay();
	glutTimerFunc(20, timerGL, 0);
}


void idleGL() {
   
}

int main(int argc, char **argv)
{  


	glutInit(&argc, argv);
	glutInitWindowSize(1200, 800);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Sean Troehler");
	glutMouseFunc(mouseGL);
	glutMotionFunc(mouseMotionGL);
	glutKeyboardFunc(keyboardGL);
	glutReshapeFunc(reshapeGL);
	glutDisplayFunc(drawGL);
	glutTimerFunc(20, timerGL, 0);
   glutIdleFunc(idleGL);
	loadScene();
	initGL();
	glutMainLoop();
	return 0;
}
