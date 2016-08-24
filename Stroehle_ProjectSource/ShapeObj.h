#pragma once
#ifndef _SHAPEOBJ_H_
#define _SHAPEOBJ_H_

#include "tiny_obj_loader.h"
#include "Program.h"
#include "MatrixStack.h"
class ShapeObj
{
public:
	ShapeObj();
	virtual ~ShapeObj();
	void load(const std::string &meshName);
	void init();
   void bindProg(Program *toBind);
	void draw(MatrixStack* MV) const;
	
private:
	std::vector<tinyobj::shape_t> shapes;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;
	unsigned indBufID;

   Program *prog;
};

#endif
