#pragma once
#ifndef Helicopter_H
#define Helicopter_H

#include <iostream>
#include <memory>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Shape.h"
#include "MatrixStack.h"
#include "Program.h"

class Helicopter
{
public:
	void draw(std::shared_ptr<MatrixStack> MV, std::shared_ptr<Program> prog, double t, bool rotate);

	static void init(std::string dir) {
		hb1->loadMesh(dir + "helicopter_body1.obj");
		hb1->init();
		hb2->loadMesh(dir + "helicopter_body2.obj");
		hb2->init();
		hp1->loadMesh(dir + "helicopter_prop1.obj");
		hp1->init();
		hp2->loadMesh(dir + "helicopter_prop2.obj");
		hp2->init();
	}

	static std::shared_ptr<Shape> hb1;
	static std::shared_ptr<Shape> hb2;
	static std::shared_ptr<Shape> hp1;
	static std::shared_ptr<Shape> hp2;
};

#endif
