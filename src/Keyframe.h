#pragma once
#ifndef Keyframe_H
#define Keyframe_H

#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Helicopter.h"

class Keyframe
{
public:
	Keyframe(float x, float y, float z, glm::quat q, Helicopter h);
	Keyframe(glm::vec3 v, glm::quat q, Helicopter h);
	Keyframe(glm::mat4 E, Helicopter h);

	glm::mat4 getMat();
	Helicopter getHeli();

private:
	glm::mat4 E;
	Helicopter h;
};

#endif
