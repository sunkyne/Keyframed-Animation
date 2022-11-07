#include "Keyframe.h"

using namespace std;

Keyframe::Keyframe(float x, float y, float z, glm::quat q, Helicopter h) {
	this->E = glm::mat4_cast(q);
	this->E[3] = glm::vec4(x, y, z, 1.0f);
	this->h = h;
}

Keyframe::Keyframe(glm::vec3 v, glm::quat q, Helicopter h) {
	this->E = glm::mat4_cast(q);
	this->E[3] = glm::vec4(v, 1.0f);
	this->h = h;
}

Keyframe::Keyframe(glm::mat4 E, Helicopter h) {
	this->E = E;
	this->h = h;
}

glm::mat4 Keyframe::getMat() {
	return this->E;
}

Helicopter Keyframe::getHeli() {
	return this->h;
}