#include "Helicopter.h"

using namespace std;

shared_ptr<Shape> Helicopter::hb1 = make_shared<Shape>();
shared_ptr<Shape> Helicopter::hb2 = make_shared<Shape>();
shared_ptr<Shape> Helicopter::hp1 = make_shared<Shape>();
shared_ptr<Shape> Helicopter::hp2 = make_shared<Shape>();

void Helicopter::draw(shared_ptr<MatrixStack> MV, shared_ptr<Program> prog, double t, bool rotate) {
	MV->pushMatrix();
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glUniform3f(prog->getUniform("kd"), 1.0f, 0.0f, 0.0f);
	hb1->draw(prog);
	glUniform3f(prog->getUniform("kd"), 1.0f, 1.0f, 0.0f);
	hb2->draw(prog);

	MV->pushMatrix();
	if (rotate) {
		MV->rotate(t * 5.0f, 0.0f, 1.0f, 0.0f);
	}
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glUniform3f(prog->getUniform("kd"), 0.0f, 0.0f, 0.0f);
	hp1->draw(prog);
	MV->popMatrix();

	MV->pushMatrix();
	if (rotate) {
		MV->translate(0.6228f, 0.1179f, 0.1365f);
		MV->rotate(t * 5.0f, 0.0f, 0.0f, 1.0f);
		MV->translate(-0.6228f, -0.1179f, -0.1365f);
	}
	glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
	glUniform3f(prog->getUniform("kd"), 0.0f, 0.0f, 0.0f);
	hp2->draw(prog);
	MV->popMatrix();

	MV->popMatrix();
}

