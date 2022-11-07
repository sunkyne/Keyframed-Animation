#include <iostream>
#include <vector>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Camera.h"
#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"
#include "Shape.h"
#include "Helicopter.h"
#include "Keyframe.h"

using namespace std;

GLFWwindow *window; // Main application window
string RESOURCE_DIR = ""; // Where the resources are loaded from

int keyPresses[256] = {0}; // only for English keyboards!

shared_ptr<Program> prog;
shared_ptr<Camera> camera;

glm::mat4 B;

Helicopter h;

// Control points
vector<glm::vec3> cps;

// Quats
vector<glm::quat> rqs;

// Keyframes
vector<Keyframe> hkf;

// lookup table
vector<pair<float, float> > usTable;

// Time control points
vector<pair<float, float> > tcps;

// Time control coeff
glm::vec4 coeff;

int nkf;
float tmax;
float smax;

static void error_callback(int error, const char *description)
{
	cerr << description << endl;
}

static void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

static void char_callback(GLFWwindow *window, unsigned int key)
{
	keyPresses[key]++;
}

static void cursor_position_callback(GLFWwindow* window, double xmouse, double ymouse)
{
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	if(state == GLFW_PRESS) {
		camera->mouseMoved((float) xmouse, (float) ymouse);
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	// Get the current mouse position.
	double xmouse, ymouse;
	glfwGetCursorPos(window, &xmouse, &ymouse);
	// Get current window size.
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	if(action == GLFW_PRESS) {
		bool shift = mods & GLFW_MOD_SHIFT;
		bool ctrl  = mods & GLFW_MOD_CONTROL;
		bool alt   = mods & GLFW_MOD_ALT;
		camera->mouseClicked((float) xmouse, (float) ymouse, shift, ctrl, alt);
	}
}

glm::mat4 compute_G(int k) {
	glm::mat4 G;
	G[0] = glm::vec4(cps[(int) std::fmod(k + nkf - 1, nkf)], 0.0f);
	G[1] = glm::vec4(cps[k], 0.0f);
	G[2] = glm::vec4(cps[(int) std::fmod(k + 1, nkf)], 0.0f);
	G[3] = glm::vec4(cps[(int) std::fmod(k + 2, nkf)], 0.0f);
	return G;
}

void addQuat(glm::quat q1)
{
	if (rqs.empty()) {
		rqs.push_back(q1);
	}
	else {
		glm::quat q0 = rqs.at(rqs.size() - 1);
		if (dot(q0, q1) < 0) {
			q1 = -q1;
		}
		rqs.push_back(q1);
	}
}	

float arclength(float u1, float u2) {
	int k = (int) floor(u1);
	u1 -= k;
	u2 -= k;

	glm::mat4 Gk = compute_G(k);

	glm::vec4 uVec1(1.0f, u1, u1 * u1, u1 * u1 * u1);
	glm::vec3 P1(Gk * (B * uVec1));
	glm::vec4 uVec2(1.0f, u2, u2 * u2, u2 * u2 * u2);
	glm::vec3 P2(Gk * (B * uVec2));
	return glm::length(P2 - P1);
}

void buildTable()
{
	usTable.clear();
	float s = 0;
	float u = 0;
	float delta_u = 0.2f;
	usTable.push_back(make_pair(u, s));
	while (u <= nkf-delta_u) {
		s += arclength(u, u + delta_u);
		u += delta_u;
		usTable.push_back(make_pair(u, s));
	}
}

float s2u(float s)
{
	int i1 = 0;
	int i2 = 0;
	for (size_t i = 0; i < usTable.size(); i++) {
		if (usTable[i].second > s) {
			i2 = i;
			i1 = i - 1;
			break;
		}
	}
	if (!i1 && !i2) {
		return 0;
	}
	float s0 = usTable[i1].second;
	float s1 = usTable[i2].second;
	float u0 = usTable[i1].first;
	float u1 = usTable[i2].first;
	float alpha = (s - s0) / (s1 - s0);

	float u = (1 - alpha) * u0 + alpha * u1;

	return u;
}

static void init()
{
	GLSL::checkVersion();
	
	// Set background color
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);

	keyPresses[(unsigned)'c'] = 1;
	
	prog = make_shared<Program>();
	prog->setShaderNames(RESOURCE_DIR + "phong_vert.glsl", RESOURCE_DIR + "phong_frag.glsl");
	prog->setVerbose(true);
	prog->init();
	prog->addUniform("P");
	prog->addUniform("MV");
	prog->addUniform("lightPos");
	prog->addUniform("ka");
	prog->addUniform("kd");
	prog->addUniform("ks");
	prog->addUniform("s");
	prog->addAttribute("aPos");
	prog->addAttribute("aNor");
	prog->setVerbose(false);
	
	camera = make_shared<Camera>();
	
	Helicopter::init(RESOURCE_DIR);
	h = Helicopter();

	B[0] = glm::vec4(0, 2, 0, 0);
	B[1] = glm::vec4(-1, 0, 1, 0);
	B[2] = glm::vec4(2, -5, 4, -1);
	B[3] = glm::vec4(-1, 3, -3, 1);
	B *= 0.5;

	cps.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	cps.push_back(glm::vec3(0.6f, 1.2f, 4.3f));
	cps.push_back(glm::vec3(0.2f, 3.7f, 2.6f));
	cps.push_back(glm::vec3(3.4f, 2.9f, 0.2f));
	cps.push_back(glm::vec3(2.5f, -0.8f, 1.3f));

	addQuat(glm::angleAxis(glm::radians(90.f), glm::vec3(0.0f, 0.0f, 0.0f)));
	addQuat(glm::angleAxis(glm::radians(90.f), glm::vec3(0.2f, 0.5f, 0.0f)));
	addQuat(glm::angleAxis(glm::radians(-90.f), glm::vec3(0.4f, 0.3f, 0.7f)));
	addQuat(glm::angleAxis(glm::radians(180.f), glm::vec3(0.0f, 0.0f, 1.0f)));
	addQuat(glm::angleAxis(glm::radians(180.f), glm::vec3(0.3f, 0.8f, 0.4f)));

	nkf = cps.size();
	// Create keyframe objects
	for (int i = 0; i < nkf; i++) {
		Helicopter h_hat = Helicopter();
		hkf.push_back(Keyframe(cps[i], rqs[i], h_hat));
	}

	// Arc length parameterization table
	buildTable();

	// Time control points
	tcps.push_back(make_pair(0.0f, 0.0f));
	tcps.push_back(make_pair(0.2f, 0.5f));
	tcps.push_back(make_pair(0.7f, 0.3f));
	tcps.push_back(make_pair(1.0f, 1.0f));

	// Solve for cubic fit
	glm::mat4 A;
	glm::vec4 b;
	// Fill A and b
	b = glm::vec4(tcps[0].second, tcps[1].second, tcps[2].second, tcps[3].second);
	A[0] = glm::vec4(pow(tcps[0].first, 3.0), pow(tcps[1].first, 3.0), pow(tcps[2].first, 3.0), pow(tcps[3].first, 3.0));
	A[1] = glm::vec4(pow(tcps[0].first, 2.0), pow(tcps[1].first, 2.0), pow(tcps[2].first, 2.0), pow(tcps[3].first, 2.0));
	A[2] = glm::vec4(pow(tcps[0].first, 1.0), pow(tcps[1].first, 1.0), pow(tcps[2].first, 1.0), pow(tcps[3].first, 1.0));
	A[3] = glm::vec4(1.0f);

	// Solve for coeff
	coeff = glm::inverse(A) * b;
	cout << coeff.x << " " << coeff.y << " " << coeff.z << " " << coeff.w << endl;

	tmax = 4;
	smax = usTable.back().second;
	
	// Initialize time.
	glfwSetTime(0.0);
	
	// If there were any OpenGL errors, this will print something.
	// You can intersperse this line in your code to find the exact location
	// of your OpenGL error.
	GLSL::checkError(GET_FILE_LINE);
}

void render()
{
	// Update time.
	double t = glfwGetTime();
	
	// Get current frame buffer size.
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	// Use the window size for camera.
	glfwGetWindowSize(window, &width, &height);
	camera->setAspect((float)width/(float)height);
	
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(keyPresses[(unsigned)'c'] % 2) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(keyPresses[(unsigned)'z'] % 2) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	
	auto P = make_shared<MatrixStack>();
	auto MV = make_shared<MatrixStack>();
	
	// Apply camera transforms
	P->pushMatrix();
	camera->applyProjectionMatrix(P);
	MV->pushMatrix();
	camera->applyViewMatrix(MV);

	prog->bind();
	glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
		
	glm::mat4 G;
	
	float tNorm = (float) std::fmod(t, tmax) / tmax;
	float sNorm = tNorm;
	float s = smax * sNorm;
	float u;

	if (keyPresses[(unsigned)'s'] % 4 == 1) {
		// arc-length
		u = s2u(s);
	}
	else if (keyPresses[(unsigned)'s'] % 4 == 2) {
		// ease in/out
		sNorm = -2 * (float) pow(tNorm, 3.0) + 3 * (float) pow(tNorm, 2.0);
		s = smax * sNorm;
		u = s2u(s);
	}
	else if (keyPresses[(unsigned)'s'] % 4 == 3) {
		// custom function
		sNorm = coeff.x * (float) pow(tNorm, 3.0) + coeff.y * (float) pow(tNorm, 2.0) + coeff.z * tNorm + coeff.w;
		s = smax * sNorm;
		u = s2u(s);
	}
	else {
		// no arc-length
		u = (float) std::fmod(t, nkf);
	}

	int k = (int) floor(u);
	float u_hat = u - k;
	glm::vec4 uVec(1.0f, u_hat, u_hat * u_hat, u_hat * u_hat * u_hat);

	int i1 = (int) std::fmod(k + nkf - 1, nkf);
	int i2 = k;
	int i3 = (int) std::fmod(k + 1, nkf);
	int i4 = (int) std::fmod(k + 2, nkf);
	// Fill in G with rotation quaternion
	G[0] = glm::vec4(rqs[i1].x, rqs[i1].y, rqs[i1].z, rqs[i1].w);
	G[1] = glm::vec4(rqs[i2].x, rqs[i2].y, rqs[i2].z, rqs[i2].w);
	G[2] = glm::vec4(rqs[i3].x, rqs[i3].y, rqs[i3].z, rqs[i3].w);
	G[3] = glm::vec4(rqs[i4].x, rqs[i4].y, rqs[i4].z, rqs[i4].w);

	glm::vec4 qVec = G * (B * uVec);
	glm::quat q(qVec[3], qVec[0], qVec[1], qVec[2]); // Constructor argument order: (w, x, y, z)
	glm::mat4 E = glm::mat4_cast(glm::normalize(q)); // Creates a rotation matrix

	// Fill in G
	G = compute_G(k);

	// Compute position at u
	glm::vec3 p = G * (B * uVec);
	E[3] = glm::vec4(p, 1.0f); // Puts the position into the last column

	MV->pushMatrix();
	MV->multMatrix(E);
	h.draw(MV, prog, t, true);
	MV->popMatrix();

	if (keyPresses[(unsigned)'k'] % 2) {
		// Draw keyframes
		for (size_t i = 0; i < hkf.size(); i++) {
			MV->pushMatrix();
			MV->multMatrix(hkf[i].getMat());
			hkf[i].getHeli().draw(MV, prog, t, false);
			MV->popMatrix();
		}

		// Draw curve
		glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
		glm::mat4 G_curve;
		glBegin(GL_LINE_STRIP);
		for (float u_curve = 0; u_curve <= (float) nkf; u_curve += 0.01f) {
			int k = (int) floor(u_curve);
			float u_hat = u_curve - k;
			// Fill in G
			G_curve = compute_G(k);
			// Fill in uVec
			glm::vec4 uVec(1.0f, u_hat, u_hat * u_hat, u_hat * u_hat * u_hat);
			// Compute position at u
			glm::vec4 p_curve = G_curve * (B * uVec);
			glVertex3f(p_curve.x, p_curve.y, p_curve.z);
		}
		glEnd();

		// Draw equally spaced points on spline curve
		float ds = 0.5;
		glPointSize(3.0f);
		glBegin(GL_POINTS);
		for (float s = 0.0f; s < smax; s += ds) {
			// Convert from s to (concatenated) u
			float u_hat = s2u(s);
			int k = (int) floor(u_hat);
			float u = u_hat - k;
			// Compute spline point at u
			glm::mat4 Gk = compute_G(k);
			glm::vec4 uVec(1.0f, u, u * u, u * u * u);
			glm::vec3 P(Gk * (B * uVec));
			glVertex3fv(&P[0]);
		}
		glEnd();
	}

	prog->unbind();
	
	// Draw the frame and the grid with OpenGL 1.x (no GLSL)
	
	// Setup the projection matrix
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadMatrixf(glm::value_ptr(P->topMatrix()));
	
	// Setup the modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixf(glm::value_ptr(MV->topMatrix()));
	
	// Draw frame
	glLineWidth(2);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glColor3f(0, 1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();
	glLineWidth(1);
	
	// Draw grid
	float gridSizeHalf = 20.0f;
	int gridNx = 40;
	int gridNz = 40;
	glLineWidth(1);
	glColor3f(0.8f, 0.8f, 0.8f);
	glBegin(GL_LINES);
	for(int i = 0; i < gridNx+1; ++i) {
		float alpha = i / (float)gridNx;
		float x = (1.0f - alpha) * (-gridSizeHalf) + alpha * gridSizeHalf;
		glVertex3f(x, 0, -gridSizeHalf);
		glVertex3f(x, 0,  gridSizeHalf);
	}
	for(int i = 0; i < gridNz+1; ++i) {
		float alpha = i / (float)gridNz;
		float z = (1.0f - alpha) * (-gridSizeHalf) + alpha * gridSizeHalf;
		glVertex3f(-gridSizeHalf, 0, z);
		glVertex3f( gridSizeHalf, 0, z);
	}
	glEnd();
	
	// Pop modelview matrix
	glPopMatrix();
	
	// Pop projection matrix
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	
	// Pop stacks
	MV->popMatrix();
	P->popMatrix();
	
	GLSL::checkError(GET_FILE_LINE);
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		cout << "Please specify the resource directory." << endl;
		return 0;
	}
	RESOURCE_DIR = argv[1] + string("/");
	
	// Set error callback.
	glfwSetErrorCallback(error_callback);
	// Initialize the library.
	if(!glfwInit()) {
		return -1;
	}
	// Create a windowed mode window and its OpenGL context.
	window = glfwCreateWindow(640, 480, "Kyne Sun", NULL, NULL);
	if(!window) {
		glfwTerminate();
		return -1;
	}
	// Make the window's context current.
	glfwMakeContextCurrent(window);
	// Initialize GLEW.
	glewExperimental = true;
	if(glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return -1;
	}
	glGetError(); // A bug in glewInit() causes an error that we can safely ignore.
	cout << "OpenGL version: " << glGetString(GL_VERSION) << endl;
	cout << "GLSL version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
	// Set vsync.
	glfwSwapInterval(1);
	// Set keyboard callback.
	glfwSetKeyCallback(window, key_callback);
	// Set char callback.
	glfwSetCharCallback(window, char_callback);
	// Set cursor position callback.
	glfwSetCursorPosCallback(window, cursor_position_callback);
	// Set mouse button callback.
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	// Initialize scene.
	init();
	// Loop until the user closes the window.
	while(!glfwWindowShouldClose(window)) {
		if(!glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
			// Render scene.
			render();
			// Swap front and back buffers.
			glfwSwapBuffers(window);
		}
		// Poll for and process events.
		glfwPollEvents();
	}
	// Quit program.
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
