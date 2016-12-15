#include "game.h"
#include "physics.h"
#include "cPhysicsComponents.h"
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <graphics_framework.h>
#include <phys_utils.h>
#include <thread>

using namespace std;
using namespace graphics_framework;
using namespace glm;
#define physics_tick 1.0 / 60.0

static vector<unique_ptr<Entity>> SceneList;
static unique_ptr<Entity> floorEnt;
free_camera cam;
double cursor_x = 0.0;
double cursor_y = 0.0;

unique_ptr<Entity> CreateParticle() {
	unique_ptr<Entity> ent(new Entity());
	ent->SetPosition(vec3(0, 5.0 + (double)(rand() % 200) / 20.0, 0));
	unique_ptr<Component> physComponent(new cParticle());
	unique_ptr<cShapeRenderer> renderComponent(new cShapeRenderer(cShapeRenderer::SPHERE));
	renderComponent->SetColour(phys::RandomColour());
	ent->AddComponent(physComponent);
	ent->AddComponent(unique_ptr<Component>(new cSphereCollider()));
	ent->AddComponent(unique_ptr<Component>(move(renderComponent)));
	return ent;
}

unique_ptr<Entity> CreateBox(const vec3 &position) {

	unique_ptr<Entity> ent(new Entity());
	ent->SetPosition(position);
	//ent->SetRotation(angleAxis(-90.0f, vec3(1, 0, 0)));
	unique_ptr<Component> physComponent(new cRigidCube());
	unique_ptr<cShapeRenderer> renderComponent(new cShapeRenderer(cShapeRenderer::BOX));
	renderComponent->SetColour(phys::RandomColour());
	ent->AddComponent(physComponent);
	ent->SetName("Cuby");
	ent->AddComponent(unique_ptr<Component>(new cBoxCollider()));
	ent->AddComponent(unique_ptr<Component>(move(renderComponent)));

	return ent;
}

bool initialise() {

	glfwSetInputMode(renderer::get_window(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwGetCursorPos(renderer::get_window(), &cursor_x, &cursor_y);

	return true;
}

//Graphics framework free camera code, does not work
void fCam(double delta_time)
{
	// The ratio of pixels to rotation - remember the fov
	static double ratio_width = quarter_pi<float>() / static_cast<float>(renderer::get_screen_width());
	static double ratio_height = (quarter_pi<float>() * (static_cast<float>(renderer::get_screen_height()) / static_cast<float>(renderer::get_screen_width()))) / static_cast<float>(renderer::get_screen_height());

	double current_x;
	double current_y;

	glfwGetCursorPos(renderer::get_window(), &current_x, &current_y);



	double delta_x = current_x - cursor_x;
	double delta_y = current_y - cursor_y;

	delta_x = delta_x * ratio_width;
	delta_y = -delta_y * ratio_height;



	cam.rotate(delta_x, delta_y);


	if (glfwGetKey(renderer::get_window(), GLFW_KEY_W))
		cam.move(vec3(0.0f, 0.0f, 0.5f));
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_S))
		cam.move(vec3(0.0f, 0.0f, -0.5f));
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_A))
		cam.move(vec3(-0.5f, 0.0f, 0.0f));
	if (glfwGetKey(renderer::get_window(), GLFW_KEY_D))
		cam.move(vec3(0.5f, 0.0f, 0.0f));


	cam.update(delta_time);

	glfwGetCursorPos(renderer::get_window(), &cursor_x, &cursor_y);
}

bool update(double delta_time) {
	static double t = 0.0;
	static double accumulator = 0.0;
	accumulator += delta_time;

	if (glfwGetKey(renderer::get_window(), GLFW_KEY_SPACE)) {
		for (auto &e : SceneList) {
			auto b = e->getComponent<cRigidCube>();
			if (b != NULL) {
				b->AddAngularForce({0, 0, 5.0});
			}
		}
	}

	while (accumulator > physics_tick) {
		UpdatePhysics(t, physics_tick);
		accumulator -= physics_tick;
		t += physics_tick;
	}

	for (auto &e : SceneList) {
		e->Update(delta_time);
	}
	fCam(delta_time);
	phys::Update(delta_time);
	
	return true;
}

bool load_content() {
	phys::Init();
	for (size_t i = 0; i < 1; i++) {
		SceneList.push_back(move(CreateParticle()));
	}

	SceneList.push_back(move(CreateBox({0, 0, 0})));
	SceneList.push_back(move(CreateBox({ 0, 3, 0 })));
	//SceneList.push_back(move(CreateBox({ 0, 10, 0 })));
	//SceneList.push_back(move(CreateBox({ 0, 9, 0 })));
	//SceneList.push_back(move(CreateBox({ 0, 8, 0 })));

	floorEnt = unique_ptr<Entity>(new Entity());
	floorEnt->AddComponent(unique_ptr<Component>(new cPlaneCollider()));
	floorEnt->SetName("Floor");

	cam.set_position(vec3(0.0f, 10.0f, 0.0f));
	cam.set_target(vec3(0.0f, 4.0f, 0.0f));
	auto aspect = static_cast<float>(renderer::get_screen_width()) / static_cast<float>(renderer::get_screen_height());
	cam.set_projection(quarter_pi<float>(), aspect, 2.414f, 1000.0f);
	InitPhysics();
	return true;
}

bool render() {
	for (auto &e : SceneList) {
		e->Render();
	}
	phys::DrawScene();
	return true;
}

void main() {
	// Create application
	app application;
	// Set load content, update and render methods
	application.set_load_content(load_content);
	application.set_initialise(initialise);
	application.set_update(update);
	application.set_render(render);
	// Run application
	application.run();
}