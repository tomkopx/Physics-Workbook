#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <graphics_framework.h>
#include <phys_utils.h>
#include <thread>
#include "main.h"
using namespace std;
using namespace graphics_framework;
using namespace glm;

std::vector<sBall> balls;
std::vector<sBall> ballsy;
std::vector<sBall> ballsier;
chrono::time_point<chrono::high_resolution_clock> tp_start;
chrono::time_point<chrono::high_resolution_clock> tp_end;

//use this function simulate render workload
void doWork() {
  // this_thread::sleep_for(std::chrono::milliseconds(rand()%50));
}



bool update(double delta_time) {

	doWork();

	static uint16_t frames = 0;
	static uint16_t ticks = 0;
	static double t = 0.0;
	static double accumulator = 0.0;

    frames++;
    accumulator += delta_time;

    while (accumulator > physics_tick) {
      ticks++;
      UpdatePhysics_Euler(t, physics_tick); //Ballsier
      UpdatePhysics_Verlet(t, physics_tick); // Ballsy
      UpdatePhysics_rk4(t, physics_tick); // Balls
      accumulator -= physics_tick;
      t += physics_tick;
      
    }
    

	phys::SetCameraTarget(ballsy[0].position + dvec3(0.0, 0.0, 10.0));
	phys::Update(delta_time);
	return true;
}

bool load_content() {
  phys::Init();
  for (size_t i = 0; i < 10; i++)
  {
    balls.push_back({ dvec3(0), dvec3((i*2)-15.0,10+i,0), dvec3((i * 2) - 15.0,10 + i,0) });
  }
  for (size_t i = 0; i < 10; i++)
  {
	  ballsy.push_back({ dvec3(0), dvec3((i * 2) - 15.0,10 + i,0), dvec3((i * 2) - 15.0,10 + i,0) });
  }
  for (size_t i = 0; i < 10; i++)
  {
	  ballsier.push_back({ dvec3(0), dvec3((i * 2) - 15.0,10 + i,0), dvec3((i * 2) - 15.0,10 + i,0) });
  }

  phys::SetCameraPos(vec3(20.0f, 10.0f, 20.0f));
  phys::SetCameraTarget(vec3(0, 10.0f, 0));
  tp_start = chrono::high_resolution_clock::now();
  tp_end = chrono::high_resolution_clock::now();
  return true;
}

bool render() {
  for (size_t i = 0; i < balls.size(); i++)
  {
    phys::DrawSphere(balls[i].position, 1.0f, BLUE);
  }
  for (size_t i = 0; i < ballsy.size(); i++)
  {
	  phys::DrawSphere(ballsy[i].position + dvec3(0.0, 0.0, 10.0), 1.0f, RED);
  }
  for (size_t i = 0; i < ballsier.size(); i++)
  {
	  phys::DrawSphere(ballsier[i].position + dvec3(0.0, 0.0, 20.0), 1.0f, GREEN);
  }

  phys::DrawScene();
  return true;
}

void main() {
  // Create application
  app application;
  // Set load content, update and render methods
  application.set_load_content(load_content);
  application.set_update(update);
  application.set_render(render);
  // Run application
  application.run();
}