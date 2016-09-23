#include <glm/glm.hpp>
#include "main.h"
using namespace std;
using namespace glm;

static dvec3 gravity = dvec3(0, -10.0, 0);

void UpdatePhysics_Verlet(const double t, const double dt) {
  for (size_t i = 0; i < ballsy.size(); i++)
  {
    // *********************************
    // calcualte velocity from current and previous position
	  dvec3 velocity = ballsy[i].position - ballsy[i].position_prev;
    // set previous position to current position
	  ballsy[i].position_prev = ballsy[i].position;
    // position += v + a * (dt^2)
	  ballsy[i].position += velocity + gravity * (dt * dt);
    // *********************************

    if (ballsy[i].position.y <= 0.0f) {
      //Note: We can't use: 
      // ballsy[i].velocity.y = -ballsy[i].velocity.y;
      // As verlet calculates it's own velocity, instead we must use impulses.
      // but our simple model can't support that yet, so let's just hack it
      ballsy[i].position_prev = ballsy[i].position + (ballsy[i].position - ballsy[i].position_prev);
    }
  }
}