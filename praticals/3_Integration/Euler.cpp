#include <glm/glm.hpp>
#include "main.h"
using namespace std;
using namespace glm;

static dvec3 gravity = dvec3(0, -10.0, 0);

void UpdatePhysics_Euler(const double t, const double dt) {
  for (size_t i = 0; i < ballsier.size(); i++)
  {
    // *********************************
    // Apply Accleration to Velocity
	  ballsier[i].velocity += gravity * dt;
    // Apply Velocity to position
	  ballsier[i].position += ballsier[i].velocity * dt;
    // *********************************
    
    //super simple bounce function
    if (ballsier[i].position.y <= 0.0f) {
      ballsier[i].velocity.y = -ballsier[i].velocity.y;
    }
  }
}