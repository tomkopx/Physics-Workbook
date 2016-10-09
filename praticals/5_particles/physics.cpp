#include "physics.h"
#include <glm/glm.hpp>
using namespace std;
using namespace glm;
static vector<cPhysics *> physicsScene;

static dvec3 gravity = dvec3(0, -10.0, 0);
static dvec3 verticalA = dvec3(0, 30.0, 0);

cPhysics::cPhysics() : pm(POINT), Component("Physics") { physicsScene.push_back(this); }

cPhysics::~cPhysics() {
  auto position = std::find(physicsScene.begin(), physicsScene.end(), this);
  if (position != physicsScene.end()) {
    physicsScene.erase(position);
  }
}

void cPhysics::Update(double delta) {
  for (auto &e : physicsScene) {
    e->GetParent()->SetPosition(e->position);
  }
}

void cPhysics::SetParent(Entity *p) {
  Component::SetParent(p);
  position = Ent_->GetPosition();
  prev_position = position;
}

void UpdatePhysics(const double t, const double dt) {
  for (auto &e : physicsScene) {
    e->Render();
    // calcualte velocity from vertical acceleration and dtime
    dvec3 velocity = verticalA * dt;
    // set previous position to current position
    e->prev_position = e->position;
    // position += v + a * (dt^2)
	e->position += velocity + verticalA * pow(dt, 2);

	//The effect of gravity on the upwards acceleration
	verticalA += gravity * dt;

	cout << velocity << endl;

    if (e->position.y <= 0.0f) {
		gravity *= 0.0;
		//e->prev_position = e->position + (e->position - e->prev_position);
		e->position = e->prev_position;
    }
  }
}

void InitPhysics() {}

void ShutdownPhysics() {}
