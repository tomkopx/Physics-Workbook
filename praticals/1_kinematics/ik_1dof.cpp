#include "ik.h"
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <iostream>
#include <phys_utils.h>
#include <vector>
using namespace std;
using namespace glm;
static unsigned int numLinks = 0;

static void Reach(int i, const vec3 &target, std::vector<Link> &const links) {
  // our current orientation
  dquat qCur = angleAxis(links[i].m_angle, links[i].m_axis);
  // current position of this effector
  vec3 vlinkBasePos = (links[i].m_base)[3];
  // current position of the effector at the end of the chain
  vec3 vEndEffPos = links[links.size() - 1].m_end[3];
  // convert our axis to world space
  vec3 vLinkAxis = links[i].m_worldaxis;
  // project target onto axis plane
  vec3 vplanetarget = projectOntoPlane(target, vLinkAxis, vlinkBasePos);
  // project vEndEffPos onto axis plane
  vec3 vplaneEndEff = projectOntoPlane(vEndEffPos, vLinkAxis, vlinkBasePos);

  // These are the two vectors we want to converge.
  vec3 vLinkBaseToEndEffDirection = normalize(vplaneEndEff - vlinkBasePos);
  vec3 vLinkBaseToTargetDirection = normalize(vplanetarget - vlinkBasePos);

  // Get Dot of the two vectors
  float cosAngle = dot(vLinkBaseToTargetDirection, vLinkBaseToEndEffDirection);
  if (abs(cosAngle) < 1.0f) {
	  // *********************************
	  // Get the Angle between the two vectors
	  float angleBetween = acos(cosAngle);
	  // Turn into a Quat with our axis
	  dquat qRot = angleAxis(angleBetween, vLinkAxis);
	  // Multply our current Quat with it
	  dquat qFinal = qCur * qRot;
	  // Pull out the angle component, set the link params
	  links[i].m_angle = angle(qFinal);
	  links[i].m_axis = axis(qFinal);
	  // *********************************
  }
}

void ik_1dof_Update(const vec3 &const target, std::vector<Link> &const links, const float linkLength) {
  numLinks = links.size();
  // for (size_t i = links.size(); i >= 1; --i) {
  for (size_t i = 0; i < links.size() - 1; ++i) {
    UpdateHierarchy();
    Reach(i, target, links);
    const float distance = length(vec3(links[links.size() - 1].m_end[3]) - target);
    if (distance < 0.5f) {
      return;
    }
  }
}