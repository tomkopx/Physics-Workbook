#include "collision.h"
#include "cPhysicsComponents.h"
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
using namespace std;
using namespace glm;
namespace collision {

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cSphereCollider &c1, const cSphereCollider &c2) {
	const dvec3 p1 = c1.GetParent()->GetPosition();
	const dvec3 p2 = c2.GetParent()->GetPosition();
	const dvec3 d = p2 - p1;
	const double distance = glm::length(d);
	const double sumRadius = c1.radius + c2.radius;

	if (distance < sumRadius) {
		auto depth = sumRadius - distance;
		auto norm = -glm::normalize(d);
		auto pos = p1 - norm * (c1.radius - depth * 0.5f);
		civ.push_back({&c1, &c2, pos, norm, depth});
		return true;
	}

	return false;
}

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cSphereCollider &s, const cPlaneCollider &p) {
	const dvec3 sp = s.GetParent()->GetPosition();
	const dvec3 pp = p.GetParent()->GetPosition();

	// Calculate a vector from a point on the plane to the center of the sphere
	const dvec3 vecTemp(sp - pp);

	// Calculate the distance: dot product of the new vector with the plane's normal
	double distance = dot(vecTemp, p.normal);

	if (distance <= s.radius) {
		civ.push_back({&s, &p, sp - p.normal * distance, p.normal, s.radius - distance});
		return true;
	}

	return false;
}

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cSphereCollider &c1, const cBoxCollider &c2) {
	const dvec3 sp = c1.GetParent()->GetPosition();
	const dvec3 bp = c2.GetParent()->GetPosition();

	if (length(sp - bp) < c1.radius + c2.radius) {
		// TODO
		cout << "Sphere Box" << endl;
		// return true;
	}

	return false;
}

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cPlaneCollider &c1, const cPlaneCollider &c2) {
	// TODO
	cout << "PLANE PLANE" << endl;
	return false;
}

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cPlaneCollider &p, const cBoxCollider &b) {
	const dvec3 pp = p.GetParent()->GetPosition();
	const dvec3 bp = b.GetParent()->GetPosition();

	// local coords on cube
	dvec3 points[8] = {dvec3(b.radius, b.radius, b.radius),   dvec3(-b.radius, b.radius, b.radius),
		               dvec3(b.radius, -b.radius, b.radius),  dvec3(-b.radius, -b.radius, b.radius),
                       dvec3(b.radius, b.radius, -b.radius),  dvec3(-b.radius, b.radius, -b.radius),
                       dvec3(b.radius, -b.radius, -b.radius), dvec3(-b.radius, -b.radius, -b.radius)};

	// transfrom to global
	const mat4 m = glm::translate(bp) * mat4_cast(b.GetParent()->GetRotation());
	for (int i = 0; i < 8; i++) {
		points[i] = dvec3(m * dvec4(points[i], 1.0));
	}

	// For each point on the cube, which side of cube are they on?
	double distances[8];
	bool isCollided = false;
	for (int i = 0; i < 8; i++) {
		dvec3 planeNormal = p.normal;

		distances[i] = dot(pp, planeNormal) - dot(points[i], planeNormal);

		if (distances[i] > 0) {
		//		cout << "CuboidPlane!\n";
			civ.push_back({&p, &b, points[i] + planeNormal * distances[i], planeNormal, distances[i]});
			isCollided = true;
		}
	}

	return isCollided;
}

bool IsCollidingCheck(std::vector<collisionInfo> &civ, const cBoxCollider &c1, const cBoxCollider &c2) {

	const dvec3 ap = c1.GetParent()->GetPosition();
	const dvec3 bp = c2.GetParent()->GetPosition();

	//Axis of c1
	dvec3 a[3] = { dvec3(1, 0, 0), dvec3(0, 1, 0), dvec3(0, 0, 1) };

	//Axis of c2
	dvec3 b[3] = { dvec3(1, 0, 0), dvec3(0, 1, 0), dvec3(0, 0, 1) };
	

	//Since half lengths of all of them is 1
	double whd = c1.radius;


	dvec3 T = bp - ap;

	double R[3][3] = { {} };

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[i][j] = dot(a[i], b[j]);
		}
	}

	bool collision = true;

	// L = Ax
	if (dot(T, a[0]) > whd + (whd * R[0][0]) + (whd * R[0][1]) + (whd * R[0][2])) {
		collision = false;
	}
	// L = Ay
	if (dot(T, a[1]) > whd + (whd * R[1][0]) + (whd * R[1][1]) + (whd * R[1][2])) {
		collision = false;
	}
	// L = Az
	if (dot(T, a[2]) > whd + (whd * R[2][0]) + (whd * R[2][1]) + (whd * R[2][2])) {
		collision = false;
	}
	// L = Bx
	if (dot(T, b[0]) > (whd * R[0][0]) + (whd * R[1][0]) + (whd * R[2][0]) + whd) {
		collision = false;
	}
	// L = By
	if (dot(T, b[1]) > (whd * R[0][1]) + (whd * R[1][1]) + (whd * R[2][1]) + whd) {
		collision = false;
	}
	// L = Bz
	if (dot(T, b[2]) > (whd * R[0][2]) + (whd * R[1][2]) + (whd * R[2][2]) + whd) {
		collision = false;
	}
	// L = Ax x Bx
	if ((dot(T, a[2]) * R[1][0]) - (dot(T, a[1]) * R[2][0]) > (whd * R[2][0]) + (whd * R[1][0]) + (whd * R[0][2]) + (whd * R[0][1])) {
		collision = false;
	}
	// L = Ax x By
	if ((dot(T, a[2]) * R[1][1]) - (dot(T, a[1]) * R[2][1]) > (whd * R[2][1]) + (whd * R[1][1]) + (whd * R[0][2]) + (whd * R[0][0])) {
		collision = false;
	}
	// L = Ax x Bz
	if ((dot(T, a[2]) * R[1][2]) - (dot(T, a[1]) * R[2][2]) > (whd * R[2][2]) + (whd * R[1][2]) + (whd * R[0][1]) + (whd * R[0][0])) {
		collision = false;
	}
	// L = Ay x Bx
	if ((dot(T, a[0]) * R[2][0]) - (dot(T, a[2]) * R[0][0]) > (whd * R[2][0]) + (whd * R[0][0]) + (whd * R[1][2]) + (whd * R[1][1])) {
		collision = false;
	}
	// L = Ay x By
	if ((dot(T, a[0]) * R[2][1]) - (dot(T, a[2]) * R[0][1]) > (whd * R[2][1]) + (whd * R[0][1]) + (whd * R[1][2]) + (whd * R[1][0])) {
		collision = false;
	}
	// L = Ay x Bz
	if ((dot(T, a[0]) * R[2][2]) - (dot(T, a[2]) * R[0][2]) > (whd * R[2][2]) + (whd * R[0][2]) + (whd * R[1][1]) + (whd * R[1][0])) {
		collision = false;
	}
	// L = Az x Bx
	if ((dot(T, a[1]) * R[0][0]) - (dot(T, a[0]) * R[1][0]) > (whd * R[1][0]) + (whd * R[0][0]) + (whd * R[2][2]) + (whd * R[2][1])) {
		collision = false;
	}
	// L = Az x By
	if ((dot(T, a[1]) * R[0][1]) - (dot(T, a[0]) * R[1][1]) > (whd * R[1][1]) + (whd * R[0][1]) + (whd * R[2][2]) + (whd * R[2][0])) {
		collision = false;
	}
	// L = Az x Bz
	if ((dot(T, a[1]) * R[0][2]) - (dot(T, a[0]) * R[1][2]) > (whd * R[1][2]) + (whd * R[0][2]) + (whd * R[2][1]) + (whd * R[2][0])) {
		collision = false;
	}

	//If all of the cases above are false, then there is a collision
	if (collision = true) {
		//FIX THIS
		//civ.push_back(&c1, &c2, );
	}

	cout << "Box Box" << endl;
	return collision;
}

bool IsColliding(std::vector<collisionInfo> &civ, const cCollider &c1, const cCollider &c2) {
	enum shape { UNKOWN = 0, PLANE, SPHERE, BOX };
	shape s1 = UNKOWN;
	shape s2 = UNKOWN;
	if (dynamic_cast<const cSphereCollider *>(&c1)) {
		s1 = SPHERE;
	} 
	else if (dynamic_cast<const cPlaneCollider *>(&c1)) {
		s1 = PLANE;
	} 
	else if (dynamic_cast<const cBoxCollider *>(&c1)) {
		s1 = BOX;
	}

	if (dynamic_cast<const cSphereCollider *>(&c2)) {
		s2 = SPHERE;
	} 
	else if (dynamic_cast<const cPlaneCollider *>(&c2)) {
		s2 = PLANE;
	} 
	else if (dynamic_cast<const cBoxCollider *>(&c2)) {
		s2 = BOX;
	}

	if (!s1 || !s2) {
		cout << "Routing Error" << endl;
		return false;
	}

	if (s1 == PLANE) {
		if (s2 == PLANE) {
			return IsCollidingCheck(civ, dynamic_cast<const cPlaneCollider &>(c1), dynamic_cast<const cPlaneCollider &>(c2));
		} 
		else if (s2 == SPHERE) {
			return IsCollidingCheck(civ, dynamic_cast<const cSphereCollider &>(c1), dynamic_cast<const cPlaneCollider &>(c2));
		} 
		else if (s2 == BOX) {
			return IsCollidingCheck(civ, dynamic_cast<const cPlaneCollider &>(c1), dynamic_cast<const cBoxCollider &>(c2));
		} 
		else {
			cout << "Routing Error" << endl;
			return false;
		}
	} 
	else if (s1 == SPHERE) {
		if (s2 == PLANE) {
			return IsCollidingCheck(civ, dynamic_cast<const cSphereCollider &>(c1), dynamic_cast<const cPlaneCollider &>(c2));
		} 
		else if (s2 == SPHERE) {
			return IsCollidingCheck(civ, dynamic_cast<const cSphereCollider &>(c1),
									dynamic_cast<const cSphereCollider &>(c2));
		} 
		else if (s2 == BOX) {
			return IsCollidingCheck(civ, dynamic_cast<const cSphereCollider &>(c1), dynamic_cast<const cBoxCollider &>(c2));
		} 
		else {
			cout << "Routing Error" << endl;
			return false;
		}
	} 
	else if (s1 == BOX) {
		if (s2 == PLANE) {
			return IsCollidingCheck(civ, dynamic_cast<const cPlaneCollider &>(c2), dynamic_cast<const cBoxCollider &>(c1));
		} 
		else if (s2 == SPHERE) {
			return IsCollidingCheck(civ, dynamic_cast<const cSphereCollider &>(c2), dynamic_cast<const cBoxCollider &>(c1));
		} 
		else if (s2 == BOX) {
			return IsCollidingCheck(civ, dynamic_cast<const cBoxCollider &>(c2), dynamic_cast<const cBoxCollider &>(c1));
		} 
		else {
			cout << "Routing Error" << endl;
			return false;
		}
	}

  return false;
}
}