#include "Object3D.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus < tmin) && (tminus < tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}


Plane::Plane(const Vector3f &normal, float d, Material *m) 
    : Object3D(m), _normal(normal.normalized()), _d(d) {

}

bool Plane::intersect(const Ray &r, float tmin, Hit &h) const
{
    // direction and origin
    Vector3f dir = r.getDirection();
    Vector3f orig = r.getOrigin();

    float denom = Vector3f::dot(dir,_normal);

    if (fabs(denom) < 1e-6) {
        return false;
    }

    // distance
    float t = (_d - Vector3f::dot(orig,_normal)) / denom;

    // 
    if (t < tmin || t > h.getT()) {
        return false;
    }

    // 
    h.set(t, material, _normal);
    return true;
}

bool Triangle::intersect(const Ray &r, float tmin, Hit &h) const 
{
    // TODO implement
    const float EPSILON = 1e-8;

    Vector3f dir = r.getDirection();
    Vector3f orig = r.getOrigin();
    Vector3f edge1 = _v[1] - _v[0];
    Vector3f edge2 = _v[2] - _v[0];
    Vector3f h_vec = Vector3f::cross(dir,edge2);
    float denom = Vector3f::dot(h_vec,edge1);
    if (fabs(denom) < EPSILON) {
        return false; //
    }
    float f = 1.0 / denom;
    Vector3f s = orig - _v[0];
    float u = f * Vector3f::dot(s,h_vec);
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    Vector3f q = Vector3f::cross(s,edge1);    
    float v = f*Vector3f::dot(dir,q);
    if (v < 0.0 || u + v > 1.0) {
        return false; // 光线在三角形外部
    }
    float t = f * Vector3f::dot(edge2,q);
    if (t > tmin && t < h.t) { // 光线与三角形相交
            Vector3f interpolatedNormal = (1 - u - v) * _normals[0] + u * _normals[1] + v * _normals[2];
            h.set(t, material, interpolatedNormal.normalized());
            return true;
        }
    return false;
}


Transform::Transform(const Matrix4f &m,Object3D *obj) : _object(obj) {
    // TODO implement Transfor  m constructor
    _inverseTransform = m.inverse();

}
bool Transform::intersect(const Ray &r, float tmin, Hit &h) const
{
    // TODO implement
    Vector3f transformedOrigin = (_inverseTransform * Vector4f(r.getOrigin(), 1.0f)).xyz();
    Vector3f transformedDirection = (_inverseTransform * Vector4f(r.getDirection(), 0.0f)).xyz();
    Ray transformedRay(transformedOrigin, transformedDirection);

// 
    if (_object->intersect(transformedRay, tmin, h)) {
        // 
        Vector3f transformedNormal = (_inverseTransform.transposed() * Vector4f(h.normal, 0.0f)).xyz().normalized();
        h.normal = transformedNormal;
        return true;
    }
    return false;
}