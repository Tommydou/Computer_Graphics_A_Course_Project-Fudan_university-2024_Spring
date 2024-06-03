#include "Material.h"
#include <cmath>


static float max (float a,float b){
    return a>b?a:b;
}
Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
    // TODO implement Diffuse and Specular phong terms
    Vector3f res_shade = Vector3f(0,0,0);
    // diffuse

    float clamp= max(hit.normal.dot(dirToLight,hit.getNormal()),0);
    // res_shade = clamp*  lightIntensity*_diffuseColor;
    res_shade = (lightIntensity*_diffuseColor)*clamp;
    // specular

    Vector3f Cam = -ray.getDirection().normalized();


    float cos = Vector3f::dot(Cam,hit.getNormal().normalized());
    Vector3f R = (-Cam +2*hit.getNormal().normalized()*cos).normalized();
    clamp = max(hit.normal.dot(dirToLight.normalized(),R),0);
    clamp =pow(clamp,_shininess);
    // result +=Vector3f(clamp*lightIntensity.x()*hit.getMaterial()->getSpecularColor().x(),\
    //             clamp*lightIntensity.y()*hit.getMaterial()->getSpecularColor().y(),\
    //             clamp*lightIntensity.z()*hit.getMaterial()->getSpecularColor().z());

    res_shade += clamp*lightIntensity*_specularColor;


    

    return res_shade;


}
