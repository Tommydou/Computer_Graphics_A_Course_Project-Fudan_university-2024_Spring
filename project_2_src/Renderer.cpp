#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    Image k3image(3*w,3*h);
    Image k3nimage(3*w,3*h);
    Image k3dimage(3*w,3*h); // 提高分辨率到三倍，用于高斯滤波抗锯齿处理

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.

    Camera* cam = _scene.getCamera();
    for (int y = 0; y < 3*h; ++y) { // 三倍分辨率，抖动采样
        float ndcy = 2 * (y / (3*h - 1.0f)) - 1.0f;
        for (int x = 0; x < 3*w; ++x) {
            float ndcx = 2 * (x / (3*w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            Ray r = cam->generateRay(Vector2f(ndcx, ndcy));

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h);

            k3image.setPixel(x, y, color);
            k3nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                k3dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }
    // END SOLN
    for(int y = 0; y < h; ++y){ // 高斯滤波
        for(int x = 0; x < w; ++x){
            int c_x = 3 * (x + 1) -2;
            int c_y = 3 * (y + 1) -2;
            Vector3f value = k3image.getPixel(c_x,c_y) / 4
                            + k3image.getPixel(c_x+1,c_y)/8 + k3image.getPixel(c_x-1,c_y)/8 
                            + k3image.getPixel(c_x,c_y+1)/8 + k3image.getPixel(c_x,c_y-1)/8
                            + k3image.getPixel(c_x-1,c_y-1)/16 + k3image.getPixel(c_x+1,c_y-1)/16 
                            + k3image.getPixel(c_x-1,c_y+1)/16 + k3image.getPixel(c_x+1,c_y+1)/16;
            image.setPixel(x,y,value);

            Vector3f dvalue = k3dimage.getPixel(c_x,c_y) / 4
                            + k3dimage.getPixel(c_x+1,c_y)/8 + k3dimage.getPixel(c_x-1,c_y)/8 
                            + k3dimage.getPixel(c_x,c_y+1)/8 + k3dimage.getPixel(c_x,c_y-1)/8
                            + k3dimage.getPixel(c_x-1,c_y-1)/16 + k3dimage.getPixel(c_x+1,c_y-1)/16 
                            + k3dimage.getPixel(c_x-1,c_y+1)/16 + k3dimage.getPixel(c_x+1,c_y+1)/16;
            dimage.setPixel(x,y,dvalue);

            Vector3f nvalue = k3nimage.getPixel(c_x,c_y) / 4
                            + k3nimage.getPixel(c_x+1,c_y)/8 + k3nimage.getPixel(c_x-1,c_y)/8 
                            + k3nimage.getPixel(c_x,c_y+1)/8 + k3nimage.getPixel(c_x,c_y-1)/8
                            + k3nimage.getPixel(c_x-1,c_y-1)/16 + k3nimage.getPixel(c_x+1,c_y-1)/16 
                            + k3nimage.getPixel(c_x-1,c_y+1)/16 + k3nimage.getPixel(c_x+1,c_y+1)/16;
            nimage.setPixel(x,y,nvalue);
        }
    }

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}


#include <iostream> // 添加此行以确保可以使用 std::cerr

Vector3f
Renderer::traceRay(const Ray &r,
    float tmin,
    int bounces,
    Hit &h) const
{
    // The starter code only implements basic drawing of sphere primitives.
    // You will implement phong shading, recursive ray tracing, and shadow rays.

    // TODO: IMPLEMENT 
    Vector3f direct(0,0,0);
    Vector3f indirect(0,0,0);

    if (_scene.getGroup()->intersect(r, tmin, h)) {

        int num_lights = _scene.getNumLights();
        Vector3f ambient_light = _scene.getAmbientLight();

        Vector3f dirtolight(0);
        Vector3f lightintensity(0);

        for(int i = 0; i < num_lights; i++){

            Vector3f intersectionPoint = r.pointAtParameter(h.getT());
            float dist = 0;

            _scene.getLight(i)->getIllumination(intersectionPoint, dirtolight,lightintensity, dist);
            
            // 阴影投射
            Vector3f intersect_point = r.pointAtParameter(h.getT());
            Vector3f tolight = dirtolight; 
            Ray shadow(intersect_point,tolight);
            Hit new_hit;
            if(_args.shadows == 0) direct += h.getMaterial()->shade(r,h,dirtolight,lightintensity);
            if(_args.shadows != 0 && !_scene.getGroup()->intersect(shadow,0.0001,new_hit))
                direct += h.getMaterial()->shade(r,h,dirtolight,lightintensity);
        }
        direct += ambient_light;

        // 光线追踪
        if(bounces > 0){
            Vector3f normal = h.getNormal().normalized();
            Vector3f new_origin = r.getOrigin() + r.getDirection() * h.getT();
            Vector3f Intertoview = -r.getDirection();
            Vector3f temp = normal.dot(normal, Intertoview) * normal;
            Vector3f Reflectdir = 2 * temp - Intertoview;
            Reflectdir = Reflectdir.normalized();

            Hit h1;

            Ray new_ray(new_origin + Reflectdir / 100, Reflectdir);

            indirect = traceRay(new_ray,tmin,bounces-1,h1);; 

            Vector3f specular = h.getMaterial()->getSpecularColor();
            indirect[0] = specular[0] * indirect[0];
            indirect[1] = specular[1] * indirect[1];
            indirect[2] = specular[2] * indirect[2];

        }

    }
    else direct = _scene.getBackgroundColor(r.getDirection());
    
    return direct + indirect;
    
}
