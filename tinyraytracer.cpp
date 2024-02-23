#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "model.h"
#include "geometry.h"

int envmap_width, envmap_height;
std::vector<Vec3f> envmap;
Model duck("../duck.obj");
Model button1("../boutonF.obj");
Model button2("../boutonF.obj");
Model button3("../boutonF.obj");
Model cap("../chapeau.obj");
Model batD("../batD.obj");
Model batG("../batG.obj");
Model scarf("../scarf.obj");
Model baguette("../Baguette.obj");


std::vector<Model> models;

struct Light {
    Light(const Vec3f &p, const float i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material {
    Material(const float r, const Vec4f &a, const Vec3f &color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1,0,0,0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere {
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const {
        Vec3f L = center - orig;
        float tca = L*dir;
        float d2 = L*L - tca*tca;
        if (d2 > radius*radius) return false;
        float thc = sqrtf(radius*radius - d2);
        t0       = tca - thc;
        float t1 = tca + thc;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }
};

Vec3f reflect(const Vec3f &I, const Vec3f &N) {
    return I - N*2.f*(I*N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? Vec3f(1,0,0) : I*eta + N*(eta*cosi - sqrtf(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material) {
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i=0; i < spheres.size(); i++) {
        float dist_i;
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) {
            spheres_dist = dist_i;
            hit = orig + dir*dist_i;
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
    

   float bouton_dist = std::numeric_limits<float>::max();
   for(size_t j=0; j < models.size(); j++){
     for (int i=0; i<models[j].nfaces(); i++) {
        float dist_i;
        if (models[j].ray_triangle_intersect(i, orig, dir, dist_i) && dist_i < bouton_dist && dist_i < spheres_dist && dist_i) {
            bouton_dist = dist_i;
            hit = orig + dir*dist_i;
            N = cross(models[j].point(models[j].vert(i,1))-models[j].point(models[j].vert(i,0)), models[j].point(models[j].vert(i,2))-models[j].point(models[j].vert(i,0))).normalize();
            material.diffuse_color = Vec3f(0.3, 0.1, 0.1);
        }
    }

   }
   

    float checkerboard_dist = std::numeric_limits<float>::max();
    if (fabs(dir.y)>1e-3)  {
        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
        Vec3f pt = orig + dir*d;
        if (d>0 && fabs(pt.x)<40 && pt.z<30 && pt.z>-50 && d<spheres_dist && d<bouton_dist  ){
            checkerboard_dist = d;
            hit = pt;
            N = Vec3f(0,1,0);
            material.diffuse_color = (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 ? Vec3f(.3, .3, .3) : Vec3f(.3, .3 ,.3);
        }
    }
    
    return std::min({spheres_dist, bouton_dist,checkerboard_dist })<1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth=0) {
    Vec3f point, N;
    Material material;


    if (depth>4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
        /*return Vec3f(0.9, 0.9, 0.9);*/
        float theta = acos(dir.y);
        float phi = atan2(dir.z, dir.x);
        int longitude = ((phi)/(0.5+M_PI))*(envmap_width-1);
        int latitude = (theta/M_PI)*(envmap_height-1);
        return envmap[longitude+(latitude*envmap_width)];
    }

    Vec3f reflect_dir = reflect(dir, N).normalize();
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f reflect_orig = reflect_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // offset the original point to avoid occlusion by the object itself
    Vec3f refract_orig = refract_dir*N < 0 ? point - N*1e-3 : point + N*1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();
        float light_distance = (lights[i].position - point).norm();

        Vec3f shadow_orig = light_dir*N < 0 ? point - N*1e-3 : point + N*1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt-shadow_orig).norm() < light_distance)
            continue;

        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent)*lights[i].intensity;
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.)*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];

}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights) {
    
    const int   width    = 1024;
    const int   height   = 768;
    const float fov      = M_PI/3.;
    std::vector<Vec3f> framebuffer(width*height);
    

   /*
    const float eyesep   = 0.2;
    const int   delta    = 60; // focal distance 3
    const int   width    = 1024+delta;
    const int   height   = 768;
    const float fov      = M_PI/3.;

    std::vector<Vec3f> framebuffer1(width*height);
    std::vector<Vec3f> framebuffer2(width*height);
*/
#pragma omp parallel for
    for (size_t j = 0; j<height; j++) { // actual rendering loop
        for (size_t i = 0; i<width; i++) {
            float dir_x =  (i + 0.5) -  width/2.;
            float dir_y = -(j + 0.5) + height/2.;    // this flips the image at the same time
            float dir_z = -height/(2.*tan(fov/2.));
            framebuffer[i+j*width] = cast_ray(Vec3f(0,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
            //framebuffer1[i+j*width] = cast_ray(Vec3f(-eyesep/2,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
            //framebuffer2[i+j*width] = cast_ray(Vec3f(+eyesep/2,0,0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
        }
    }

    
    std::vector<unsigned char> pixmap(width*height*3);
    for (size_t i = 0; i < height*width; ++i) {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        if (max>1) c = c*(1./max);
        for (size_t j = 0; j<3; j++) {
            pixmap[i*3+j] = (unsigned char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    stbi_write_jpg("out.jpg", width, height, 3, pixmap.data(), 100);
    
   /*
    std::vector<unsigned char> pixmap((width-delta)*height*3);
    for (size_t j = 0; j<height; j++) {
        for (size_t i = 0; i<width-delta; i++) {
            Vec3f c1 = framebuffer1[i+delta+j*width];
            Vec3f c2 = framebuffer2[i+      j*width];
            float max1 = std::max(c1[0], std::max(c1[1], c1[2]));
            if (max1>1) c1 = c1*(1./max1);
            float max2 = std::max(c2[0], std::max(c2[1], c2[2]));
            if (max2>1) c2 = c2*(1./max2);
            float avg1 = (c1.x+c1.y+c1.z)/3.;
            float avg2 = (c2.x+c2.y+c2.z)/3.;
            pixmap[(j*(width-delta) + i)*3  ] = 255*avg1;
            pixmap[(j*(width-delta) + i)*3+1] = 0;
            pixmap[(j*(width-delta) + i)*3+2] = 255*avg2;
        }
    }
    stbi_write_jpg("out.jpg", width-delta, height, 3, pixmap.data(), 100);
    */
}

int main() {
    int n = -1;
    unsigned char *pixmap = stbi_load("../envmap.jpg", &envmap_width, &envmap_height, &n, 0);
    if (!pixmap || 3!=n) {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;
    }
    envmap = std::vector<Vec3f>(envmap_width*envmap_height);
    for (int j = envmap_height-1; j>=0 ; j--) {
        for (int i = 0; i<envmap_width; i++) {
            envmap[i+j*envmap_width] = Vec3f(pixmap[(i+j*envmap_width)*3+0], pixmap[(i+j*envmap_width)*3+1], pixmap[(i+j*envmap_width)*3+2])*(1/255.);
        }
    }
    stbi_image_free(pixmap);

    Material      ivory(1.0, Vec4f(0.6,  0.3, 0.1, 0.0), Vec3f(0.0, 0.0, 0.0),   50.);
    //Material      glass(1.5, Vec4f(0.0,  0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8),  125.);
    Material red_rubber(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1),   10.);
    Material snow(1.0, Vec4f(0.9,  0.1, 0.0, 0.0), Vec3f(1.0, 1.0, 1.0),   10.);
    //Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);

    button1.translate(-0.7, -2, -17.5);
    button2.translate(-0.7, -1.0, -17);
    button3.translate(-0.7, -0.2, -16.5);
    //batG.translate(-2, -0.2, -16.5);
    //batD.translate(2, -0.2, -16.5);
    //cap.translate(-0.7, 3.5, -16.5);

    std::vector<Sphere> spheres;
    //spheres.push_back(Sphere(Vec3f(-3,    0,   -16), 2,      ivory));
    //spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2,      glass));
    spheres.push_back(Sphere(Vec3f( -1.0, -1.5, -22), 3, snow));
    spheres.push_back(Sphere(Vec3f( -1.0, 2, -21), 2, snow));
    spheres.push_back(Sphere(Vec3f( -1.3, 2.5, -19.1), 0.2, ivory));
    spheres.push_back(Sphere(Vec3f( -0.3, 2.5, -19.1), 0.2, ivory));
    spheres.push_back(Sphere(Vec3f( -1.8, 1.4, -19.2), 0.2, ivory));
    spheres.push_back(Sphere(Vec3f( -1.2, 1.2, -19.2), 0.2, ivory));
    spheres.push_back(Sphere(Vec3f( -0.5, 1.2, -19.2), 0.2, ivory));
    spheres.push_back(Sphere(Vec3f( 0.1, 1.4, -19.2), 0.2, ivory));

    models.push_back(button1);
    models.push_back(button2);
    models.push_back(button3);
    //models.push_back(batD);
    //models.push_back(batG);
    //models.push_back(cap);
   
    //spheres.push_back(Sphere(Vec3f( 7,    5,   -18), 4,     mirror));

    std::vector<Light>  lights;
    //lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));
    //lights.push_back(Light(Vec3f( 30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f( 30, 20,  30), 1.7));

    render(spheres, lights);

    return 0;
}

