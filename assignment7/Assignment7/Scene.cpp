//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = intersect(ray);
    if (!p.happened) return Vector3f();
    if (p.m->hasEmission()) return p.m->getEmission();

    Vector3f wo = -ray.direction;

    Vector3f L_dir;
    Intersection x;
    float pdf_light;
    sampleLight(x,pdf_light);
    Vector3f ws = normalize(x.coords - p.coords);
    Intersection is_blocked = intersect(Ray(p.coords,ws));
	
    if((is_blocked.distance < (x.coords-p.coords).norm()) )
    {
        float d2 = pow((x.coords - p.coords).norm(),2);
        L_dir = x.emit * p.m->eval(ws,wo,p.normal)*dotProduct(ws,p.normal)*
                dotProduct(-ws,x.normal) / (d2*pdf_light);
    }
	// indirect Illumination
    Vector3f L_indir;
    if(get_random_float() < RussianRoulette)
    {
        Vector3f wi = normalize(p.m->sample(wo,p.normal));
        Intersection hit = intersect(Ray(p.coords,wi));
        if(hit.happened && (hit.emit.norm() < EPSILON))
        {
            L_indir = castRay(Ray(p.coords,wi),depth) * p.m->eval(wi,wo,p.normal)
                    * dotProduct(wi,p.normal) / (p.m->pdf(wi,wo,p.normal)*RussianRoulette);
        }
    }
    L_indir = L_indir * 1.0 / RussianRoulette;
    
    return L_dir + L_indir;
}