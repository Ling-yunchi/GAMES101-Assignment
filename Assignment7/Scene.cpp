//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray& ray,
	const std::vector<Object*>& objects,
	float& tNear, uint32_t& index, Object** hitObject)
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
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here

	// intersect wiht scene 
	auto inter = intersect(ray);
	if (!inter.happened) {
		return { 0,0,0 };
	}

	Vector3f L_dir(0);

	if (inter.obj->hasEmit()) {
		L_dir += inter.m->getEmission();
	}

	// uniformly sample the light
	Intersection light_inter;
	float light_pdf;
	sampleLight(light_inter, light_pdf);

	auto l2p = inter.coords - light_inter.coords;
	Ray light_ray(light_inter.coords, l2p.normalized());
	auto light_ray_inter = intersect(light_ray);
	if (light_ray_inter.obj == inter.obj) {
		// not blocked in the middle
		auto in_light_dir = -light_ray.direction;
		auto out_dir = -ray.direction;
		auto dis2 = light_ray_inter.distance * light_ray_inter.distance;

		L_dir += light_inter.emit
			* inter.m->eval(in_light_dir, out_dir, inter.normal)
			* dotProduct(in_light_dir, inter.normal)
			* dotProduct(-in_light_dir, light_inter.normal)
			/ dis2
			/ light_pdf;
	}

	Vector3f L_indir(0);
	// judge to continue trace, 3/4
	constexpr float russian_roulette = 0.75f;
	if (depth < 30 && get_random_float() < russian_roulette) {
		auto mat = inter.m;
		auto out_dir = mat->sample(ray.direction, inter.normal);
		auto pdf = mat->pdf(-ray.direction, out_dir, inter.normal);

		Ray reflect_ray(inter.coords, out_dir);
		auto reflect_inter = intersect(reflect_ray);

		if (reflect_inter.happened && !reflect_inter.obj->hasEmit()) {
			L_indir += castRay(reflect_ray, depth + 1)
				* mat->eval(-ray.direction, out_dir, inter.normal)
				* dotProduct(out_dir, inter.normal)
				/ pdf
				/ russian_roulette;
		}
	}

	return L_dir + L_indir;
}