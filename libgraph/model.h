#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<vec3> verts;
	std::vector<vec2> tex_coord;
	std::vector<vec3> norms;
	std::vector<int> face_vert;
	std::vector<int> face_text;
	std::vector<int> face_norm;
public:
	Model(const char* filename);
	~Model();
	int nverts() const;
	int nfaces() const;
	vec3 vert(const int i) const;
	vec3 vert(const int iface, const int nthvert) const;
	vec2 Model::uv(const int iface, const int nthvert) const;
	vec3 Model::normal(const int iface, const int nthvert) const;
};

#endif //__MODEL_H__#pragma once
