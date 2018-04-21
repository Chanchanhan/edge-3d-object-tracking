//
// Created by chuguanyi on 18-3-21.
//

#include "CameraModel.h"

vec3 world2camera(vec4 Xw, mat4 T)
{
	vec4 Xc = T * Xw;
	Xc = Xc / Xc[3];
	return { Xc[0], Xc[1], Xc[2] };
}

// $\pi(X) = x$
// from the camera coordinate to image coordinate
vec2 Project(vec3 pt, mat3 K)
{
	if (pt[2] <= 0)
		return { -1,-1 };
	pt = pt / pt[2];
	auto p = K * pt;
	p = p / pt[2];
	return { p[0],p[1] };
}

vec2 world2pixel(vec4 Xw, mat4 T, mat3 K)
{
	return Project(world2camera(Xw, T), K);
}

vec2 world2pixel(vec3 Xw, mat3 R, vec3 t, mat3 K)
{

	return Project(R * Xw + t, K);
}