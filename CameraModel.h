//
// Created by chuguanyi on 18-3-21.
//

#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H

#include "types.h"

vec3 world2camera(vec4 Xw, mat4 T);

// $\pi(X) = x$
// from the camera coordinate to image coordinate
vec2 Project(vec3 pt, mat3 K);

vec2 world2pixel(vec4 Xw, mat4 T, mat3 K);

vec2 world2pixel(vec3 Xw, mat3 R, vec3 t, mat3 K);

#endif