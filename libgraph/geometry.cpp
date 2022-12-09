#include "geometry.h"

vec3 cross(const vec3& v1, const vec3& v2) {
    return vec<3>{v1.y* v2.z - v1.z * v2.y, v1.z* v2.x - v1.x * v2.z, v1.x* v2.y - v1.y * v2.x};
}

mat<4, 4> rotateX(double angle) {
    return { {{1,0,0,0},{0,cos(angle),-sin(angle),0},{0,sin(angle),cos(angle),0},{0,0,0,1}} };
}

mat<4, 4> rotateY(double angle) {
    return { {{cos(angle),0,sin(angle),0},{0,1,0,0},{-sin(angle),0,cos(angle),0},{0,0,0,1}} };
}

mat<4, 4> rotateZ(double angle) {
    return { {{cos(angle),-sin(angle),0,0},{sin(angle),cos(angle),0,0},{0,0,1,0},{0,0,0,1}} };
}

mat<4, 4> translateXYZ(vec3 tran) {
    return { {{1,0,0,tran.x},{0,1,0,tran.y},{0,0,1,tran.z},{0,0,0,1}} };
}

mat<4, 4> scaleXYZ(vec3 scale) {
    return { {{scale.x,0,0,0},{0,scale.y,0,0},{0,0,scale.z,0 },{0,0,0,1}} };
}
