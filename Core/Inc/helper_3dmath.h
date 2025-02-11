#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <math.h>
#include <stdint.h>
#define PI 3.141592654

// Quaternion structure
typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;

// Quaternion functions
static inline void Quaternion_init(Quaternion *q, float nw, float nx, float ny, float nz) {
  q->w = nw;
  q->x = nx;
  q->y = ny;
  q->z = nz;
}

static inline Quaternion Quaternion_getProduct(const Quaternion *q1, const Quaternion *q2) {
  // Quaternion multiplication is defined by:
  //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
  //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
  //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
  //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
  Quaternion result;
  result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
  result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
  return result;
}

static inline Quaternion Quaternion_getConjugate(const Quaternion *q) {
  return (Quaternion){q->w, -q->x, -q->y, -q->z};
}

static inline float Quaternion_getMagnitude(const Quaternion *q) {
  return sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

static inline void Quaternion_normalize(Quaternion *q) {
  float mag = Quaternion_getMagnitude(q);
  q->w /= mag;
  q->x /= mag;
  q->y /= mag;
  q->z /= mag;
}

// VectorInt16 structure
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} VectorInt16;

// VectorInt16 functions
static inline float VectorInt16_getMagnitude(const VectorInt16 *v) {
  return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static inline void VectorInt16_normalize(VectorInt16 *v) {
  float mag = VectorInt16_getMagnitude(v);
  v->x /= mag;
  v->y /= mag;
  v->z /= mag;
}

// VectorFloat structure
typedef struct {
  float x;
  float y;
  float z;
} VectorFloat;

// VectorFloat functions
static inline float VectorFloat_getMagnitude(const VectorFloat *v) {
  return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

static inline void VectorFloat_normalize(VectorFloat *v) {
  float mag = VectorFloat_getMagnitude(v);
  v->x /= mag;
  v->y /= mag;
  v->z /= mag;
}

#endif /* _HELPER_3DMATH_H_ */
