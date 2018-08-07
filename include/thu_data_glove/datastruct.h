#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <stdint.h>

typedef struct{
	uint32_t sensor_id;

	float yaw;
	float pitch;
	float roll;

	float Ax;
	float Ay;
	float Az;

	float Wx;
	float Wy;
	float Wz;

	float Mx;
	float My;
	float Mz;
} SensorData_single;

typedef struct{
	uint32_t gloveID;
	float timestemp;
	SensorData_single sensor_data[18];
} SensorData;

/*        name:freedom
 *
 *       f2:1 m2:1 r2:1 l2:1
 *        |    |    |    |
 *  t2:1 f1:1 m1:1 r1:1 l1:1
 *   |    |    |    |    |
 *  t1:1 f0:2 m0:2 r0:2 l0:2
 *     \
 *     t0:2   a0:1
 *             |
 *            a1:2
 *             |
 *            b0:3
 */


typedef struct {
	uint32_t gloveID;
	float timestemp;
	struct {
		float t00, t01, t1, t2;
	} thumb;
	struct {
		float f00, f01, f1, f2;
	} forefinger;
	struct {
		float m00, m01, m1, m2;
	} middlefinger;
	struct {
		float r00, r01, r1, r2;
	} ringfinger;
	struct {
		float l00, l01, l1, l2;
	} littlefinger;
	struct {
		float a0, a10, a11;
	} arm;
	struct {
		float x, y, z, w;
	} base;
} GloveData;

#endif // DATASTRUCT_H
