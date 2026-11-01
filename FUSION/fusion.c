#include "fusion.h"
#include <math.h>
#include "stm32f4xx_hal.h"

//float version of math.h double pi constant
#define M_PI_F 3.14159265358979323846f
#define RAD_TO_DEG 57.295779513f
static float kp=2.0f, ki=0.0137f;
static float e_int[] = {0,0,0};
static const float delta_t = 1.0f/208.0f;

static uint32_t start_time_ms = 0;
static uint32_t transient_time_ms = 0;
static uint16_t settled_counter = 0;
static uint8_t  is_settled = 0;

void AHRS_Reset_Benchmark(void) {
    start_time_ms = HAL_GetTick();
    settled_counter = 0;
    is_settled = 0;
    transient_time_ms = 0;
}

uint8_t AHRS_IsSettled(void) {
    return is_settled;
}

uint32_t AHRS_GetTransientTimeMs(void) {
    return transient_time_ms;
}

// Internal helper called directly inside Mahony_Update()
static void check_transient_internal(float ex, float ey, float ez) {
    if (is_settled) return;

    float error_sq = (ex * ex) + (ey * ey) + (ez * ez);

    if (error_sq < 0.0001f) { // ~0.57 deg threshold
        settled_counter++;
        if (settled_counter >= 20) { // 20 consecutive frames (~100 ms)
            transient_time_ms = HAL_GetTick() - start_time_ms;
            is_settled = 1;
        }
    } else {
        settled_counter = 0;
    }
}


void Mahony_Update(float *q, float *accel, float *gyro, float *mag, uint8_t mag_data_ready) {
	//scalar variable declarations to utilize the fpu
	float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
	float ax = accel[0], ay = accel[1], az = accel[2];
	float ex = 0.0f, ey = 0.0f, ez = 0.0f;
	float q_dot[4];
	float accel_norm_factor, mag_norm_factor, q_norm_factor;
	float gx_corrected, gy_corrected, gz_corrected;

	//precompute common quaternion products
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	//normalize accelerometer
	accel_norm_factor = sqrtf(ax*ax + ay*ay + az*az);
	if (accel_norm_factor > 0.0f) {
		float inv_accel = 1.0f / accel_norm_factor;
		ax = -ax * inv_accel;
		ay = -ay * inv_accel;
		az = -az * inv_accel;

		//estimated gravity vector
		float gx = 2.0f * (q1q3 - q0q2);
		float gy = 2.0f * (q0q1 + q2q3);
		float gz = q0q0 - q1q1 - q2q2 + q3q3;

		//accel error cross product
		ex += ay*gz - az*gy;
		ey += az*gx - ax*gz;
		ez += ax*gy - ay*gx;
	}

	if(mag_data_ready){
		float magx = mag[0], magy = mag[1], magz = mag[2];

		mag_norm_factor = sqrtf(magx*magx + magy*magy + magz*magz);
		if (mag_norm_factor > 0.0f) {
			float inv_mag = 1.0f / mag_norm_factor;
			magx *= inv_mag;
			magy *= inv_mag;
			magz *= inv_mag;

			//rotate body-frame mag vector into earth frame
			float hx = 2.0f * magx * (0.5f - q2q2 - q3q3) + 2.0f * magy * (q1q2 - q0q3) + 2.0f * magz * (q1q3 + q0q2);
			float hy = 2.0f * magx * (q1q2 + q0q3) + 2.0f * magy * (0.5f - q1q1 - q3q3) + 2.0f * magz * (q2q3 - q0q1);
			float hz = 2.0f * magx * (q1q3 - q0q2) + 2.0f * magy * (q2q3 + q0q1) + 2.0f * magz * (0.5f - q1q1 - q2q2);

			//align with earth reference
			hx = sqrtf(hx*hx + hy*hy);
			hy = 0.0f;

			//rotate reference back into body frame
			float wx = 2.0f * hx * (0.5f - q2q2 - q3q3) + 2.0f * hz * (q1q3 - q0q2);
			float wy = 2.0f * hx * (q1q2 - q0q3) + 2.0f * hz * (q0q1 + q2q3);
			float wz = 2.0f * hx * (q0q2 + q1q3) + 2.0f * hz * (0.5f - q1q1 - q2q2);

			//mag error cross product
			ex += magy*wz - magz*wy;
			ey += magz*wx - magx*wz;
			ez += magx*wy - magy*wx;
		}
	}

	check_transient_internal(ex, ey, ez);

	e_int[0] += ex * delta_t;
	e_int[1] += ey * delta_t;
	e_int[2] += ez * delta_t;

	gx_corrected = gyro[0] + kp*ex + ki*e_int[0];
	gy_corrected = gyro[1] + kp*ey + ki*e_int[1];
	gz_corrected = gyro[2] + kp*ez + ki*e_int[2];

	q_dot[0] = 0.5f * (-q1*gx_corrected - q2*gy_corrected - q3*gz_corrected);
	q_dot[1] = 0.5f * ( q0*gx_corrected + q2*gz_corrected - q3*gy_corrected);
	q_dot[2] = 0.5f * ( q0*gy_corrected - q1*gz_corrected + q3*gx_corrected);
	q_dot[3] = 0.5f * ( q0*gz_corrected + q1*gy_corrected - q2*gx_corrected);

	q0 += q_dot[0]*delta_t;
	q1 += q_dot[1]*delta_t;
	q2 += q_dot[2]*delta_t;
	q3 += q_dot[3]*delta_t;

	q_norm_factor = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (q_norm_factor > 0.0f) {
		float inv_q = 1.0f / q_norm_factor;
		q[0] = q0 * inv_q;
		q[1] = q1 * inv_q;
		q[2] = q2 * inv_q;
		q[3] = q3 * inv_q;
	}
}

void Mahony_GetEuler(const float *q, float *roll, float *pitch, float *yaw) {
	float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	//roll
	*roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

	//pitch
	float sin_p = 2.0f * (q0 * q2 - q1 * q3);
	if (sin_p >= 1.0f) {
		*pitch = 90.0f;
	} else if (sin_p <= -1.0f) {
		*pitch = -90.0f;
	} else {
		*pitch = asinf(sin_p) * RAD_TO_DEG;
	}

	//yaw
	*yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}


void Mahony_Init(float *q, float *accel, float *mag) {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    float ax = -accel[0], ay = -accel[1], az = -accel[2];
    float magx = mag[0], magy = mag[1], magz = mag[2];

    //normalize accelerometer reading with zero guard
    float a_sq = ax * ax + ay * ay + az * az;
    if (a_sq > 0.0f) {
        float inv_a = 1.0f / sqrtf(a_sq);
        ax *= inv_a;
        ay *= inv_a;
        az *= inv_a;
    }

    //normalize magnetometer reading with zero guard
    float m_sq = magx * magx + magy * magy + magz * magz;
    if (m_sq > 0.0f) {
        float inv_m = 1.0f / sqrtf(m_sq);
        magx *= inv_m;
        magy *= inv_m;
        magz *= inv_m;
    }

    //calculate initial roll and pitch from accelerometer
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    //tilt-compensate magnetometer readings into horizontal plane
    float cos_r = cosf(roll);
    float sin_r = sinf(roll);
    float cos_p = cosf(pitch);
    float sin_p = sinf(pitch);

    float Xm = magx * cos_p + magy * sin_r * sin_p + magz * cos_r * sin_p;
    float Ym = magy * cos_r - magz * sin_r;

    //calculate yaw heading relative to magnetic north
    yaw = atan2f(-Ym, Xm);

    //convert euler angles to initial quaternion
    float cos_r2 = cosf(roll * 0.5f);
    float sin_r2 = sinf(roll * 0.5f);
    float cos_p2 = cosf(pitch * 0.5f);
    float sin_p2 = sinf(pitch * 0.5f);
    float cos_y2 = cosf(yaw * 0.5f);
    float sin_y2 = sinf(yaw * 0.5f);

    q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;

    //normalize initial quaternion
    float q_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (q_sq > 0.0f) {
        float inv_q = 1.0f / sqrtf(q_sq);
        q[0] *= inv_q;
        q[1] *= inv_q;
        q[2] *= inv_q;
        q[3] *= inv_q;
    }
}
