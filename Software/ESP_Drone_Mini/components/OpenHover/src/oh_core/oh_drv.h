#ifndef _OH_DRV_H_
#define _OH_DRV_H_

typedef struct {
    // imu status
    float pitch;
    float roll;
    float yaw;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;

} oh_drv_status_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} __oh_drv_debug_target_status_t;


// range: [0, 1]
typedef struct {
    float m1;
    float m2;
    float m3;
    float m4;
} oh_drv_quadrotor_output_t;




#endif
