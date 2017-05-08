#ifndef __MPU_H
#define __MPU_H

#include "stm32f4xx_hal.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"

extern void Error_Handler(void);
extern struct hal_s hal;

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
		unsigned long next_data_tx_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};

extern struct hal_s hal;

void handle_input(void);
void Mpu_data_refresh(void);
void mpu_init_all(void);
void read_from_mpl(void);
void run_self_test(void);
void setup_gyro(void);


#ifdef COMPASS_ENABLED

void send_status_compass(void)

#endif

#endif /* __MAIN_H */
