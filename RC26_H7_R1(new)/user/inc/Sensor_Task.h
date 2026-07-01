#ifndef __SENSOR_TASK_H__
#define __SENSOR_TASK_H__

#include "upper_pc_protocol.h"

typedef struct {
    rc_odom_t odom;
} sensor_task_data_t;

extern volatile sensor_task_data_t g_sensor_task_data;

void Sensor_Task(void *argument);

#endif /* __SENSOR_TASK_H__ */