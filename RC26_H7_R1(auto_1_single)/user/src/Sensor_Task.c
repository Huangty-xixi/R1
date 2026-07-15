#include "Sensor_Task.h"
#include "upper_pc_protocol.h"
#include "cmsis_os2.h"
#include "auto_mode.h"

#include <string.h>

volatile sensor_task_data_t g_sensor_task_data __attribute__((used)) = {0};

void Sensor_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        {
            const rc_odom_t *p = rc_get_latest_odom();
            (void)memcpy((void *)&g_sensor_task_data.odom, (const void *)p, sizeof(rc_odom_t));
            mid_x = g_sensor_task_data.odom.x;  
            mid_y = g_sensor_task_data.odom.y;
            mid_yaw = g_sensor_task_data.odom.yaw; 
        }

        /* by deepseek: always compute chassis center from ODOM */
        chassis_coord_update();

        rc_poll();

        osDelay(2);
    }
}