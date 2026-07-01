#include "Sensor_Task.h"
#include "upper_pc_protocol.h"
#include "cmsis_os2.h"

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
        }

        osDelay(2);
    }
}