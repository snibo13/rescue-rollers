#pragma once

#include <stdbool.h>
#include "freertos/FreeRTOS.h"

BaseType_t encoder_service(void);
#pragma message "Services included"

BaseType_t uros_service(void);

BaseType_t drive_service(void);

BaseType_t odometry_service(void);
