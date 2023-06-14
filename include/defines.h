// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#define LED_RED_PIN      PA0 // RED
#define LED_GREEN_PIN    PB9 // GREEN
#define LED_YELLOW_PIN  PB8 // YELLOW
#define LED_UP_PIN       PB5 // BLUE1
#define LED_DOWN_PIN     PB4 // BLUE2

#define RECEIVER_PIN    PC15 // REMOTE

#define SENSOR1_PIN PA4
#define SENSOR2_PIN PC14

#define MPU_SCL PB6
#define MPU_SDA PB7

#define PRINT_ACCEL                 (0x01)
#define PRINT_GYRO                  (0x02)
#define PRINT_QUAT                  (0x04)
#define PRINT_EULER                 (0x08)
#define PRINT_TEMP                  (0x10)
#define PRINT_REMOTE                (0x20)
#define PRINT_SERIAL_IN             (0x40)
#define PRINT_SERIAL_OUT            (0x80)

#define LED_RED_SET                 (0x01)
#define LED_GREEN_SET               (0x02)
#define LED_YELLOW_SET              (0x04)
#define LED_UP_SET                  (0x08)
#define LED_DOWN_SET                (0x10)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

#endif //  DEFINES_H
