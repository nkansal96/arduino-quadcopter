#ifndef _UTIL_H
#define _UTIL_H

#ifndef max
#define max(A,B) ({ \
        typeof (A) _a = (A); \
        typeof (B) _b = (B); \
        (_a > _b) ? _a : _b; \
      })
#endif

#ifndef min
#define min(A,B) ({ \
        typeof (A) _a = (A); \
        typeof (B) _b = (B); \
        (_a < _b) ? _a : _b; \
      })
#endif

#ifndef pow2
#define pow2(A) ((A) * (A))
#endif

#ifndef limit
#define limit(low,v,high)   min((high), max((low), (v)))
#endif

// CONSTANTS
const int ESC_RF_PIN = 4; // right-front ESC
const int ESC_RR_PIN = 5; // right-rear ESC
const int ESC_LR_PIN = 6; // left-rear ESC
const int ESC_LF_PIN = 7; // left-front ESC
const int LED_PIN = 12;
const int BATTERY_PIN = 0;
const int LOW_BATTERY_V = 1000;

const int GYRO_CALIBRATED_LOC = 300;
const int GYRO_STRUCT_LOC = 301;

#endif
