
/* Available ground station commands */

#define RequestTelemetry 0
#define AvionicsReboot 1
#define GPSRestart 2
#define Ra02LoRaRestart 3
#define Buzzer10sTest 4
#define EnterPowerSaveMode 5
#define ControlSurfaceLock 6
#define ControlSurfaceUnlock 7
#define DeployParachute 8
#define DumpSavedTelemetry 9
#define OverridePhasetoPrelaunch 10
#define OverridePhasetoCountdown 11
#define OverridePhasetoFlight 12
#define OverridePhasetoRecovery 13

/* Default GY85 states*/
#define DEFGX 270
#define DEFGY 0
#define DEFGZ 0

/* Flight phases */

#define IDLE 0
#define LAUNCH 1
#define ASCENT 2
#define APOGEE 3
#define DESCENT 4
#define LANDING 5
#define LoRa_INIT_ERROR 69
#define BMP_INIT_ERROR 70
#define GPS_INIT_ERROR 71
#define GY85_INIT_ERROR 72

/* Debug states*/

// #define USE_LORA
#define USE_ESP
// #define USE_IMU
#define USE_BMP280
// #define USE_GPS
