
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


/* Flight phases */

#define IDLE 0
#define LAUNCH 1
#define ASCENT 2
#define APOGEE 3
#define DESCENT 4
#define LANDING 5
#define MODULE_INIT_ERROR 69

