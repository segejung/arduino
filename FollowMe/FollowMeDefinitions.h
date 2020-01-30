// Blynk Auth
char auth[] = "12061c131f7044d4be343ea7cbdc87db";

// Pin variables
#define GPS_TX_PIN 6

#define BLUETOOTH_TX_PIN 10
#define BLUETOOTH_RX_PIN 11


#define MOTOR_A_EN_PIN 5
#define MOTOR_B_EN_PIN 9
#define MOTOR_A_IN_1_PIN 7 // Direction control for motor A
#define MOTOR_A_IN_2_PIN 8 // PWM control (speed) for motor A
#define MOTOR_B_IN_1_PIN 12 // Direction control for motor B to in 3
#define MOTOR_B_IN_2_PIN 4 // PWM control (speed) for motor B to in 4

// If one motor tends to spin faster than the other, add offset
#define MOTOR_A_OFFSET 100
#define MOTOR_B_OFFSET 0
#define DECLINATION_ANGLE 0.0f

// The offset of the mounting position to true north
#define COMPASS_OFFSET 0.0f

// How often the GPS should update in MS
// Keep this above 1000
#define GPS_UPDATE_INTERVAL 1000

// Number of changes in movement to timeout for GPS streaming
// Keeps the cooler from driving away if there is a problem
#define GPS_STREAM_TIMEOUT 18

// Number of changes in movement to timeout for GPS waypoints
// Keeps the cooler from driving away if there is a problem
#define GPS_WAYPOINT_TIMEOUT 45

// Definitions (don't edit these)
struct GeoLoc {
  float lat;
  float lon;
};
