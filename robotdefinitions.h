#ifndef ROBOTDEFINITIONS_H
#define ROBOTDEFINITIONS_H

/* Define colors for parts of menus */
#define MENU_C WHITE
#define TEXT_C GOLD
#define SELT_C RED
#define SHOW_C BLUE
#define HI_C GREEN

/* Misc Definitions */
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* Motor Definitions */
#define DT_MOTOR_L FEHMotor::Motor0
#define DT_MOTOR_R FEHMotor::Motor1
#define DT_MOTOR_LV 12.0
#define DT_MOTOR_RV 12.0
#define MTR_ENCODE_R FEHIO::P0_6
#define MTR_ENCODE_L FEHIO::P0_7
#define COUNTS_PER_REV 318
#define PID_MAX_DIFF 5.0

/* Servo Definitions */
#define SRV_ARM FEHServo::Servo0
#define SRV_FRK_LFT FEHServo::Servo1
#define SRV_MAX 2215
#define SRV_MIN 710
#define SRV_FRK_MAX 2215
#define SRV_FRK_MIN 710
#define SERVO_FULL_EXT 90.0
#define SERVO_NO_EXT 0.0
#define SRV_FRK_FULL_EXT 160.0
#define SRV_FRK_NO_EXT 0.0

/* IO Pin Definitions */
#define CDS_CELL FEHIO::P1_0
#define BUTTON_TOP_LEFT FEHIO::P3_7
#define BUTTON_TOP_RIGHT FEHIO::P0_0
#define BUTTON_BOTTOM_LEFT FEHIO::P3_6
#define BUTTON_BOTTOM_RIGHT FEHIO::P0_1

/* Wheel info in inches */
#define WHEEL_RAD 1.25

/* Motor Speed Definitions */
#define STOP 0.0
#define MAX 35.0
#define TURN_MAIN 25.0
#define TURN_FINE 5.0
#define LINE_FOLLOW 10.0
#define LINE_FOLLOW_STRAIGHT 20.0
#define MOTOR_SPEED_RAMP_TIME 0.01

/* CdS Color Definitions */
#define BLACK_LIGHT 2.0
#define BLUE_LIGHT 1.0

/* Direction Definitions */
#define NORTH 90.0
#define EAST 0.0
#define SOUTH 270.0
#define WEST 180.0

/* Button Combination Definitions */
typedef enum
{
    FRONT,
    BACK
} ButtonSide;

/* Robot State Defintions */
typedef enum
{
    waitToStart,
    //Satelite
    startMoveSat,
    moveToSat,
    interactSat,
    //Lever
    startMoveLever,
    moveToLever,
    interactLever,
    //Siesmograph Button
    startMoveSismoBut,
    moveToSismoBut,
    interactSismoBut,
    //Core
    startMoveCore,
    moveToCore,
    interactCore,
    //Deposit Core
    startMoveDepCore,
    moveToDepCore,
    interactDepCore,
    //Return
    startMoveRet,
    moveToRet,

    shutdown

} State;

/* Motor Turn Definitions */
#define RIGHT true
#define LEFT false
#define HEAD_ERR 5.0
#define HEAD_ERR_FINE 1.0

/* Course Coordinates */
#define SAT_X 29.5
#define SAT_Y 15.0
#define LEV_X 12.0
#define LEV_Y 45.0
#define BUT_X 25.0
#define BUT_Y 54.0
#define COR_X 18.5
#define COR_Y 54.0
#define DEP_X 12.0
#define DEP_Y 15.5
#define RET_X 6.0
#define RET_Y 27.0
#define CEN_X 18.0
#define CEN_Y 45.0
#define DEGREE_OFFSET 0.0

/* Code Dev Flags */
#define RUN_STATE_MACHINE true
#define USE_RPS false
#define QUIT_AFTER_ONE_STATE true
#define INIT_CHECK true
#define PID_ALLOW false

#endif // ROBOTDEFINITIONS_H
