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
#define MTR_ENCODE_R FEHIO::P1_1
#define MTR_ENCODE_L FEHIO::P1_2
#define COUNTS_PER_REV 275

/* Servo Definitions */
#define SRV_ARM FEHServo::Servo0
#define SRV_FRK_LFT FEHServo::Servo1
#define SRV_MAX 2945
#define SRV_MIN 500
#define SRV_FRK_MAX 2945
#define SRV_FRK_MIN 500
#define SERVO_FULL_EXT 90.0
#define SERVO_NO_EXT 0.0
#define SRV_FRK_FULL_EXT 90.0
#define SRV_FRK_NO_EXT 0.0

/* IO Pin Definitions */
#define CDS_CELL FEHIO::P0_0
#define BUTTON_TOP_LEFT FEHIO::P0_1
#define BUTTON_TOP_RIGHT FEHIO::P0_2
#define BUTTON_BOTTOM_LEFT FEHIO::P0_3
#define BUTTON_BOTTOM_RIGHT FEHIO::P0_4

/* Wheel info in inches */
#define WHEEL_RAD 1.25

/* Motor Speed Definitions */
#define STOP 0.0
#define MAX 25.0
#define MAIN_TURN 25.0
#define FINE_TURN 5.0

/* CdS Color Definitions */
#define RED_LIGHT 0.0
#define BLUE_LIGHT 0.0

/* Direction Definitions */
#define NORTH 90.0
#define EAST 0.0
#define SOUTH 270.0
#define WEST 180.0

/* Button Combination Definitions */
typedef enum
{
    front,
    back
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
#define SAT_X 24
#define SAT_Y 15
#define LEV_X 13
#define LEV_Y 46
#define BUT_X 26
#define BUT_Y 61
#define COR_X 20
#define COR_Y 53
#define DEP_X 12
#define DEP_Y 21
#define RET_X 7
#define RET_Y 30
#define DEGREE_OFFSET 110.0

/* Code Dev Flags */
#define RUN_STATE_MACHINE true
#define USE_RPS true
#define QUIT_AFTER_ONE_STATE true

#endif // ROBOTDEFINITIONS_H
