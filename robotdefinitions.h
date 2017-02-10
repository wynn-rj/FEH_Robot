#ifndef ROBOTDEFINITIONS_H
#define ROBOTDEFINITIONS_H

/* Define colors for parts of menus */
#define MENU_C WHITE
#define TEXT_C GOLD
#define SELT_C RED
#define SHOW_C BLUE
#define HI_C GREEN

/* Motor Definitions */
#define DT_MOTOR_L FEHMotor::Motor0
#define DT_MOTOR_R FEHMotor::Motor1
#define DT_MOTOR_LV 12.0
#define DT_MOTOR_RV 12.0

/* Servo Definitions */
#define SRV_ARM FEHServo::Servo0
#define SRV_MAX 2945
#define SRV_MIN 500

/* IO Pin Definitions */
#define CDS_CELL FEHIO::P0_0
#define BUTTON_TOP_LEFT FEHIO::P0_1
#define BUTTON_TOP_RIGHT FEHIO::P0_2
#define BUTTON_BOTTOM_LEFT FEHIO::P0_3
#define BUTTON_BOTTOM_RIGHT FEHIO::P0_4

#endif // ROBOTDEFINITIONS_H
