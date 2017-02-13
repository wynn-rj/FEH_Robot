#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#include <FEHBattery.h>
#include <FEHBuzzer.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <robotdefinitions.h>
#include <math.h>



/////////////////////////////////////REGION: Declerations/////////////////////////////////////

/* Global Variable Decleration */

FEHMotor leftMotor(DT_MOTOR_L, DT_MOTOR_LV);
FEHMotor rightMotor(DT_MOTOR_R, DT_MOTOR_RV);

DigitalEncoder rightEncoder(MTR_ENCODE_R);
DigitalEncoder leftEncoder(MTR_ENCODE_L);

FEHServo servoArm(SRV_ARM);
FEHServo servoForkLift(SRV_FRK_LFT);

DigitalInputPin buttonTopLeft(BUTTON_TOP_LEFT);
DigitalInputPin buttonTopRight(BUTTON_TOP_RIGHT);
DigitalInputPin buttonBottomLeft(BUTTON_BOTTOM_LEFT);
DigitalInputPin buttonBottomRight(BUTTON_BOTTOM_RIGHT);

AnalogInputPin CdS(CDS_CELL);

/* Class declerations */

//Course Class used to for proteus to know what course it is on and extra info for that course
class Course
{
public:
    Course(char);
    Course();

private:
    char courseLetter;
};

/* Function declerations */

Course initMenu();
void extendRetractArm(bool);
void setForkLiftPos(float);
void waitToStart();
unsigned int readCdS();
void drive(float, float);
void drive(float);
void driveTilHitSide(bool, ButtonSide);
void turn(float, bool);
void turn(bool);
void driveToCoord(float, float);


//////////////////////////////////////////END REGION//////////////////////////////////////////

/*
void labTest()
{

    FEHServo servo(FEHServo::Servo0);
    AnalogInputPin CdS(FEHIO::P0_0);
    servo.SetMin(500);
    servo.SetMax(2495);

    while(true)
    {
        servo.SetDegree((56*CdS.Value()));
        LCD.WriteRC(CdS.Value(),0,0);
    }

    DigitalInputPin button1 (FEHIO::P0_0);
    DigitalInputPin button2 (FEHIO::P1_0);
    DigitalInputPin button3 (FEHIO::P2_0);
    DigitalInputPin button4 (FEHIO::P3_0);

    FEHMotor rightMotor(FEHMotor::Motor0, 12.0);
    FEHMotor leftMotor(FEHMotor::Motor1, 12.0);

    rightMotor.SetPercent(50.);
    leftMotor.SetPercent(50.);

    while (button1.Value() && button2.Value()) {}
    rightMotor.SetPercent(-50.);
    leftMotor.SetPercent(-10.);

    while (button4.Value()) {}
    leftMotor.Stop();

    while (button3.Value()) {}
    rightMotor.Stop();

    Sleep(500);

    rightMotor.SetPercent(50.);
    leftMotor.SetPercent(50.);

    while (button1.Value() && button2.Value()) {}
    rightMotor.SetPercent(-10.);
    leftMotor.SetPercent(-50.);

    while (button3.Value()) {}
    rightMotor.Stop();

    while (button4.Value()) {}
    leftMotor.Stop();

    Sleep(500);

    rightMotor.SetPercent(50.);
    leftMotor.SetPercent(50.);

    while (button1.Value() && button2.Value()) {}
    rightMotor.Stop();
    leftMotor.Stop();
}
*/

/**
 * @brief main
 *      main program method
 * @return 0
 */
int main(void)
{
    Course currentCourse;

    //CLears the LCD of anything previously on it
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);


    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log\n");

    //Determine course robot located on

    //RPS.InitializeTouchMenu();
    currentCourse = initMenu();

    //Set Servo Values
    servoArm.SetMax(SRV_MAX);
    servoArm.SetMin(SRV_MIN);

    servoForkLift.SetMax(SRV_FRK_MAX);
    servoForkLift.SetMin(SRV_FRK_MIN);

    //Closing SD log file
    SD.Printf("Closing Log");
    SD.CloseLog();

    //Ending Program
    return 0;
}

/**
 * @brief initMenu
 *       This method is used before running the course so that
 *       the robot knows which of the 8 courses it is on
 */
Course initMenu()
{
    /* Variable Decleration */
    bool noCourseSelected = true;
    int n;
    float x, y;
    float m = 0, bat_v = 0;
    Course selectedCourse;

    LCD.Clear(BLACK);

    //Create icons for main menu
    FEHIcon::Icon MAIN_T[1];
    char main_t_label[1][20] = {"LIBERTATUM VOLCANUS"};
    FEHIcon::DrawIconArray(MAIN_T, 1, 1, 1, 201, 1, 1, main_t_label, HI_C, TEXT_C);
    MAIN_T[0].Select();

    FEHIcon::Icon MAIN[6];
    char main_label[8][20] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    FEHIcon::DrawIconArray(MAIN, 4, 2, 40, 20, 1, 1, main_label, MENU_C, TEXT_C);


    LCD.SetFontColor(TEXT_C);
    LCD.WriteAt("BATT:        V", 0, 222);

    do
    {
        //Display average battery voltage to screen
        bat_v = ((bat_v*m)+Battery.Voltage());
        bat_v = bat_v/(++m);
        LCD.WriteAt(bat_v, 72, 222);
        if(bat_v < 10)
        {
            Buzzer.Buzz(10);
        }
        if (LCD.Touch(&x, &y))
        {
            //Check to see if a main menu icon has been touched
            for (n=0; n<=7; n++)
            {
                if (MAIN[n].Pressed(x, y, 0))
                {
                    char letter[2] = {main_label[n][0], ' '};
                    if (n == 0)
                        letter[0] = 'A';
                    if (n == 1)
                        letter[0] = 'B';

                    //States selected course to screen and log
                    LCD.Clear(BLACK);
                    LCD.WriteLine("Course   Selected");
                    LCD.WriteRC(letter, 0, 7);
                    LCD.WriteRC(n, 0, 20);
                    LCD.WriteRC(sqrt(4.0),0, 22);
                    SD.Printf("Selecting Course: ");
                    SD.Printf(letter);
                    SD.Printf("\n");                    

                    selectedCourse = Course(letter[0]);

                    noCourseSelected = false;
                    break;
                }
            }
        }
    } while(noCourseSelected);

    return selectedCourse;
}

/**
 * @brief extendRetractArm
 *      Sets the satelite arm to max or min
 * @param isExtending
 *      If it is true sets it to max, otherwise set to min
 */
void extendRetractArm(bool isExtending){
    if(isExtending)
    {
        servoArm.SetDegree(SERVO_FULL_EXT);
    } else {
        servoArm.SetDegree(SERVO_NO_EXT);
    }
}

void setForkLiftPos(float percent)
{
    servoForkLift.SetDegree((percent/100)*(SRV_FRK_MAX-SRV_FRK_MIN) + SRV_FRK_MIN);
}

/**
 * @brief waitToStart
 *      For the beginning for the robot to wait to start
 */
void waitToStart()
{
    while(readCdS() != RED){}
    SD.Printf("\n==STARTING COURSE==\n\n");
}

/**
 * @brief readCdS
 *      Reads the CdS cell
 * @return
 *      The color of the light, Black, Red, or Blue
 */
unsigned int readCdS()
{
    //TODO: Fix how this works
    unsigned int retColor;   //Color to return

    if(CdS.Value() > RED_LIGHT)
    {
        retColor = RED;
    } else if(CdS.Value() > BLUE_LIGHT) {
        retColor = BLUE;
    } else {
        retColor = BLACK;
    }

    LCD.Clear(retColor);
    return retColor;
}

/////////////////////////////////////REGION: Drive Functions/////////////////////////////////////


/**
 * @brief drive
 *      Set the speed of both motors seperatly
 * @param rMPercent
 *      Percent for right motor
 * @param lMPercent
 *      Percent for left motor
 */
void drive(float rMPercent, float lMPercent)
{
    if(rMPercent <= .05)
    {
        rightMotor.Stop();
    } else {
        rightMotor.SetPercent(rMPercent);
    }

    if(lMPercent <= .05)
    {
        leftMotor.Stop();
    } else {
        leftMotor.SetPercent(lMPercent);
    }
}

/**
 * @brief drive
 *      Set the speed of both motors
 * @param mPercent
 *      Percent for both motors
 */
void drive(float mPercent)
{
    drive(mPercent, mPercent);
}

/**
 * @brief driveTilHitSide
 *      Drive forward at max speed till both buttons on the side are hit
 * @param forward
 *      If the the robot is supposed to move forward
 * @param side
 *      What side the robot is to hit
 */
void driveTilHitSide(bool forward, ButtonSide side)
{
    int direction = (forward) ? 1:-1;
    drive(direction * MAX);
    switch (side) {
    case front:
        while (buttonTopLeft.Value() && buttonTopRight.Value()) {}
        break;
    case back:
        while (buttonBottomLeft.Value() && buttonBottomRight.Value()) {}
        break;
    default:
        SD.Printf("[driveTilHitSide] ERROR: Call to non defined ButtonSide Case\n");
        break;
    }
    drive(STOP);
}

/**
 * @brief turn
 *      Turn the proteus in place the specified degree to the right or left
 * @param degree
 *      How far the proteus is to turn
 * @param goRight
 *      Whether or not to turn right or left
 */
void turn(float degree, bool goRight)
{
    int rDirection = (goRight) ? 1:-1;
    int lDirection = (goRight) ? -1:1;

    drive(rDirection * MAX, lDirection * MAX);
    Sleep(1);  //Fix this
    drive(STOP);
}

/**
 * @brief turn
 * @param goRight
 */
void turn(bool goRight)
{
    int rDirection = (goRight) ? 1:-1;
    int lDirection = (goRight) ? -1:1;

    drive(rDirection * MAX, lDirection * MAX);
}

/**
 * @brief driveToCoord
 *      The robot takes the shortest path to the given coords
 * @param x
 *      The x coordinate to go to
 * @param y
 *      The y coordinate to go to
 */
void driveToCoord(float x, float y)
{
    float currHeading = RPS.Heading();
    float currX = RPS.X(),
          currY = RPS.Y();

    float distanceToTravel = sqrtf(pow(x-currX, 2) + pow(y-currY, 2));
    float directionToHead = atan2f(y-currY,x-currX) * (180.0/M_PI);
    //Switches directions to be from 0 to 360
    directionToHead = (directionToHead > 0.0 ? directionToHead: (360.0+directionToHead));
    if(directionToHead < currHeading && (currHeading - directionToHead) < 180)
    {
        turn(RIGHT);
        while(abs(RPS.Heading() - directionToHead) > .1){}
        drive(STOP);
    } else {
        turn(LEFT);
        while(abs(RPS.Heading() - directionToHead) > .1){}
        drive(STOP);
    }
    rightEncoder.ResetCounts();

    drive(MAX);
    while((rightEncoder.Counts() * (WHEEL_RAD * 2 * M_PI)) < distanceToTravel){}
    drive(STOP);

}

/////////////////////////////////////////////END REGION//////////////////////////////////////////


/////////////////////////////////////REGION: Struct Definitions/////////////////////////////////////

/**
 * @brief Course::Course
 *      Constructor for the Course Struct
 * @param courseLet
 *      A char that is used to define which
 *      course the robot is starting on
 */
Course::Course(char c)
{
    courseLetter = c;
}

Course::Course(){}

/////////////////////////////////////////////END REGION/////////////////////////////////////////////


