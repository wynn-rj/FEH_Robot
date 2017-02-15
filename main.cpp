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

/* Class declerations */

class Coord
{
public:
    float x, y;
    Coord(float, float);
    Coord();
};

//Course Class used to for proteus to know what course it is on and extra info for that course
class Course
{
public:
    Course(char);
    Course();
    Coord satelite, lever, seismoButton, core, coreDepo, home;

private:
    char courseLetter;
};


/* Function declerations */

Course initMenu();
void queState(State);
void extendRetractArm(bool);
void setForkLiftPos(float);
unsigned int readCdS();
void drive(float, float);
void drive(float);
bool checkTouchingSide(ButtonSide);
void turn(bool);
void rotateTo(float);
void driveToCoord(Coord);


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

bool runningCourse;
State currentState;

float distanceToTravel;
Coord destination;

int numberOfStateChanges = 0;

const char STATE_NAMES[19][20] = {"waitToStart", "startMoveSat", "moveToSat", "interactSat",
                                 "startMoveLever", "moveToLever", "interactLever", "startMoveSismoBut",
                                 "moveToSismoBut", "interactSismoBut", "startMoveCore", "moveToCore",
                                 "interactCore", "startMoveDepCore", "moveToDepCore", "interactDepCore",
                                 "startMoveRet", "moveToRet", "shutdown"};


//////////////////////////////////////////END REGION//////////////////////////////////////////

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

    //Set Servo Values
    servoArm.SetMax(SRV_MAX);
    servoArm.SetMin(SRV_MIN);

    servoForkLift.SetMax(SRV_FRK_MAX);
    servoForkLift.SetMin(SRV_FRK_MIN);

    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log\n");

    //Determine course robot located on
    if(USE_RPS)
    {
        RPS.InitializeTouchMenu();
    }
    currentCourse = initMenu();
    runningCourse = RUN_STATE_MACHINE;

    while(runningCourse)
    {
		switch (currentState)
		{
        case waitToStart:
            if(readCdS() == RED)
            {
                SD.Printf("\n==STARTING COURSE==\n\n");
                queState(startMoveSat);
            }
            break;

        case startMoveSat:
            driveToCoord(currentCourse.satelite);
            queState(moveToSat);
            extendRetractArm(true);
            break;
        case moveToSat:

            if((rightEncoder.Counts() / COUNTS_PER_REV * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);
                queState(interactSat);
            }
            break;

        case interactSat:

            queState(startMoveLever);
            extendRetractArm(true);
            break;

        case startMoveLever:
            driveToCoord(currentCourse.lever);
            queState(moveToLever);
            break;

        case moveToLever:
            if(((rightEncoder.Counts() / COUNTS_PER_REV) * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);
                queState(interactLever);
            }
            break;

        case interactLever:

            queState(startMoveSismoBut);
            break;
        case startMoveSismoBut:
            driveToCoord(currentCourse.seismoButton);
            queState(moveToSismoBut);
            break;

        case moveToSismoBut:
            if((rightEncoder.Counts() / COUNTS_PER_REV * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);
                queState(interactSismoBut);
            }
            break;

        case interactSismoBut:
            if(!checkTouchingSide(back))
            {
                drive(-1*MAX);
                while(!checkTouchingSide(back));
                drive(STOP);
            }
            for(int i = 0; i < 10; i++)
            {
                Buzzer.Beep();
                Sleep(0.5);
                if(!checkTouchingSide(back))
                {
                    i = 0;
                    SD.Printf("[interactSismoBut] Haven't touched button for 5 seconds, resestting\n");
                    drive(-1*MAX);
                    while(!checkTouchingSide(back));
                    drive(STOP);
                }
            }
            Buzzer.Buzz(5);
            Buzzer.Buzz(5);
            Buzzer.Buzz(5);
            queState(startMoveCore);
            break;

        case startMoveCore:
            driveToCoord(currentCourse.core);
            queState(moveToCore);
            break;

        case moveToCore:
            if((rightEncoder.Counts() / COUNTS_PER_REV * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);
                queState(interactCore);
            }
            break;

        case interactCore:

            queState(startMoveDepCore);
            break;

        case startMoveDepCore:
            driveToCoord(currentCourse.coreDepo);
            queState(moveToDepCore);
            break;

        case moveToDepCore:
            if((rightEncoder.Counts() / COUNTS_PER_REV * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);
                queState(interactSismoBut);
            }
            break;

        case interactDepCore:
            if(readCdS() == RED)
            {
                SD.Printf("[interactDepCore] Droping core in right bin\n");
                queState(startMoveRet);
            }
            else if (readCdS() == BLUE)
            {
                SD.Printf("[interactDepCore] Droping core in left bin\n");
                queState(startMoveRet);
            } else {
                SD.Printf("[interactDepCore] Can't read light!\n");
                Buzzer.Buzz(5);
            }
            break;

        case startMoveRet:
            driveToCoord(currentCourse.home);
            queState(moveToRet);
            break;

        case moveToRet:
            if((rightEncoder.Counts() / COUNTS_PER_REV * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel)
            {
                drive(STOP);

                rotateTo(SOUTH);
                drive(-1*MAX);
                while(!checkTouchingSide(back));
                drive(STOP);

                queState(shutdown);
            }
            break;

        case shutdown:
            SD.Printf("Course complete!\n");
            runningCourse = false;
            break;
		default:
			break;
		}
    }

    LCD.WriteLine("Shutting down..");
    //Closing SD log file
    SD.Printf("Closing Log\n");
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
    FEHIcon::DrawIconArray(MAIN, 4, 2, 40, 51, 1, 1, main_label, MENU_C, TEXT_C);

    FEHIcon::Icon MAIN_D[1];
    char main_d_label[1][20] = {"DEBUG"};
    FEHIcon::DrawIconArray(MAIN_D, 1, 1, 190, 20, 1, 1, main_d_label, HI_C, TEXT_C);

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
                    SD.Printf("Selecting Course: ");
                    SD.Printf(letter);
                    SD.Printf("\n");                    

                    selectedCourse = Course(letter[0]);

                    noCourseSelected = false;

                    queState(waitToStart);
                    break;
                }
            }
            if (MAIN_D[0].Pressed(x, y, 0))
            {
                LCD.Clear(BLACK);

                FEHIcon::Icon DEBUG_M[1];
                char debug_label[1][20] = {"DEBUG MENU"};
                FEHIcon::DrawIconArray(DEBUG_M, 1, 1, 1, 201, 1, 1, debug_label, HI_C, TEXT_C);
                DEBUG_M[0].Select();

                char debug_states[20][20] = {"WTS", "SMS", "MTS", "IWS",
                                                 "SML", "MTL", "IWL", "SMB",
                                                 "MTB", "IWB", "SMC", "MTC",
                                                 "IWC", "SMD", "MTD", "IWD",
                                                 "SMR", "MTR", "STP", "RPS"};

                FEHIcon::Icon DEBUG[20];
                FEHIcon::DrawIconArray(DEBUG, 5, 4, 40, 1, 1, 1, debug_states, MENU_C, TEXT_C);

                SD.Printf("Debug mode entered\n");

                do{
                    if (LCD.Touch(&x, &y))
                    {
                        //Check to see if a main menu icon has been touched
                        for (n=0; n<20; n++)
                        {
                            if (DEBUG[n].Pressed(x, y, 0))
                            {
                                if(n != 19)
                                {
                                    queState(static_cast<State>(n));
                                    noCourseSelected = false;
                                    LCD.Clear(BLACK);
                                    selectedCourse = Course('A');
                                } else {
                                    SD.Printf("Reading coords");
                                    LCD.Clear(BLACK);
                                    LCD.WriteRC("X:",0,0);
                                    LCD.WriteRC("Y:",1,0);
                                    LCD.WriteRC("Head:", 2, 0);
                                    LCD.WriteRC("Read time:", 4, 0);
                                    Sleep(0.5);
                                    double time, time2;
                                    while(!LCD.Touch(&x, &y))
                                    {
                                        time = TimeNow();
                                        LCD.WriteRC(RPS.X(), 0, 4);
                                        LCD.WriteRC(RPS.Y(), 1, 4);
                                        LCD.WriteRC(RPS.Heading(), 2, 7);
                                        time2 = TimeNow();
                                        LCD.WriteRC(time2-time, 4, 12);
                                        //SD.Printf(time2-time);
                                        //SD.Printf(" s\n");
                                        LCD.WriteRC(" s ", 4, 23);
                                    }
                                }
                                break;
                            }
                        }
                    }
                }while(noCourseSelected);
                SD.Printf("Debug mode exited\n");

            }
        }
    } while(noCourseSelected);

    return selectedCourse;
}

/**
 * @brief queState
 *      Sets the state to nextState and say the state change in the log
 * @param nextState
 *      State being changed to
 */
void queState(State nextState)
{
    currentState = nextState;

    numberOfStateChanges++;

    SD.Printf("Queing state ");
    SD.Printf(STATE_NAMES[nextState]);
    SD.Printf("\n");

    if(QUIT_AFTER_ONE_STATE && numberOfStateChanges > 2)
    {
        currentState = shutdown;
        SD.Printf("Debug completed, state change stopped\n");
    }
}

/**
 * @brief extendRetractArm
 *      Sets the satelite arm to max or min
 * @param isExtending
 *      If it is true sets it to max, otherwise set to min
 */
void extendRetractArm(bool isExtending)
{
    if(isExtending)
    {
        servoArm.SetDegree(SERVO_FULL_EXT);
    } else {
        servoArm.SetDegree(SERVO_NO_EXT);
    }
}

/**
 * @brief setForkLiftPos
 *      Sets the forklift height based off of a percent of its max height
 * @param percent
 *      What percent heightwise the forklift is at
 */
void setForkLiftPos(float percent)
{
    servoForkLift.SetDegree((percent/100)*(SRV_FRK_MAX-SRV_FRK_MIN) + SRV_FRK_MIN);
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
    if(rMPercent == 0)
    {
        rightMotor.Stop();
    } else {
        rightMotor.SetPercent(rMPercent);
    }

    if(lMPercent == 0)
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
 * @brief checkTouchingSide
 * @param side
 * @return
 */
bool checkTouchingSide(ButtonSide side)
{
    bool buttonsPressed = false;

    switch (side) {
    case front:
        buttonsPressed = !(buttonTopLeft.Value() || buttonTopRight.Value());
        break;
    case back:
        buttonsPressed = !(buttonBottomLeft.Value() || buttonBottomRight.Value());
        break;
    default:
        SD.Printf("[driveTilHitSide] ERROR: Call to non defined ButtonSide Case\n");
        break;
    }

    return buttonsPressed;
}

/**
 * @brief turn
 *      Turn right or left
 * @param goRight
 *      True to go right, false left
 */
void turn(bool goRight)
{
    int rDirection = (goRight) ? 1:-1;
    int lDirection = (goRight) ? -1:1;

    drive(rDirection * MAX, lDirection * MAX);
}

/**
 * @brief rotateTo
 *      Rotate till at given direction
 * @param directionToHead
 *      The degree of direction to be facing
 */
void rotateTo(float directionToHead)
{
    float currHeading = RPS.Heading(),
          leftDistance = currHeading - directionToHead,
          rightDistance = directionToHead - currHeading;

    while(RPS.Heading() < 0)
    {
        Buzzer.Buzz(5);
    }

    currHeading = RPS.Heading();

    SD.Printf("At heading: %f moving to heading: %f\n", currHeading, directionToHead);


    leftDistance = (leftDistance > 0.0 ? leftDistance: (360.0+leftDistance));
    rightDistance = (rightDistance > 0.0 ? rightDistance: (360.0+rightDistance));

    if(rightDistance < leftDistance)
    {
        turn(RIGHT);
        while(abs(RPS.Heading() - directionToHead) > HEAD_ERR){}
        drive(STOP);
    } else {
        turn(LEFT);
        while(abs(RPS.Heading() - directionToHead) > HEAD_ERR){}
        drive(STOP);
    }
    SD.Printf("Angle reached! Heading out at: %f\n", RPS.Heading());
}

/**
 * @brief driveToCoord
 *      The robot takes the shortest path to the given coords
 * @param pos
 *      The coordinate to head to
 */
void driveToCoord(Coord pos)
{    
    float currX = RPS.X(),
          currY = RPS.Y(),
          directionToHead;

    while(currX < 0)
    {
        Buzzer.Buzz(5);
    }
    currX = RPS.X();
    currY = RPS.Y();

    distanceToTravel = sqrtf(pow(pos.x-currX, 2) + pow(pos.y-currY, 2));
    directionToHead = atan2f(pos.y-currY,pos.x-currX) * (180.0/M_PI);
    //Switches directions to be from 0 to 360

    SD.Printf("At: (%f, %f) moving to (%f, %f) at calculated angle %f\n", currX, currY, pos.x, pos.y, directionToHead);

    directionToHead = (directionToHead > 0.0 ? directionToHead: (360.0+directionToHead));

    directionToHead += DEGREE_OFFSET;
    directionToHead = (directionToHead > 360.0 ? (directionToHead-360): (directionToHead));


    rotateTo(directionToHead);



    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    destination = pos;

    drive(MAX);
}

/////////////////////////////////////////////END REGION//////////////////////////////////////////


/////////////////////////////////////REGION: Class Definitions//////////////////////////////////////

/**
 * @brief Course::Course
 *      Constructor for the Course Class
 * @param courseLet
 *      A char that is used to define which
 *      course the robot is starting on
 */
Course::Course(char c)
{
    courseLetter = c;
    satelite = Coord(SAT_X, SAT_Y);
    lever = Coord(LEV_X, LEV_Y);
    seismoButton = Coord(BUT_X, BUT_Y);
    core = Coord(COR_X, COR_Y);
    coreDepo = Coord(DEP_X, DEP_Y);
    home = Coord(RET_X, RET_Y);
}

/**
 * @brief Course::Course
 *      Empty constructor. Not to be used except for initial decleration of variable
 */
Course::Course(){}

/**
 * @brief Coord::Coord
 *      Constructor for the coord class
 * @param X
 *      X part of the coord
 * @param Y
 *      Y part of the coord
 */
Coord::Coord(float X, float Y)
{
    x = X;
    y = Y;
}

/**
 * @brief Coord::Coord
 *      Empty constructor. Not to be used except for initial decleration of variable
 */
Coord::Coord(){}

/////////////////////////////////////////////END REGION/////////////////////////////////////////////


