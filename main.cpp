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
#include <string.h>



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
    Coord satelite, lever, seismoButton, core, coreDepo, home, bottomOfRamp, topOfRamp;

private:
    char courseLetter;
};


/* Function declerations */

void initialSetup();
Course initMenu();
void queState(State);
void extendRetractArm(bool);
void setForkLiftPos(float);
unsigned int readCdS();
bool verifyStartConditions();
void drawRunningScreen();
bool drivedDistance();

//Drive functions
void drive(float, float);
void drive(float);
void driveDistance(float, float);
bool checkTouchingSide(ButtonSide);
void turn(bool, float);
void turnBlind(bool, float, float);
void rotateTo(float);
void driveToCoord(Coord);
void PIDCheck();
void followLine(float);
int leftCountOffset();
int rightCountOffset();


/* Global Variable Decleration */

//Motors
FEHMotor leftMotor(DT_MOTOR_L, DT_MOTOR_LV);
FEHMotor rightMotor(DT_MOTOR_R, DT_MOTOR_RV);

FEHMotor motorForkLift(MTR_FRK_LFT, MTR_FRK_V);

//Motor encoders
DigitalEncoder rightEncoder(MTR_ENCODE_R);
DigitalEncoder leftEncoder(MTR_ENCODE_L);

//Servos
FEHServo servoArm(SRV_ARM);

//Corner bump buttons
DigitalInputPin buttonTopLeft(BUTTON_TOP_LEFT);
DigitalInputPin buttonTopRight(BUTTON_TOP_RIGHT);
DigitalInputPin buttonBottomLeft(BUTTON_BOTTOM_LEFT);
DigitalInputPin buttonBottomRight(BUTTON_BOTTOM_RIGHT);

DigitalInputPin buttonForkLiftBottom(BUTTON_FRK_BOT);
DigitalInputPin buttonForkLiftTop(BUTTON_FRK_TOP);

//CdS Cell
AnalogInputPin CdS(CDS_CELL);

//Optosensors
AnalogInputPin rightOpt(OPTO_RIGHT);
AnalogInputPin middleOpt(OPTO_CENTER);
AnalogInputPin leftOpt(OPTO_LEFT);

//Text representation of all of the states
const char STATE_NAMES[19][20] = {"waitToStart", "startMoveSat", "moveToSat", "interactSat",
                                 "startMoveLever", "moveToLever", "interactLever", "startMoveSismoBut",
                                 "moveToSismoBut", "interactSismoBut", "startMoveCore", "moveToCore",
                                 "interactCore", "startMoveDepCore", "moveToDepCore", "interactDepCore",
                                 "startMoveRet", "moveToRet", "shutdown"};

//If the course is being run by the robot. i.e. if it is in the state machine
bool runningCourse;

//The current state of the state machine the robot is in
State currentState;

//How far the robot is to travel before next state
float distanceToTravel;

//The destination the robot is heading towards
Coord destination;

//Whether or not the robot is driving straight and should be using PID correction
bool enablePID;

//The speeds that each motor are set at
float rightMotorSpeed,
      leftMotorSpeed;

//The percentage the servos are set at
float servoFrkPer,
      servoArmPer;

//How many state changes the robot has gone through
int numberOfStateChanges;

//Message displayed on the Running Screen
char screenMessage[20];

//The course the robot is running on
Course currentCourse;

//Color fo the bin
unsigned int binColor;

//////////////////////////////////////////END REGION//////////////////////////////////////////


/**
 * @brief main
 *      main program method
 * @return 0
 */
int main(void)
{
    //drive(MAX);

    initialSetup();

    unsigned int waitTime = TimeNowSec();
    unsigned int time;

    while(runningCourse)
    {
        //If trying to drive straight verify actually driving straight
        if(enablePID)
        {
            PIDCheck();
        }

        //Draw the running screen for hardware verification
        drawRunningScreen();

        //Loop through state machine
		switch (currentState)
		{
        case waitToStart:
            //This is being changed for performance tests

            if(readCdS() != BLACK || (TimeNowSec() - waitTime > 20))
            {
                SD.Printf("\n==STARTING COURSE==\n");
                LCD.Clear(BLACK);

                queState(startMoveSat);
            }

            break;

        case startMoveSat:

            SD.Printf("Driving Forward\n");
            driveDistance(MAX, 5);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            Sleep(0.2);
            drive(STOP);
            SD.Printf("Turning\n");
            turnBlind(RIGHT, 85, TURN_MAIN);
            time = TimeNowSec();
            while(!checkTouchingSide(BACK) && (TimeNowSec() - time < 3))
            {
                drive(-0.5*MAX, -.6*MAX);
            }
            if(!checkTouchingSide(BACK))
            {
                if(!buttonBottomLeft.Value())
                {
                    drive(0, MAX);
                    Sleep(0.05);
                    drive(-MAX);
                    Sleep(0.25);
                }
                if(!buttonBottomRight.Value())
                {
                    drive(MAX, 0);
                    Sleep(0.05);
                    drive(-MAX);
                    Sleep(0.25);
                }
            }
            drive(STOP);
            Sleep(0.1);
            SD.Printf("Back found, x coordinate confirmed\n");
            SD.Printf("Driving till detect light\n");
            drive(MAX);
            while(readCdS() == BLACK)
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            //Sleep(0.2);
            drive(STOP);
            binColor = readCdS();
            if(binColor == BLACK)
            {
                binColor = BLUE;
            }
            SD.Printf("Light Found!  Color: %s\n", (binColor == RED) ? "Red":"Blue");
            queState(moveToSat);
            break;

        case moveToSat:
            SD.Printf("Driving till hit front\n");
            //drive(MAX, 0);
            drive(MAX);
            time = TimeNowSec();
            while(!checkTouchingSide(FRONT) && (TimeNowSec() - time < 10))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            SD.Printf("Front found, turning satelite\n");
            queState(interactSat);
            break;

        case interactSat:
            extendRetractArm(true);
            driveDistance(-MAX, 7.0);
            time = TimeNowSec();
            while(!drivedDistance() && (TimeNowSec() - time < 3))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            extendRetractArm(false);


            driveDistance(MAX, 3.0);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            //Sleep(0.1);
            drive(STOP);

            setForkLiftPos(FRK_NO_EXT);

            rotateTo(WEST);

            queState(startMoveSismoBut);

            break;

        case startMoveSismoBut:
            SD.Printf("Driving till hit back\n");
            drive(-MAX);
            time = TimeNowSec();
            while(!checkTouchingSide(BACK) && (TimeNowSec() - time < 10))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);

            driveDistance(MAX, 5);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            Sleep(0.4);
            drive(STOP);

            turnBlind(RIGHT, 85, TURN_MAIN);
            //rotateTo(SOUTH);

            SD.Printf("At base of ramp\n");
            strcpy(screenMessage, "At base of ramp");
            drawRunningScreen();

            motorForkLift.SetPercent(FRK_UP);
            Sleep(2.0);
            motorForkLift.SetPercent(STOP);

            driveDistance(-MAX, 18);
            time = TimeNowSec();
            while(!drivedDistance() && (TimeNowSec() - time < 5))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            queState(moveToSismoBut);
            break;

        case moveToSismoBut:
            //setForkLiftPos(FRK_NO_EXT);
            //rotateTo(WEST);
            turnBlind(LEFT, 83, TURN_MAIN);
            drive(-MAX);
            while(!checkTouchingSide(BACK))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            drive(MAX);
            Sleep(0.5);
            drive(STOP);
            turnBlind(RIGHT, 85, TURN_MAIN);
            drive(-MAX);

            SD.Printf("Moving to hit button\n");
            while(buttonBottomLeft.Value() && buttonBottomRight.Value())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            queState(interactSismoBut);
            break;

        case interactSismoBut:

            SD.Printf("Button found, waiting 5 seconds\n");
            Sleep(5.5);
            queState(startMoveLever);
            break;

        case startMoveLever:

            driveDistance(MAX, 10);
            motorForkLift.SetPercent(FRK_UP);
            Sleep(0.5);
            motorForkLift.SetPercent(STOP);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            Sleep(0.10);
            drive(STOP);
            queState(moveToLever);
            break;

        case moveToLever:

            SD.Printf("Turning\n");
            //rotateTo(WEST);
            turnBlind(LEFT, 87, TURN_MAIN);
            setForkLiftPos(FRK_FULL_EXT);
            driveDistance(MAX, 11);
            setForkLiftPos(FRK_FULL_EXT);
            while(!drivedDistance() && !checkTouchingSide(FRONT))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }            
            Sleep(0.3);
            SD.Printf("Lever found!\n");
            drive(STOP);
            queState(interactLever);

            break;

        case interactLever:
            /*
            drive(-MAX);
            Sleep(0.1);
            drive(STOP);
            */
            motorForkLift.SetPercent(.5*FRK_DOWN);
            Sleep(.75);
            motorForkLift.SetPercent(.5*FRK_UP);
            Sleep(0.25);
            motorForkLift.SetPercent(STOP);
            queState(startMoveCore);
            break;

        case startMoveCore:
            SD.Printf("Rotating to core\n");
            strcpy(screenMessage, "Rotating to core");
            drawRunningScreen();

            drive(-MAX);
            Sleep(0.5);
            drive(STOP);

            setForkLiftPos(FRK_NO_EXT);

            SD.Printf("Verifying X\n");
            rotateTo(WEST);
            rotateTo(WEST);
            driveDistance(-MAX, 2.5);
            while(abs(RPS.X() - currentCourse.core.x) > 0.5)
            {
                if(RPS.X() > currentCourse.core.x)
                {
                    SD.Printf("Decreasing X\n");
                    drive(TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                } else {
                    SD.Printf("Increasing X\n");
                    drive(-TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                }
            }

            SD.Printf("Verifying Y\n");
            rotateTo(NORTH);
            driveDistance(MAX, 13);
            while(abs(RPS.Y() - currentCourse.core.y) > 0.5)
            {
                if(RPS.Y() > currentCourse.core.y)
                {
                    SD.Printf("Decreasing Y\n");
                    drive(-TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                } else {
                    SD.Printf("Increasing Y\n");
                    drive(TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                }
            }

            queState(moveToCore);
            break;

        case moveToCore:
            setForkLiftPos(FRK_NO_EXT);
            SD.Printf("Current Pos: (%f,%f)", RPS.X(), RPS.Y());
            rotateTo((NORTH+WEST)/2 + 2);

            SD.Printf("Line following\n");
            strcpy(screenMessage, "Line following");
            drawRunningScreen();
            drive(MAX);
            Sleep(1.75);
            drive(STOP);
            queState(interactCore);
            break;

        case interactCore:

            SD.Printf("Grabing core\n");
            strcpy(screenMessage, "Grabing core");
            drawRunningScreen();

            drive(-.5*MAX);
            Sleep(0.01);
            drive(STOP);

            motorForkLift.SetPercent(.5*FRK_UP);
            Sleep(1.0);
            motorForkLift.SetPercent(STOP);
            drive(-MAX);
            Sleep(1.25);
            drive(STOP);
            setForkLiftPos(FRK_NO_EXT);
            queState(startMoveDepCore);
            break;

        case startMoveDepCore:

            //Move to top of ramp and move down it

            SD.Printf("Heading down ramp\n");
            strcpy(screenMessage, "Heading down ramp");
            drawRunningScreen();

            rotateTo(NORTH);
            driveDistance(-MAX, 4);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            rotateTo(NORTH);
            motorForkLift.SetPercent(FRK_UP);
            Sleep(2.0);
            motorForkLift.SetPercent(STOP);
            //driveToCoord(currentCourse.bottomOfRamp);
            driveDistance(-MAX, 16);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            Sleep(0.2);
            drive(STOP);
            setForkLiftPos(FRK_NO_EXT);
            queState(startMoveRet);
            break;

        case moveToDepCore:
            SD.Printf("Moving to deposit\n");
            strcpy(screenMessage, "Moving to deposit");
            drawRunningScreen();
            turnBlind(RIGHT, 85, TURN_MAIN);

            drive(-MAX);
            time = TimeNowSec();

            while(!checkTouchingSide(BACK) && (TimeNowSec() - time < 10))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);

            float xVal;
            if(binColor == RED)
            {
                driveDistance(MAX, 11);
                xVal = 11.5;
            }
            else
            {
                driveDistance(MAX, 7);
                xVal = 16.9;
            }
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            while(abs(RPS.X() - xVal) > .5)
            {
                if(RPS.X() > xVal)
                {
                    drive(TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                } else {
                    drive(-TURN_MAIN);
                    Sleep(0.1);
                    drive(STOP);
                }
            }
            queState(interactDepCore);
            break;

        case interactDepCore:

            SD.Printf("Dropping core\n");
            drawRunningScreen();

            setForkLiftPos(FRK_NO_EXT);
            rotateTo(SOUTH);
            setForkLiftPos(FRK_FULL_EXT);
            drive(MAX);

            time = TimeNowSec();


            while(buttonTopLeft.Value() && buttonTopRight.Value() && (TimeNowSec() - time < 5))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            drive(-MAX);
            Sleep(0.15);
            drive(STOP);
            setForkLiftPos(FRK_NO_EXT);
            drive(-MAX);
            Sleep(0.15);
            drive(STOP);
            motorForkLift.SetPercent(FRK_UP);
            Sleep(2.0);
            motorForkLift.SetPercent(STOP);
            drive(MAX);
            Sleep(0.15);
            drive(STOP);
            driveDistance(-MAX, 5);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            queState(startMoveRet);
            break;

        case startMoveRet:
            setForkLiftPos(FRK_NO_EXT);
            turnBlind(LEFT, 85, TURN_MAIN);
            drive(-MAX);

            time = TimeNowSec();
            while(!checkTouchingSide(BACK) && (TimeNowSec() - time < 10))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            drive(MAX);
            Sleep(0.15);
            rotateTo(SOUTH+2);

            queState(moveToRet);
            break;

        case moveToRet:
            drive(-MAX);

            queState(shutdown);
            break;

        case shutdown:
            SD.Printf("Course complete!\n");
            runningCourse = false;
            break;

		default:
            SD.Printf("DEFAULT ENCOUNTERED IN STATE!\n");
			break;
		}
    }

    LCD.Clear(BLACK);
    LCD.WriteLine("Shutting down..");
    //Closing SD log file
    SD.Printf("Closing Log\n");
    SD.CloseLog();

    //Ending Program
    return 0;
}

/**
 * @brief initialSetup
 *      Sets initial values and settings
 */
void initialSetup()
{
    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log\n");

    //CLears the LCD of anything previously on it
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    //Set Servo Values
    SD.Printf("Initializing Servo\n");
    servoArm.SetMax(SRV_MAX);
    servoArm.SetMin(SRV_MIN);
    servoArm.SetDegree(SERVO_FULL_EXT);
    servoArm.SetDegree(SERVO_NO_EXT);

    //Set fork to highest height
    SD.Printf("Setting Fork lift to max height\n");
    setForkLiftPos(FRK_FULL_EXT);

    //Sets initial global variable values
    SD.Printf("Setting initial global variable values\n");
    rightMotorSpeed = leftMotorSpeed = 0;
    numberOfStateChanges = 0;
    enablePID = false;
    strcpy(screenMessage,"INIT");

    //Determine course robot located on
    if(USE_RPS)
    {
        SD.Printf("Initializing RPS\n");
        RPS.InitializeTouchMenu();
    } else {
        SD.Printf("Starting without RPS\n");
    }

    SD.Printf("Starting Initial Check\n");
    while(INIT_CHECK && !verifyStartConditions())
    {
        Buzzer.Buzz(5);
    }

    //Gets if running course and what course it is on    
    SD.Printf("Running initMenu - ");
    runningCourse = RUN_STATE_MACHINE;
    currentCourse = initMenu();
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
    float m = 0, batteryVoltage = 0;
    Course selectedCourse;

    LCD.Clear(BLACK);

    //Create icons for main menu
    FEHIcon::Icon iconMainT[1];
    char mainTLabel[1][20] = {"LIBERTATUM VOLCANUS"};
    FEHIcon::DrawIconArray(iconMainT, 1, 1, 1, 201, 1, 1, mainTLabel, HI_C, TEXT_C);
    iconMainT[0].Select();

    FEHIcon::Icon iconMain[6];
    char mainLabel[8][20] = {"A", "B", "C", "D", "E", "F", "G", "H"};
    FEHIcon::DrawIconArray(iconMain, 4, 2, 40, 51, 1, 1, mainLabel, MENU_C, TEXT_C);

    FEHIcon::Icon iconMainD[1];
    char mainDLabel[1][20] = {"DEBUG"};
    FEHIcon::DrawIconArray(iconMainD, 1, 1, 190, 20, 1, 1, mainDLabel, HI_C, TEXT_C);

    LCD.SetFontColor(TEXT_C);
    LCD.WriteAt("BATT:        V", 0, 222);

    do
    {
        //Display average battery voltage to screen
        batteryVoltage = ((batteryVoltage*m)+Battery.Voltage());
        batteryVoltage = batteryVoltage/(++m);
        LCD.WriteAt(batteryVoltage, 72, 222);
        if(batteryVoltage < 10)
        {
            Buzzer.Buzz(10);
        }
        if (LCD.Touch(&x, &y))
        {
            //Check to see if a main menu icon has been touched
            for (n=0; n<=7; n++)
            {
                if (iconMain[n].Pressed(x, y, 0))
                {
                    char letter[2] = {mainLabel[n][0], ' '};
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
            if (iconMainD[0].Pressed(x, y, 0))
            {
                LCD.Clear(BLACK);

                FEHIcon::Icon iconDebugM[1];
                char debugLabel[1][20] = {"DEBUG MENU"};
                FEHIcon::DrawIconArray(iconDebugM, 1, 1, 1, 201, 1, 1, debugLabel, HI_C, TEXT_C);
                iconDebugM[0].Select();

                char debugStates[20][20] = {"WTS", "SMS", "MTS", "IWS",
                                                 "SML", "MTL", "IWL", "SMB",
                                                 "MTB", "IWB", "SMC", "MTC",
                                                 "IWC", "SMD", "MTD", "IWD",
                                                 "SMR", "MTR", "STP", "RPS"};

                FEHIcon::Icon iconDebug[20];
                FEHIcon::DrawIconArray(iconDebug, 5, 4, 40, 1, 1, 1, debugStates, MENU_C, TEXT_C);

                SD.Printf("Debug mode entered\n");

                do{
                    if (LCD.Touch(&x, &y))
                    {
                        //Check to see if a main menu icon has been touched
                        for (n=0; n<20; n++)
                        {
                            if (iconDebug[n].Pressed(x, y, 0))
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

                                    while(!LCD.Touch(&x, &y))
                                    {
                                        drawRunningScreen();
                                        readCdS();
                                    }
                                    noCourseSelected = false;
                                    runningCourse = false;
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

    SD.Printf("\n=== Queing state %s ===\n", STATE_NAMES[nextState]);

    strcpy(screenMessage, STATE_NAMES[nextState]);

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
        servoArmPer = SERVO_FULL_EXT;
    } else {
        servoArm.SetDegree(SERVO_NO_EXT);
        servoArmPer = SERVO_NO_EXT;
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
    SD.Printf("Setting fork lift to %f percent\n", percent);
    if(percent > 95.0)
    {        
        if(buttonForkLiftBottom.Value()){
            motorForkLift.SetPercent(FRK_UP);
        }
        unsigned int time = TimeNowSec();

        while(buttonForkLiftBottom.Value() && (TimeNowSec() - time < 10));
    }
    else if (percent < 5.0)
    {
        if(buttonForkLiftTop.Value()){
            motorForkLift.SetPercent(FRK_UP);
        }
        motorForkLift.SetPercent(FRK_DOWN);
        unsigned int time = TimeNowSec();
        while(buttonForkLiftTop.Value() && (TimeNowSec() - time < 10));
    }
    SD.Printf("Percent reached!\n");
    motorForkLift.SetPercent(STOP);
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

    if(CdS.Value() > BLACK_LIGHT)
    {
        retColor = BLACK;
    } else if(CdS.Value() > BLUE_LIGHT) {
        retColor = BLUE;
    } else {
        retColor = RED;
    }
    LCD.Clear(retColor);
    return retColor;
}

/**
 * @brief verifyStartConditions
 *      checks if the start conditions for the robot are what are expected
 * @return
 *      if start conditions are good
 */
bool verifyStartConditions()
{
    bool flag = true;
    if(!buttonBottomLeft.Value())
    {
        LCD.WriteRC("BOTTOM LEFT ACTIVATED", 0, 0);
        flag = false;
    }
    else
    {
        LCD.WriteRC("                      ", 0, 0);
    }
    if(!buttonBottomRight.Value())
    {
        LCD.WriteRC("BOTTOM RIGHT ACTIVATED", 1, 0);
        flag = false;
    }
    else
    {
        LCD.WriteRC("                      ", 1, 0);
    }
    if(!buttonTopLeft.Value())
    {
        LCD.WriteRC("BOTTOM LEFT ACTIVATED", 2, 0);
        flag = false;
    }
    else
    {
        LCD.WriteRC("                      ", 2, 0);
    }
    if(!buttonTopRight.Value())
    {
        LCD.WriteRC("BOTTOM LEFT ACTIVATED", 3, 0);
        flag = false;
    }
    else
    {
        LCD.WriteRC("                      ", 3, 0);
    }

    if(readCdS() != BLACK)
    {
        LCD.WriteRC("CDS NOT READING BLACK", 4, 0);
        flag = false;
    }


    return flag;
}

/**
 * @brief drawRunningScreen
 *      draws information on the screen to visually see if hardware
 *      and software are interacting correctly
 */
void drawRunningScreen()
{
    if(!buttonBottomLeft.Value())
    {
        LCD.SetFontColor(LIGHTGREEN);
    } else {
        LCD.SetFontColor(DARKGRAY);
    }
    LCD.FillCircle(15, 220, 10);
    if(!buttonBottomRight.Value())
    {
        LCD.SetFontColor(LIGHTGREEN);
    } else {
        LCD.SetFontColor(DARKGRAY);
    }
    LCD.FillCircle(305, 220, 10);
    if(!buttonTopLeft.Value())
    {
        LCD.SetFontColor(LIGHTGREEN);
    } else {
        LCD.SetFontColor(DARKGRAY);
    }
    LCD.FillCircle(15, 10, 10);
    if(!buttonTopRight.Value())
    {
        LCD.SetFontColor(LIGHTGREEN);
    } else {
        LCD.SetFontColor(DARKGRAY);
    }
    LCD.FillCircle(305, 10, 10);

    LCD.SetFontColor(WHITE);

    LCD.WriteRC(" RPS (     ,     ) @     ", 3, 0);
    LCD.WriteRC(" DRS ( NOT , IN  ) @ USE ", 4, 0);

    LCD.WriteRC((int)RPS.X(), 3, 7);
    LCD.WriteRC((int)RPS.Y(), 3, 13);
    LCD.WriteRC(((int)RPS.Heading()), 3, 21);

    LCD.WriteRC(" RM:           RE: ", 5, 0);
    LCD.WriteRC(rightMotorSpeed, 5, 5);
    LCD.WriteRC(rightEncoder.Counts(), 5, 18);

    LCD.WriteRC(" LM:           LE: ", 6, 0);
    LCD.WriteRC(leftMotorSpeed, 6, 5);
    LCD.WriteRC(leftEncoder.Counts(), 6, 18);

    LCD.WriteRC(" FRK:              ", 7, 0);
    LCD.WriteRC(servoFrkPer, 7, 6);

    LCD.WriteRC(" ARM:              ", 8, 0);
    LCD.WriteRC(servoArmPer, 8, 6);

    LCD.WriteRC(" ST:               ", 9, 0);
    LCD.WriteRC(screenMessage, 9, 5);

    LCD.WriteRC(" CDS:               ", 10, 0);
    LCD.WriteRC(CdS.Value(), 10, 6);

    LCD.WriteRC("OPT(      ,      ,      )", 11, 0);

    LCD.WriteRC(leftOpt.Value(), 11, 4);
    LCD.WriteRC(middleOpt.Value(), 11, 11);
    LCD.WriteRC(rightOpt.Value(), 11, 18);
    LCD.WriteRC(",", 11, 10);
    LCD.WriteRC(",", 11, 17);

}

/**
 * @brief drivedDistance
 *      Checks if the robot has traveled the qued distance
 * @returns
 *      If the robot has traveled the specified distance
 */
bool drivedDistance()
{
    return ((((rightEncoder.Counts()+rightCountOffset() + leftEncoder.Counts()+leftCountOffset()) /2)
             / COUNTS_PER_REV)
                * (WHEEL_RAD * 2 * M_PI)) > (.95 * distanceToTravel);
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
    lMPercent *= 1.013;
    float rMPerToChange = (rMPercent - rightMotorSpeed)/10.0;
    float lMPerToChange = (lMPercent - leftMotorSpeed)/10.0;

    for(int i = 0; i < 10; i++)
    {
        rightMotorSpeed += rMPerToChange;
        leftMotorSpeed += lMPerToChange;

        rightMotor.SetPercent(RIGHT_MOTOR_MODIFIER*rightMotorSpeed);
        leftMotor.SetPercent(LEFT_MOTOR_MODIFIER*leftMotorSpeed);

        Sleep(MOTOR_SPEED_RAMP_TIME);

    }

    rightMotorSpeed = rMPercent;
    leftMotorSpeed = lMPercent;
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
    enablePID = true;
}

/**
 * @brief driveDistance
 *      Sets the speed of the motor and sets how long the robot should travel for
 * @param mPercent
 *      Percentfor both motors
 * @param distance
 *      How far it should travel
 */
void driveDistance(float mPercent, float distance)
{
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    distanceToTravel = 1.5*distance;
    drive(mPercent);
    enablePID = true;
    SD.Printf("Driving at speed %f for distance %f\n", mPercent, distance);
}

/**
 * @brief checkTouchingSide
 *      Checks to see if the bump switches on a particular side are both being pressed.
 *      If only one of them is being pressed it disables that motor.
 * @param side
 *      What side to check for, front or back
 * @returns
 *      Whether or not both buttons are being pressed
 */
bool checkTouchingSide(ButtonSide side)
{
    bool buttonsPressed = false;

    switch (side) {
    case FRONT:
        buttonsPressed = !(buttonTopLeft.Value() || buttonTopRight.Value());
        if(!buttonTopLeft.Value() && leftMotorSpeed != 0)
        {
            SD.Printf("Top Left Bump Switch touching when checking front, Disabling left motor\n");
            leftMotor.Stop();
        }
        if(!buttonTopRight.Value()&& rightMotorSpeed != 0)
        {
            SD.Printf("Top Right Bump Switch touching when checking front, Disabling right motor\n");
            rightMotor.Stop();
        }
        break;
    case BACK:
        buttonsPressed = !(buttonBottomLeft.Value() || buttonBottomRight.Value());
        if(!buttonBottomLeft.Value()&& leftMotorSpeed != 0)
        {
            SD.Printf("Bottom Left Bump Switch touching when checking front, Disabling left motor\n");
            leftMotor.Stop();
        }
        if(!buttonBottomRight.Value() && rightMotorSpeed != 0)
        {
            SD.Printf("Bottom Right Bump Switch touching when checking front, Disabling right motor\n");
            rightMotor.Stop();
        }
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
void turn(bool goRight, float speed)
{
    int rDirection = (goRight) ? 1:-1;
    int lDirection = (goRight) ? -1:1;

    drive(rDirection * speed, lDirection * speed);
    enablePID = false;
    SD.Printf("Turning %s at speed %f\n", (goRight) ? "right":"left", speed);
}

/**
 * @brief turnBlind
 *      Turns the robot a desired degree, at a desired speed, without using RPS
 * @param goRight
 *      Whether to turn right or left
 * @param degree
 *      The degree amount to turn
 * @param speed
 *      The speed at which to turn
 */
void turnBlind(bool goRight, float degree, float speed)
{
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    float wheelSeperationRad = 4.5;

    SD.Printf("Turning blind %f degrees  ", degree);

    turn(goRight, speed);
    while(((leftEncoder.Counts() + rightEncoder.Counts() + leftCountOffset())/2 * ((WHEEL_RAD * 2 * M_PI) /COUNTS_PER_REV))
          < (degree * M_PI / 180 * wheelSeperationRad));
    drive(STOP);
}

/**
 * @brief rotateTo
 *      Rotate till at given direction
 * @param directionToHead
 *      The degree of direction to be facing
 * @requires USE_RPS == true
 */
void rotateTo(float directionToHead)
{
    float currHeading = RPS.Heading(),
          leftDistance = currHeading - directionToHead,
          rightDistance = directionToHead - currHeading;
    bool turnDirection;

    while(currHeading < 0)
    {
        Buzzer.Buzz(5);
        currHeading = RPS.Heading();
        SD.Printf("Can't get heding, turning to get heading\n");
        turnBlind(LEFT, 2.0, TURN_FINE);
    }

    SD.Printf("At heading: %f moving to heading: %f\n", currHeading, directionToHead);

    leftDistance = (leftDistance > 0.0 ? leftDistance: (360.0+leftDistance));
    rightDistance = (rightDistance > 0.0 ? rightDistance: (360.0+rightDistance));

    if(rightDistance < leftDistance)
    {
        turnDirection = RIGHT;

    } else {
        turnDirection = LEFT;
    }

    turnBlind(turnDirection, (turnDirection == RIGHT) ? rightDistance : leftDistance, TURN_MAIN);
    SD.Printf("Checking heading with rps\n");
    while(abs(RPS.Heading() - directionToHead) > 1.5)
    {
        if(RPS.Heading() >= 0)
        {
            SD.Printf("RPS Read Success   ");
            currHeading = RPS.Heading();
        } else {
            SD.Printf("RPS Read Failure   ");
        }
        drawRunningScreen();
        SD.Printf("Error: %f Heading: %f  ", abs(currHeading - directionToHead), currHeading);
        leftDistance = currHeading - directionToHead,
        rightDistance = directionToHead - currHeading;

        leftDistance = (leftDistance > 0.0 ? leftDistance: (360.0+leftDistance));
        rightDistance = (rightDistance > 0.0 ? rightDistance: (360.0+rightDistance));

        if(rightDistance < leftDistance)
        {
            turnDirection = RIGHT;
            currHeading++;
        } else {
            turnDirection = LEFT;
            currHeading--;
        }

        //turnBlind(turnDirection, 2.0, TURN_FINE);
        float amount = (abs(RPS.Heading() - directionToHead))/2.0;
        turnBlind(turnDirection, (amount < 2.0) ? 2.0 : amount, TURN_FINE);
    }


    SD.Printf("Angle reached! Heading out at: %f\n", RPS.Heading());
}

/**
 * @brief driveToCoord
 *      The robot takes the shortest path to the given coords
 * @param pos
 *      The coordinate to head to
 * @requires USE_RPS == true
 */
void driveToCoord(Coord pos)
{    
    float currX = RPS.X(),
          currY = RPS.Y(),
          directionToHead;

    while(currX < 0)
    {
        Buzzer.Buzz(5);
        currX = RPS.X();
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

    //Verify that during turn RPS Coords didn't change
    currX = RPS.X();
    currY = RPS.Y();

    distanceToTravel = sqrtf(pow(pos.x-currX, 2) + pow(pos.y-currY, 2));
    SD.Printf("Traveling for %f inches\n", distanceToTravel);

    //Reset encoders for travel
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    destination = pos;

    drive(MAX);
}

/**
 * @brief PIDCheck
 *      Makes sure that when the robot is intending to drive straight that it is
 */
void PIDCheck()
{
    if(PID_ALLOW){
        if(leftEncoder.Counts() < rightEncoder.Counts())
        {
            if(abs(leftMotorSpeed-rightMotorSpeed) <= PID_MAX_DIFF)
            {
                if(rightMotorSpeed > 0)
                {
                    rightMotorSpeed -= 1;
                } else {
                    rightMotorSpeed += 1;
                }
                rightMotor.SetPercent(RIGHT_MOTOR_MODIFIER*rightMotorSpeed);
            }
        }
        else if(leftEncoder.Counts() > rightEncoder.Counts())
        {
            if(abs(leftMotorSpeed-rightMotorSpeed) <= PID_MAX_DIFF)
            {
                if(leftMotorSpeed > 0)
                {
                    leftMotorSpeed -= 1;
                } else {
                    leftMotorSpeed += 1;
                }
                leftMotor.SetPercent(LEFT_MOTOR_MODIFIER*leftMotorSpeed);
            }
        }
    }

}

/**
 * @brief followLine
 *      The robot follows a line for a specified degree
 * @param distance
 *      How long to follow a line for
 */
void followLine(float distance)
{

    typedef enum {
        onLine,
        leftOfLine,
        rightOfLine
    } lineFollowing;

    float rightVal, middleVal, leftVal;

    lineFollowing posistion;

    const float LINE_COLOR = 2;
\
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    distanceToTravel = distance;

    SD.Printf("Line Follow\n");

    unsigned int time = TimeNowSec();

    while(!drivedDistance() && (TimeNowSec() - time < 5))
    {
        rightVal = rightOpt.Value();
        middleVal = middleOpt.Value();
        leftVal = leftOpt.Value();

        if(middleVal > LINE_COLOR)
        {
            posistion = onLine;
        }

        if(rightVal > LINE_COLOR)
        {
            posistion = rightOfLine;
        }

        if(leftVal > LINE_COLOR)
        {
            posistion = leftOfLine;
        }

        switch (posistion) {
        case onLine:
            drive(LINE_FOLLOW_STRAIGHT);
            SD.Printf("onLine\n");
            break;

        case rightOfLine:
            drive(0.25 * LINE_FOLLOW, LINE_FOLLOW);
            SD.Printf("rightOfLine\n");
            break;

        case leftOfLine:
            drive(LINE_FOLLOW, 0.25 * LINE_FOLLOW);
            SD.Printf("leftOfLine\n");
            break;

        default:
            break;
        }
    }
    drive(STOP);
}

/**
 * @brief leftCountOffset
 *      Calculates the left encoder distance offset
 *      based off inertia from the current speed
 * @returns
 *      The offset value
 */
int leftCountOffset()
{
    float x = abs(leftMotorSpeed);
    //int ans = (int)(0.006143*x*x + .3007*x -2.757);
    int ans = (int)(0.006143*x*x + x -2.757);
    ans = (ans < 0) ? 0:ans;
    return ans;
}

/**
 * @brief leftCountOffset
 *      Calculates the right encoder distance offset
 *      based off inertia from the current speed
 * @returns
 *      The offset value
 */
int rightCountOffset()
{
    float x = abs(rightMotorSpeed);
    int ans = (int)(0.006143*x*x + x -2.757);
    ans = (ans < 0) ? 0:ans;
    return ans;
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
    bottomOfRamp = Coord(BOT_X, BOT_Y);
    topOfRamp = Coord(TOP_X, TOP_Y);
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


