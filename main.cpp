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
    Coord satelite, lever, seismoButton, core, coreDepo, home, center;

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

//Motor encoders
DigitalEncoder rightEncoder(MTR_ENCODE_R);
DigitalEncoder leftEncoder(MTR_ENCODE_L);

//Servos
FEHServo servoArm(SRV_ARM);
FEHServo servoForkLift(SRV_FRK_LFT);

//Corner bumb buttons
DigitalInputPin buttonTopLeft(BUTTON_TOP_LEFT);
DigitalInputPin buttonTopRight(BUTTON_TOP_RIGHT);
DigitalInputPin buttonBottomLeft(BUTTON_BOTTOM_LEFT);
DigitalInputPin buttonBottomRight(BUTTON_BOTTOM_RIGHT);

//CdS Cell
AnalogInputPin CdS(CDS_CELL);

//Optosensors
AnalogInputPin rightOpt(FEHIO::P1_0);
AnalogInputPin middleOpt(FEHIO::P1_1);
AnalogInputPin leftOpt(FEHIO::P1_2);

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

//////////////////////////////////////////END REGION//////////////////////////////////////////


/**
 * @brief main
 *      main program method
 * @return 0
 */
int main(void)
{
    initialSetup();

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
            if(readCdS() == RED)
            {
                /*
                //ACTUAL STATE
                SD.Printf("\n==STARTING COURSE==\n\n");
                SD.Printf("Driving Forward\n");
                driveDistance(MAX, 7.5);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                queState(startMoveSat);
                */


                //Performance test
                int angle = 90;
                SD.Printf("Running Performace test 2\n");
                LCD.Clear(BLACK);

                SD.Printf("Driving Forward\n");
                driveDistance(MAX, 7.5);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                SD.Printf("Turning\n");
                turnBlind(RIGHT, angle, TURN_MAIN);
                while(!checkTouchingSide(BACK))
                {
                    drive(-MAX);
                }
                drive(STOP);

                driveDistance(MAX, 8);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                SD.Printf("Turning\n");
                turnBlind(LEFT, angle, TURN_MAIN);

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
                drive(STOP);
                readCdS();
                Sleep(2);
                SD.Printf("Light Found!\n");

                driveDistance(-MAX, 3);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                turnBlind(RIGHT, angle, TURN_MAIN);

                SD.Printf("Driving till hit front\n");
                drive(MAX);
                while(!checkTouchingSide(FRONT))
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }

                //drive up not mud ramp

                driveDistance(-MAX, 7);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                turnBlind(LEFT, angle, TURN_MAIN);
                driveDistance(-MAX, 18);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                turnBlind(LEFT, angle+5, TURN_MAIN);
                driveDistance(MAX, 8);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                queState(shutdown);

                /*
                drive(-MAX);
                Sleep(0.05);
                drive(STOP);
                turnBlind(LEFT, angle, TURN_MAIN);
                drive(-MAX);

                SD.Printf("Moving to hit button\n");
                while(buttonBottomLeft.Value())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                SD.Printf("Button found, waiting 5 seconds\n");
                for(int i = 0; i < 11; i++)
                {
                    //Buzzer.Beep();
                    Sleep(0.5);
                    if(buttonBottomLeft.Value())
                    {
                        i = 0;
                        SD.Printf("[interactSismoBut] Haven't touched button for 5 seconds, resestting\n");
                        Buzzer.Buzz();
                        drive(-1*MAX);
                        while(buttonBottomLeft.Value());
                        drive(STOP);
                    }
                }
                SD.Printf("Moving to lever\n");
                driveDistance(MAX, 18);
                while(!drivedDistance())
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                drive(STOP);
                SD.Printf("Turning\n");
                turnBlind(LEFT, angle+40, TURN_MAIN);
                driveDistance(MAX, 16);
                while(!drivedDistance() && !checkTouchingSide(FRONT))
                {
                    drawRunningScreen();
                    if(enablePID)
                    {
                        PIDCheck();
                    }
                }
                SD.Printf("Lever found!\n");
                drive(STOP);
                queState(shutdown);
                */



            }
            break;

        case startMoveSat:
            driveToCoord(currentCourse.satelite);
            queState(moveToSat);
            extendRetractArm(true);
            break;

        case moveToSat:
            if(drivedDistance())
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
            if(drivedDistance())
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
            if(drivedDistance())
            {
                drive(STOP);
                queState(interactSismoBut);
            }
            break;

        case interactSismoBut:
            //turnBlind(RIGHT, rotateTo(EAST), TURN_MAIN);
            drive(MAX);
            while(!checkTouchingSide(FRONT))
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            drive(-MAX);
            Sleep(0.05);
            drive(STOP);
            turnBlind(LEFT, 90, TURN_MAIN);
            drive(-MAX);

            SD.Printf("Moving to hit button\n");
            while(buttonBottomLeft.Value())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            SD.Printf("Button found, waiting 5 seconds\n");
            for(int i = 0; i < 11; i++)
            {
                //Buzzer.Beep();
                Sleep(0.5);
                if(buttonBottomLeft.Value())
                {
                    i = 0;
                    SD.Printf("[interactSismoBut] Haven't touched button for 5 seconds, resestting\n");
                    Buzzer.Buzz();
                    drive(-1*MAX);
                    while(buttonBottomLeft.Value());
                    drive(STOP);
                }
            }
            queState(startMoveCore);
            break;

        case startMoveCore:
            drive(-MAX);
            Sleep(0.5);
            drive(STOP);
            driveToCoord(currentCourse.core);
            queState(moveToCore);
            break;

        case moveToCore:
            if(drivedDistance())
            {
                drive(STOP);
                queState(interactCore);
            }
            break;

        case interactCore:
            rotateTo((NORTH+EAST)/2);
            setForkLiftPos(SRV_FRK_NO_EXT);
            followLine(10);
            setForkLiftPos(SRV_FRK_FULL_EXT);
            drive(-MAX);
            Sleep(0.5);
            drive(STOP);
            queState(startMoveDepCore);
            break;

        case startMoveDepCore:
            driveToCoord(currentCourse.center);
            rotateTo(SOUTH);
            driveDistance(MAX, 18);
            while(!drivedDistance())
            {
                drawRunningScreen();
                if(enablePID)
                {
                    PIDCheck();
                }
            }
            drive(STOP);
            driveToCoord(currentCourse.coreDepo);
            queState(moveToDepCore);
            break;

        case moveToDepCore:
            if(drivedDistance())
            {
                drive(STOP);
                queState(interactDepCore);
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
            if(drivedDistance())
            {
                drive(STOP);

                rotateTo(SOUTH);
                drive(-1*MAX);
                while(!checkTouchingSide(BACK));
                drive(STOP);

                queState(shutdown);
            }
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
    //CLears the LCD of anything previously on it
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    //Set Servo Values
    servoArm.SetMax(SRV_MAX);
    servoArm.SetMin(SRV_MIN);
    servoArm.SetDegree(SERVO_NO_EXT);

    servoForkLift.SetMax(SRV_FRK_MAX);
    servoForkLift.SetMin(SRV_FRK_MIN);
    servoArm.SetDegree(SRV_FRK_NO_EXT);

    //Sets initial global variable values
    rightMotorSpeed = leftMotorSpeed = 0;
    numberOfStateChanges = 0;
    enablePID = false;
    strcpy(screenMessage,"INIT");

    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log\n");

    //Determine course robot located on
    if(USE_RPS)
    {
        RPS.InitializeTouchMenu();
    }

    while(INIT_CHECK && !verifyStartConditions())
    {
        Buzzer.Buzz(5);
    }

    //Gets if running course and what course it is on
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

    SD.Printf("Queing state ");
    SD.Printf(STATE_NAMES[nextState]);
    SD.Printf("\n");

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
    servoFrkPer = (percent/100)*(SRV_FRK_MAX-SRV_FRK_MIN) + SRV_FRK_MIN;
    servoForkLift.SetDegree(servoFrkPer);
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
                * (WHEEL_RAD * 2 * M_PI)) > distanceToTravel;
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
    float rMPerToChange = (rMPercent - rightMotorSpeed)/10.0;
    float lMPerToChange = (lMPercent - leftMotorSpeed)/10.0;

    for(int i = 0; i < 10; i++)
    {
        rightMotorSpeed += rMPerToChange;
        leftMotorSpeed += lMPerToChange;

        rightMotor.SetPercent(-rightMotorSpeed);
        leftMotor.SetPercent(leftMotorSpeed);

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
    distanceToTravel = distance;
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
    enablePID = false;
    int rDirection = (goRight) ? 1:-1;
    int lDirection = (goRight) ? -1:1;

    drive(rDirection * speed, lDirection * speed);
    SD.Printf("Turning %s at speed %f", (goRight) ? "right":"left", speed);
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

    float wheelSeperationRad = 3.5 * 2.5;

    SD.Printf("Turning blind %f degrees", degree);

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

    while(RPS.Heading() < 0)
    {
        Buzzer.Buzz(5);
        currHeading = RPS.Heading();
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

    turnBlind(turnDirection, abs(RPS.Heading() - directionToHead), TURN_MAIN);


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
    /*
    if(leftEncoder.Counts() > rightEncoder.Counts())
    {
        if(abs(leftMotorSpeed-rightMotorSpeed) <= PID_MAX_DIFF)
        {
            if(rightMotorSpeed > 0)
            {
                rightMotorSpeed -= 1;
            } else {
                rightMotorSpeed += 1;
            }
            rightMotor.SetPercent(rightMotorSpeed);
        }
    }
    else if(leftEncoder.Counts() < rightEncoder.Counts())
    {
        if(abs(leftMotorSpeed-rightMotorSpeed) <= PID_MAX_DIFF)
        {
            if(leftMotorSpeed > 0)
            {
                leftMotorSpeed -= 1;
            } else {
                leftMotorSpeed += 1;
            }
            leftMotor.SetPercent(leftMotorSpeed);
        }
    }
    */

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

    const float LINE_COLOR = 1.5;
\
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    distanceToTravel = distance;

    while(!drivedDistance())
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
            break;

        case rightOfLine:
            drive(0.25 * LINE_FOLLOW, LINE_FOLLOW);
            break;

        case leftOfLine:
            drive(LINE_FOLLOW, 0.25 * LINE_FOLLOW);
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
    center = Coord(CEN_X, CEN_Y);
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


