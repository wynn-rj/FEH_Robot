#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#include <FEHBattery.h>
#include <FEHBuzzer.h>
#include <FEHServo.h>
#include <FEHMotor.h>
#include <robotdefinitions.h>



/////////////////////////////////////REGION: Declerations/////////////////////////////////////

/* Global Variable Decleration */

FEHMotor leftMotor(DT_MOTOR_L, DT_MOTOR_LV);
FEHMotor rightMotor(DT_MOTOR_R, DT_MOTOR_RV);

FEHServo servoArm(SRV_ARM);

AnalogInputPin buttonTopLeft(BUTTON_TOP_LEFT);
AnalogInputPin buttonTopRight(BUTTON_TOP_RIGHT);
AnalogInputPin buttonBottomLeft(BUTTON_BOTTOM_LEFT);
AnalogInputPin buttonBottomRight(BUTTON_BOTTOM_RIGHT);

DigitalInputPin CdS(CDS_CELL);


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

//////////////////////////////////////////END REGION//////////////////////////////////////////

void labTest()
{
    /*
    FEHServo servo(FEHServo::Servo0);
    AnalogInputPin CdS(FEHIO::P0_0);
    servo.SetMin(500);
    servo.SetMax(2495);

    while(true)
    {
        servo.SetDegree((56*CdS.Value()));
        LCD.WriteRC(CdS.Value(),0,0);
    }
    */
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
    currentCourse = initMenu();

    //Set Servo Values
    servoArm.SetMax(SRV_MAX);
    servoArm.SetMin(SRV_MIN);

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


