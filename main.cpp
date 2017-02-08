#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHSD.h>
#include <FEHBattery.h>
#include <FEHBuzzer.h>

/* Define colors for parts of menus */
#define MENU_C WHITE
#define TEXT_C GOLD
#define SELT_C RED
#define SHOW_C BLUE
#define HI_C GREEN

/* Define time for beep */
#define beep_t 10 // int milliseconds

/////////////////////////////////////REGION: Declerations/////////////////////////////////////

/* Structure declerations */

//Course Struct used to for proteus to know what course it is on and extra info for that course
typedef struct Course
{
public:
    Course(char);

private:
    char courseLetter;
} Course;

/* Function declerations */

Course initMenu();

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


    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log");

    //Determine course robot located on
    currentCourse = initMenu();

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

    Buzzer.Buzz(beep_t);


    while(noCourseSelected)
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
                    //States selected course to screen and log
                    LCD.Clear(BLACK);
                    LCD.WriteLine("Course   Selected");
                    LCD.WriteRC(main_label[n], 0, 7);
                    SD.Printf("Selecting Course: ");
                    SD.Printf(main_label[n]);
                    SD.Printf("\n");

                    selectedCourse = new Course(main_label[n][0]);

                    noCourseSelected = false;
                    break;
                }
            }
        }
    }
}

/////////////////////////////////////REGION: Struct Definitions/////////////////////////////////////

/**
 * @brief Course::Course
 *      Constructor for the Course Struct
 * @param courseLet
 *      A char that is used to define which
 *      course the robot is starting on
 */
Course::Course(char courseLet)
{
    Course::courseLetter = courseLet;
}

/////////////////////////////////////////////END REGION/////////////////////////////////////////////


,
