#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHSD.h>

int main(void)
{
    //CLears the LCD of anything previously on it
    LCD.Clear(FEHLCD::Black);
    LCD.SetFontColor(FEHLCD::White);

    //Starting SD log file
    SD.OpenLog();
    SD.Printf("Initializing Log");

    //To display on screen. Probably will be changed later
    LCD.WriteLine("Libertatum Volcanus");

    //Closing SD log file
    SD.Printf("Closing Log");
    SD.CloseLog();

    //Ending Program
    return 0;
}
