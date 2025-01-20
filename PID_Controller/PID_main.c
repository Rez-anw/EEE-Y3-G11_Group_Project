// Included Files
#include "driverlib.h"
#include "device.h"
#inculde "F28x.Project.h"

// PID Controller Variables
float kp;
float ki;
float kd;
float prevError;
float integral;
float outPutMin;
float outPutMax;

float setPointX = 0.0;
float setPointY = 0.0;
float measurementX = 0.0;
float measurementY = 0.0;
float motorOutputX = 0.0;
float motorOutputY = 0.0;


// Main
void main(void)
{
    // Initialise
    InitSysCtrl();
    InitGpio();
    DINT;
    InitPieCtrl();
}

// Update PID output
float PID()
{

}

//
// End of File
//
