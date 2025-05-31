/*
* Name: Alejandro Caro and Joshua Romeo
* Semester: Spring 2025
* Laboratory Section: Section 1
* Date: 04/05/2025
* Lab Number: Lab 10 - Final Project
* Description: This code implements a line-following robot using the MSP432, IR sensors,
*              bumper interrupts, and PWM motor control with LED feedback.
*/

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum {LED1OFF, ONEHZ, TWOHZ, FOURHZ, ON, LED1MAINTAIN} led1State;
led1State LED1State = 0;

typedef enum {RED,BLUE,GREEN,YELLOW,MAGENTA,CYAN,WHITE,ALL_LED2_OFF,steadyLED2} led2State;
led2State LED2State = 0;

typedef enum {MOTOROFF, MOTORFORWARD, MOTORREVERSE, MOTORTEST,MOTORCOAST, MOTORBRAKE} motorState; ///this is built in MotorDriver
motorState MotorLeftState = 0, MotorRightState = 0;

typedef enum {allLeft, moreLeft, equal, allRight, moreRight, LOST, STANDBY} sensorCondition;
sensorCondition sensorCon = LOST;
uint8_t SensorLostTrack = 0;   //reset the count to zero. This variable we are using as a tool to create a count for when lost track case for the sensors

typedef enum {BUTTON_OFF, BUTTON_ON} BumperSwitchStates;
BumperSwitchStates BumperZeroState = 0;
BumperSwitchStates BumperOneState  = 0;
BumperSwitchStates BumperTwoState  = 0;
BumperSwitchStates BumperThreeState = 0;
BumperSwitchStates BumperFourState  = 0;
BumperSwitchStates BumperFiveState  = 0;

uint8_t StoredValuesforSensor = 0;
volatile uint8_t leftStatus = 0, rightStatus = 0;

uint16_t DUTY = 37;
#define PERIOD 10
#define CLOCKDIVIDER 48
#define RIGHTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_3
#define LEFTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_4

Timer_A_PWMConfig timerPWMConfig;



void configPWMTimer(uint16_t, uint16_t, uint16_t,uint16_t, uint32_t);
void Config432IO();
void ConfigRobotIO();

void RGBDriver(led2State LEDColor);
void LEDsDriver();//for robot LEDs
void LEDStartUp();
void UpdateLineLED();

void bumperSwitchesHandler();
void MotorDriver();


void readLine();
void IRSensorMakeOutputHigh();
void RobotInputPinsConfiguration();//sensors

void IRProcessData();
void LineFollowing();



int main(void)
{
    MAP_WDT_A_holdTimer();  // stop WDT

    Config432IO();//setting pin config

    ConfigRobotIO();//for setting the robot inputs and output pins

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);                    // Left Wheel enable
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);                    // Left Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);                    // Left Wheel Sleep

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);                    // Right Wheel enable
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);                    // Right Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);                    // Right Wheel Sleep

    //This configPWTimer will help us create the sigals for both our LEFT and Right motor and determine how the wheels will move

    configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, RIGHTCHANNEL, TIMER_A0_BASE);//RIGHTCHANNEL IS CONNECTED TO TIMER_A_CAPTURECOMPARE_REGISTER_4
    configPWMTimer(PERIOD, CLOCKDIVIDER, DUTY, LEFTCHANNEL, TIMER_A0_BASE);//LEFTCHANNEL IS CONNECTED TO TIMER_A_CAPTURECOMPARE_REGISTER_3

    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    //Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE); WE ARE NOT USING IT BUT MIGHT HAVE TO FOR SPECIFIC MOTOR TURNS
    LEDStartUp();

    while(1)
    {
        if(BumperZeroState == BUTTON_ON)         // line tracking mode
        {
            BumperOneState = BUTTON_OFF;
            BumperTwoState = BUTTON_OFF;
            BumperThreeState = BUTTON_OFF;
            BumperFourState = BUTTON_OFF;
            BumperFiveState = BUTTON_OFF;
            while(BumperZeroState == BUTTON_ON && BumperOneState == BUTTON_OFF && BumperTwoState == BUTTON_OFF && BumperThreeState == BUTTON_OFF  && BumperFourState == BUTTON_OFF && BumperFiveState == BUTTON_OFF)
            {
                readLine();
                IRProcessData();
                UpdateLineLED();
                LineFollowing();
            }
            MotorLeftState = MOTOROFF;
            MotorRightState = MOTOROFF;
            MotorDriver();
            __delay_cycles(6000000);
            if((BumperZeroState == BUTTON_ON && BumperOneState == BUTTON_ON) || (BumperZeroState == BUTTON_ON && BumperTwoState == BUTTON_ON) || (BumperZeroState == BUTTON_ON && BumperThreeState == BUTTON_ON) || (BumperZeroState == BUTTON_ON && BumperFourState == BUTTON_ON) || (BumperZeroState == BUTTON_ON && BumperFiveState == BUTTON_ON))
            {
                MotorLeftState = MOTORREVERSE;
                MotorRightState = MOTORREVERSE;
                MotorDriver();
                __delay_cycles(1500000);
            }
            MotorLeftState = MOTOROFF;
            MotorRightState = MOTOROFF;
            MotorDriver();
            BumperOneState = BUTTON_OFF;
            BumperTwoState = BUTTON_OFF;
            BumperThreeState = BUTTON_OFF;
            BumperFourState = BUTTON_OFF;
            BumperFiveState = BUTTON_OFF;
        }
        else //this will run standby mode
        {
            readLine();
            IRProcessData();
            UpdateLineLED();
            __delay_cycles(750000);
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
            __delay_cycles(750000);
        }
    }
}
/*
* Function Name: MotorDriver()
* makes sure all variations of motor movement happen
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void MotorDriver()
{
    LEDsDriver();
    switch(MotorLeftState)
    {
        case MOTOROFF:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);// SLEEP LEFT WHEEL
        break;
        case MOTORFORWARD:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); // ENABLE LEFT WHEEL
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);  // DIRECTION LEFT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // SLEEP LEFT WHEEL
        break;
        case MOTORREVERSE:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2);// ENABLE LEFT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);// DIRECTION LEFT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);// SLEEP LEFT WHEEL
        break;
        case MOTORCOAST:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);// SLEEP LEFT WHEEL
        break;
        case MOTORBRAKE:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);// ENABLE LEFT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);//SLEEP LEFT WHEEL
        break;
    }
    switch(MotorRightState)
    {
    case MOTOROFF:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLEEP RIGHT WHEEL
        break;
        case MOTORFORWARD:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); //ENABLE RIGHT WHEEL
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);  //DIRECTION RIGHT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLEEP RIGHT WHEEL
        break;
        case MOTORREVERSE:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);// ENABLE RIGHT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);//DIRECTION RIGHT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);// SLEEP RIGHT WHEEL
        break;
        case MOTORCOAST:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);//SLEEP RIGHT WHEEL
        break;
        case MOTORBRAKE:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);//ENABLE RIGHT WHEEL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);//SLEEP RIGHT WHEEL
        break;
    }
}
/*
* Function Name: Config432IO
* This function configures the onboard MSP432 LEDs as output pins.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void Config432IO()
{
    // set output pins
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);   // Red LED1
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);   // Red LED2
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);   // Green LED2
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);   // Blue LED2
    // turn off all LEDs initially
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
}
/*
* Function Name: ConfigRobotIO
* This function initializes robot-specific I/O including motors, IR array, and bumper switches.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void ConfigRobotIO()
{
    // These lines will set the pins as inputs with pull up resistors for all bumper switch buttons
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN0);// bs0
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN2);// bs1
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);// bs2
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN5);// bs3
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN6);// bs4
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN7);// bs5
    //These lines will set enable interrupt for all bumper switches
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);// bs0
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN2);// bs1
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN3);// bs2
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN5);// bs3
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6);// bs4
    MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN7);// bs5
    //these lines will set the interrput edge select for all the bumper switches
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN0,GPIO_HIGH_TO_LOW_TRANSITION);// bs0
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN2,GPIO_HIGH_TO_LOW_TRANSITION);// bs1
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN3,GPIO_HIGH_TO_LOW_TRANSITION);// bs2
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN5,GPIO_HIGH_TO_LOW_TRANSITION);// bs3
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN6,GPIO_HIGH_TO_LOW_TRANSITION);// bs4
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P4, GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);// bs5
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN0);// bs0
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN2);// bs1
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN3);// bs2
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN5);// bs3
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);// bs4
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7);// bs5
    // when any of the bumper switches is pressed ,it triggers an interrupt and  bumperSwitchesHandler function will be called.
    MAP_GPIO_registerInterrupt(GPIO_PORT_P4, bumperSwitchesHandler);

    //set to low in default state for smooth motor configurations
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2); // Left Wheel enable
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); // Right Wheel enable
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4); // Left Wheel Direction
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5); // right Wheel Direction
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); // Left Wheel Sleep
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // right Wheel Sleep

    // Reconfigure P2.7 to use TA0 Channel 4
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    // Reconfigure P2.6 to use TA0 Channel 4
       MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6,GPIO_PRIMARY_MODULE_FUNCTION);

    //set to low in default state for LEDS
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);// Left Front LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5);// Right Front LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN6);// Left Back LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN7);// Right Back LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);// CNTL Even for IR Sensor Array
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2);// CNTL Odd for IR Sensor Array

    //These lines will start everything as low output pins
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2);// Left Wheel enable
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);// Right Wheel enable
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);// Left Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);// right Wheel Direction
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);// Left Wheel Sleep
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);// right Wheel Sleep
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);// Left Wheel PWM
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6);// Right Wheel PWM
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);// Left Front LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);// Right Front LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);// Left Back LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);// Right Back LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);// CNTL Even for IR Sensor Array
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);// CNTL Odd for IR Sensor Array
}

/*
* Function Name: bumperSwitchesHandler
* This is the ISR for bumper switches, toggles the state variables when triggered.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
// Here we generate our Interrupt service routine
void bumperSwitchesHandler()
{
    uint16_t status;
    _delay_cycles(45000);
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    switch(status)
    {
    case GPIO_PIN0:
        if(BumperZeroState == BUTTON_ON)
        {
            BumperZeroState = BUTTON_OFF;
        }
        else
        {
            BumperZeroState = BUTTON_ON;
        }
        break;
        case GPIO_PIN2:
        if(BumperOneState == BUTTON_ON)
        {
            BumperOneState = BUTTON_OFF;
        }
        else
        {
            BumperOneState = BUTTON_ON;
        }
        break;
        case GPIO_PIN3:
        if(BumperTwoState == BUTTON_ON)
        {
            BumperTwoState = BUTTON_OFF;
        }
        else
        {
            BumperTwoState = BUTTON_ON;
        }
         break;
         case GPIO_PIN5:
        if(BumperThreeState == BUTTON_ON)
        {
            BumperThreeState = BUTTON_OFF;
        }
        else
        {
            BumperThreeState = BUTTON_ON;
        }
        break;
        case GPIO_PIN6:
        if(BumperFourState == BUTTON_ON)
{
            BumperFourState = BUTTON_OFF;
        }
         else
        {
            BumperFourState = BUTTON_ON;
        }
        break;
        case GPIO_PIN7:
        if(BumperFiveState == BUTTON_ON)
        {
            BumperFiveState = BUTTON_OFF;
        }
        else
        {
            BumperFiveState = BUTTON_ON;
        }
        break;
    }
}

/*
* Function Name: RGBDriver
* This function sets the RGB LED2 to display a color based on the enum input.
* Inputs: led2State LEDColor
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void RGBDriver(led2State LEDColor)
{
    switch(LEDColor)
    {   // P2.2 IS RED, P2.1 IS GREEN, P2.2 IS BLUE
        case RED:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN OFF
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE OFF
        break;
        case BLUE:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED OFF
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN OFF
        break;
        case GREEN:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED OFF
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);//GREEN OFF
        break;
        case YELLOW:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED ON
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE OFF
        break;
        case MAGENTA:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED ON
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN OFF
        break;
        case CYAN:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);//BLUE ON
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);//GREEN ON
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED OFF
        break;
        case WHITE:
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED ON
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN ON
           MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE OFF
        break;
        case ALL_LED2_OFF:
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);//RED OFF
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);//GREEN OFF
           MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);//BLUE OFF
        break;
        case steadyLED2://NOT USED BUT ITS HERE JUST IN CASE WE NEED TO IMPLEMENT LATER
        break;
       }
}

/*
* Function Name: LEDsDriver
* This function turns on LEDs located on the front/back of the robot depending on motor direction.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 3/24/25
*/
void LEDsDriver()
{
    switch(MotorLeftState)
    {
    case MOTORFORWARD:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);// FRONT LEFT LED
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);// BACK LEFT LED
    break;
    case MOTORREVERSE:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);// FRONT LEFT LED
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);// BACK LEFT LED
    break;
    case MOTOROFF:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);// FRONT LEFT LED
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);// BACK LEFT LED
    break;
    case MOTORTEST:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);// FRONT LEFT LED
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);// BACK LEFT LED
    break;
    }
    switch(MotorRightState)
    {
    case MOTORFORWARD:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);// FRONT RIGHT LED
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); // BACK RIGHT LED
    break;
    case MOTORREVERSE:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);// FRONT RIGHT LED
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);// BACK RIGHT LED
    break;
    case MOTOROFF:
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);// FRONT RIGHT LED
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);// BACK RIGHT LED
    break;
    case MOTORTEST:
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);// FRONT RIGHT LED
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);// BACK RIGHT LED
    break;
    }
}


/*
* Function Name: LEDStartUp
* This function flashes all LEDs on the robot for a startup animation.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 3/24/25
*/

void LEDStartUp()
{
    //These lines set the pins high for LEDS
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);// SET PIN HIGH FOR LED1
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);// SET PIN HIGH FOR LED2 Red
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);// SET PIN HIGH FOR LED2 Green
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);// SET PIN HIGH FOR LED2 Blue

    //These lines set the pins high for the robot
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);// FRONT LEFT LED
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5);// FRONT RIGHT LED
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6);// BACK LEFT LED
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7);// BACK RIGHT LED

    __delay_cycles(6000000);

    //These lines will set the output pins low at startup for the LEDs on MSP432
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);// LED1
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);// LED2 Red
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);// LED2 Green
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);// LED2 Blue

    //These lines will set the output pins low at startup for the LEDs on robot
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);// Left Front LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5);// Right Front LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6);// Left Back LED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7);// Right Back LED
}

/*
* Function Name: IRSensorMakeOutputHigh
* This function charges the IR sensor array by setting P7 pins high.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void IRSensorMakeOutputHigh()
{
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN7);
}

/*
* Function Name: RobotInputPinsConfiguration
* This function configures the IR sensor array pins as high-impedance inputs.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void RobotInputPinsConfiguration()
{
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN5);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN6);
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN7);
}

/*
* Function Name: readLine
* This function reads the charged state of the IR sensors and stores them in a global variable.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void readLine()
{
    StoredValuesforSensor = 0;
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2);



    IRSensorMakeOutputHigh();       // make sensors high
    __delay_cycles(30);             // wait until fully charged
    RobotInputPinsConfiguration();            // switch to high impedance
    __delay_cycles(3000);          // wait optimal delay time


    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN0))
        StoredValuesforSensor |= BIT0;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN1))
        StoredValuesforSensor |= BIT1;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN2))
        StoredValuesforSensor |= BIT2;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN3))
        StoredValuesforSensor |= BIT3;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4))
        StoredValuesforSensor |= BIT4;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN5))
        StoredValuesforSensor |= BIT5;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN6))
        StoredValuesforSensor |= BIT6;
    if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN7))
        StoredValuesforSensor |= BIT7;
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2);
}

/*
* Function Name: IRProcessData
* This function evaluates sensor readings and sets sensor condition based on left/right values.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void IRProcessData()
{
    leftStatus = 0;//RESET TO ZERO
    rightStatus = 0;//RESET TO ZERO
    if (StoredValuesforSensor & BIT7)
        leftStatus++;
    if (StoredValuesforSensor & BIT6)
        leftStatus++;
    if (StoredValuesforSensor & BIT5)
        leftStatus++;
    if (StoredValuesforSensor & BIT4)
        leftStatus++;
    if (StoredValuesforSensor & BIT3)
        rightStatus++;
    if (StoredValuesforSensor & BIT2)
        rightStatus++;
    if (StoredValuesforSensor & BIT1)
        rightStatus++;
    if (StoredValuesforSensor & BIT0)
        rightStatus++;


    if(leftStatus > rightStatus +2) sensorCon = allLeft;

    else if(rightStatus > leftStatus +2)sensorCon = moreRight;
    else if (leftStatus > rightStatus+1)
        sensorCon = moreLeft;
    else if (rightStatus > leftStatus+1)
        sensorCon = allRight;
    else if (rightStatus == 0 && leftStatus == 0)
    {
        sensorCon = LOST;
    }
    else
        sensorCon = equal;

}
/*
* Function Name: UpdateLineLED
* This function updates RGB LED2 and LED1 depending on the current line-following condition.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void UpdateLineLED()
{
    switch(sensorCon)
    {
        case allLeft:
            RGBDriver(RED);
        break;
        case moreLeft:
            RGBDriver(YELLOW);
        break;
        case equal:
            RGBDriver(GREEN);
        break;
        case allRight:
            RGBDriver(CYAN);
        break;
        case moreRight:
            RGBDriver(BLUE);
        break;
        case LOST:
            RGBDriver(WHITE);
        break;
    }
    if(sensorCon != LOST)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
     else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

/*
* Function Name: LineFollowing
* This function implements the FSM logic for following a black line using IR sensors.
* Inputs: None
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
//new implementation to run the track
void LineFollowing()
{
    //This function will turn on the motors using MotorDriver
    switch(sensorCon)
    {
    case allLeft://HARD LEFT LOGIC
        DUTY = 50;
        MotorLeftState = MOTORREVERSE;
        MotorRightState = MOTORFORWARD;
        MotorDriver();
        __delay_cycles(200000);
    break;
    case moreLeft: //BRIEF LEFT LOGIC
        DUTY = 33;
        MotorLeftState = MOTOROFF;
        MotorRightState = MOTORFORWARD;
        MotorDriver();
        __delay_cycles(2500); // TEST1 WITH 45000,TEST2 WITH 50000,TEST 3 WITH 80000 SO FAR 2500 works best
    break;
    case equal: //FORWARD LOGIC
        DUTY = 80;
        MotorLeftState = MOTORFORWARD;
        MotorRightState = MOTORFORWARD;
        MotorDriver();
         __delay_cycles(20000);
    break;
    case allRight: //SLIGHT TURN RIGHT
        DUTY = 33;
        MotorLeftState = MOTORFORWARD;
        MotorRightState = MOTOROFF;
        MotorDriver();
        __delay_cycles(2500); // initially 80000 --45000 --5000
    break;
    case moreRight:  //TURNS RIGHT SHARP
        DUTY = 50;
        MotorLeftState = MOTORFORWARD;
        MotorRightState = MOTORREVERSE;
        MotorDriver();
        __delay_cycles(200000);
    break;
    //180 correction logic
    case LOST: //REVERSES TILL IT FINDS TRACK
        SensorLostTrack++;
        if(SensorLostTrack < 8)//correction from the right side
            {
            MotorLeftState = MOTORREVERSE;
            MotorRightState = MOTORREVERSE;
            MotorDriver();
            __delay_cycles(75000);
}
        else if(SensorLostTrack >=5)//correctionfrom the left side
            {
            MotorLeftState = MOTOROFF;
            MotorRightState = MOTOROFF;
            _delay_cycles(50000);
            DUTY = 50;
            MotorLeftState = MOTORFORWARD;
            MotorRightState = MOTORREVERSE;
            MotorDriver();
            _delay_cycles(750000);
            SensorLostTrack = 0;

            }

        break;
        case STANDBY:
            MotorLeftState = MOTOROFF;
            MotorRightState = MOTOROFF;
            MotorDriver();
         break;

    }
}

/*
* Function Name: configPWMTimer
* This function initializes PWM configuration for motor channels.
* Inputs: uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty, uint16_t channel, uint32_t timer
* Return: None
* Author: Alejandro Caro & Joshua Romeo
* Date Written: 4/5/25
*/
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty, uint16_t channel, uint32_t timer)
{
    uint16_t dutyCycle = duty*clockPeriod/100;
    timerPWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timerPWMConfig.clockSourceDivider = clockDivider;
    timerPWMConfig.timerPeriod = clockPeriod;
    timerPWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    timerPWMConfig.compareRegister = channel;
    timerPWMConfig.dutyCycle = dutyCycle;
    MAP_Timer_A_generatePWM(timer, &timerPWMConfig);
    MAP_Timer_A_stopTimer(timer);
}
