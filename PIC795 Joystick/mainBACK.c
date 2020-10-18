/**********************************************************************************
 * PROJECT: PIC795 MD13S CONTOLLER
 * main.c
 * 
 * Compiled for PIC32MX795 XC32 compiler version 1.30 
 * For Robotnik MD13S Controller Board Rev 1.0
 * Adapted from Robotnik_Brain_Board
 * 
 * 8-13-20: Set up IO for new board
 * 
 ***********************************************************************************/
// For 26:1 motors
#define PWM_OFFSET 0 // Was 220
#define KP 8.0
#define KI 0.01
#define KD 10.0 
#define MAX_COMMAND_COUNTS 850
#define MIN_COMMAND_COUNTS 100

#include <xc.h>
#include "HardwareProfile.h"
#include "uart2.h"
#include "Delay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "I2C_4BUS_EEPROM_PIC32.h"

#include "FSIO.h"
#include "Delay.h"
#include "Defs.h"
#include "SD-SPI.h"
#define _SUPPRESS_PLIB_WARNING

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

/** I N C L U D E S **********************************************************/

#define false FALSE
#define true TRUE

#define HALTED 0
#define LOCAL 1
#define JOG 2
#define REMOTE 3

#define TEST_OUT LATCbits.LATC1

#define PWM1 OC1RS
#define PWM2 OC2RS
#define PWM3 OC3RS
#define PWM4 OC4RS
#define PWM5 OC5RS

#define EncoderOne TMR1
#define EncoderTwo TMR5
#define EncoderThree TMR3
#define EncoderFour TMR4

#define EncoderOneDir PORTEbits.RE9
#define EncoderTwoDir PORTEbits.RE5
#define EncoderThreeDir PORTEbits.RE7
#define EncoderFourDir PORTEbits.RE8



#define DIR1 LATGbits.LATG12
#define DIR2 LATDbits.LATD13
#define DIR3 LATDbits.LATD11
#define DIR4 LATAbits.LATA5
#define DIR5 LATDbits.LATD10


#define EEPROM_WRITE_PROTECT LATDbits.LATD8
#define LED LATDbits.LATD9
#define RS485_ENABLE LATBbits.LATB5
#define SW1 PORTBbits.RB0
#define SW2 PORTBbits.RB1
#define SW3 PORTBbits.RB2
#define SW4 PORTCbits.RC13
#define SW5 PORTDbits.RD7

// UARTS
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define RS485uart UART5
#define RS485bits U5STAbits
#define RS485_VECTOR _UART_5_VECTOR


#define CR 13
#define LF 10
#define BACKSPACE 8
#define SPACE 32
#define ESCAPE 27
#define RIGHT_ARROW 67
#define LEFT_ARROW 68
#define UP_ARROW 65
#define DOWN_ARROW 66
#define MAXNUM 16
#define MAXBUFFER 255

struct PIDtype
{
    short error[FILTERSIZE];
    long  sumError;    
    float kP;
    float kI;
    float kD;
    short PWMoffset;
    short PWMvalue;
    short ADActual;
    short ADCommand;
    short ErrorCounter;
    unsigned char saturation;
    unsigned char reset;    
} PID[NUMMOTORS];


/** V A R I A B L E S ********************************************************/
unsigned char NUMbuffer[MAXNUM + 1];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = false;
unsigned char HOSTTxBuffer[MAXBUFFER+1]; 
unsigned char MEMORYBuffer[MAXBUFFER+1]; 
unsigned char displayFlag = false;

unsigned char XBEERxBuffer[MAXBUFFER+1];
unsigned char XBEERxBufferFull = false;
long ActualXBEEBaudRate;

unsigned char RS485RxBufferFull = false;
unsigned char RS485TxBufferFull = false;
unsigned char RS485RxBuffer[MAXBUFFER+1];
unsigned char RS485RxBufferCopy[MAXBUFFER+1];
unsigned char ServoData[MAXBUFFER+1];
short servoPositions[MAXSERVOS];
long ActualRS485BaudRate = 0;
int timeout = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void setMotorOne(short TargetDutyCycle);
void setMotorTwo(short TargetDutyCycle);

static void InitializeSystem(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART();
extern unsigned char CheckCRC (unsigned char *ptrRxModbus, short RxModbusLength);
unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData);
void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand);
unsigned char processPacketData(short packetDataLength, unsigned char *ptrPacketData, short *numServos, short *ptrServoPositions, unsigned char *command, unsigned char *subCommand);
unsigned char SendReceiveSPI(unsigned char dataOut);
void ResetPID();
long PIDcontrol(long servoID, struct PIDtype *PID);

unsigned char ADready = false;
unsigned char EnableAD = false;
unsigned char DATABufferFull = false;
void ConfigAd(void);
void ClearCopyBuffer();
void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii);

enum
{
    QUAD_NONE = 0,
    QUAD_ONE,
    QUAD_TWO,
    QUAD_THREE
};

enum
{
    STANDBY = 0,
    RECORD,
    PLAY,
    PAUSE_RECORD,
    PAUSE_PLAY,
    HALT
};

short time, seconds = 0, minutes = 0, hundredths = 0;
unsigned char TestMode = false;
#define MAXPOTS 10
unsigned short ADresult[MAXPOTS];
unsigned short PortBRead = 0;
unsigned char PortBflag = false;
unsigned char intFlag = false;
unsigned char memoryFlag = false;

int main(void) 
{
    short i = 0, j = 0, p = 0, q = 0;        
    long PWMvalue = 0;    
    unsigned char command, subCommand, ch;
    int JogPWM = 0;
    float floValue;            
    unsigned char HBridgeData[4];
    short LEDcounter = 0;
    short length = 0;
    unsigned short numBytes;
    short dataLength = 0;
    unsigned char PacketData[MAXBUFFER];
    short numServos;
    short ServoPositions[MAXSERVOS] = {512,512,512,512};
    unsigned char MessageOut[] = "You can eat the driver and his gloves\r\n ";
    unsigned ADdisplay = true;
    short SPIcounter = 0;
    short PotValue = 0;
    
    unsigned char runMode = LOCAL;
    unsigned char previousRunMode = LOCAL;
    unsigned char testChar = 'A';
    
    ResetPID();
        
    DelayMs(10);
    InitializeSystem();      
    mAD1IntEnable(INT_ENABLED);
    DelayMs(10);
    
    RS485_ENABLE = 0;

    DelayMs(100);
    printf("\r\rTesting ADC and PWM...");
    
    DIR1 = DIR2 = DIR3 = DIR4 = DIR5 = 1;
    OC1RS = OC2RS = OC3RS = OC4RS = OC5RS = 2000;

    
    /*
    if (runMode) printf("\r\rBrain Board - PID for Cytron MD13S board. CHANNEL #0 MOTOR ON: ");
    else printf("\r\rBrain Board - PID for Cytron MD13S board. CHANNEL #0 MOTORm OFF: ");
    
    if (previousRunMode == LOCAL) printf("LOCAL MODE");
    else if (previousRunMode == REMOTE) printf("REMOTE MODE");
    else if (previousRunMode == JOG) printf("JOG MODE");
    else printf("MODE ERROR");    
    */
    
    /*
    printf("\r\rTesting Pot #1...");
    while(1)
    {
        DelayMs(100);
        PotValue = ADresult[0];
        mAD1IntEnable(INT_ENABLED);
        printf("\rPot #1: %d", PotValue);
        PWM1 = PWM2 = PWM3 = PWM4 = PotValue * 4;
    }
    */
    
    while(1) 
    {         
        DelayMs(1);
        if (ADready)
        {
            ADready = false;
            printf("\rAD12: %d, AD13: %d, AD14: %d, AD15: %d, ", ADresult[6], ADresult[7], ADresult[8], ADresult[9]);
            mAD1IntEnable(INT_ENABLED);
            DelayMs(100);
        }
        
        if (RS485RxBufferFull)
        {
            RS485RxBufferFull = false;
            RS485_ENABLE = 1;
            DelayMs(10);
            while(!UARTTransmitterIsReady(RS485uart));
            UARTSendDataByte (RS485uart, '>');
            length = strlen(RS485RxBuffer);
            for (i = 0; i < length; i++)
            {
                ch = RS485RxBuffer[i];
                while(!UARTTransmitterIsReady(RS485uart));
                UARTSendDataByte (RS485uart, ch);
            }
            DelayMs(10);
            RS485_ENABLE = 0;
        }
        
        if (XBEERxBufferFull)
        {
            XBEERxBufferFull = false;
            while(!UARTTransmitterIsReady(XBEEuart));
            UARTSendDataByte (XBEEuart, '>');
            length = strlen(XBEERxBuffer);
            for (i = 0; i < length; i++)
            {
                ch = XBEERxBuffer[i];
                while(!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte (XBEEuart, ch);
            }
        }
        
            
            /*
            dataLength =  decodePacket(RS485RxBufferCopy, PacketData);            
            if (!processPacketData(dataLength, PacketData, &numServos, ServoPositions, &command, &subCommand)) 
                printf("\rCRC ERROR");
            else PrintServoData(numServos, ServoPositions, command, subCommand);                                    
            ClearCopyBuffer();
            
            LEDcounter++;
            if (LEDcounter > 4)
            {
                LEDcounter = 0;
                if (LED) LED = 0;
                else LED = 1;
            }            
            */
        
        /*
        if (intFlag)
        {
            intFlag = false;
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;

            if (runMode)
            {               
                mAD1IntEnable(INT_ENABLED);
                for (i = 0; i < NUMMOTORS; i++)
                {                            
                    PID[i].ADCommand = (short)(ADresult[i]);
                    if (PID[i].ADCommand > MAX_COMMAND_COUNTS) PID[i].ADCommand = MAX_COMMAND_COUNTS; 
                    if (PID[i].ADCommand < MIN_COMMAND_COUNTS) PID[i].ADCommand = MIN_COMMAND_COUNTS; 
                    PID[i].ADActual = (short)(ADresult[i+4]);                    
                    
                    if (runMode == JOG) 
                    {
                        PWMvalue = JogPWM;
                    }
                    else
                    {
                        PIDcontrol(i, PID);                                    
                        PWMvalue = PID[i].PWMvalue;            
                    }
                    if (i == 0) 
                    {
                        //setMotorOne(PWMvalue);  // $$$$                            
                        if (PWMvalue < 0)
                        {            
                            DIR1 = REVERSE;                              
                            PWMvalue = 0 - PWMvalue;                            
                        }
                        else DIR1 = FORWARD;                                    
                        PWM1 = PWMvalue;                            
                        
                    }       
                    PWM2 = PWM3 = PWM4 = 0;
                    
                    else if (i == 1)
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR2 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR2 = FORWARD;
                        PWM2 = 0; // PWMvalue;                            
                    }                                                
                    else if (i == 2) 
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR3 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR3 = FORWARD;                            
                        PWM3 = 0; // PWMvalue;
                    }
                    else
                    {
                        if (PWMvalue < 0)
                        {            
                            DIR4 = REVERSE;                    
                            PWMvalue = 0 - PWMvalue;
                        }
                        else DIR4 = FORWARD;                            
                        PWM4 = 0; // PWMvalue;                            
                    }
                    
                }
            }
            else
            {
                PID[0].sumError = 0;
                PWM1 = PWM2 = PWM3 = PWM4 = 0;
                LED = 0;
            }
        }    
        */
        
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false; 
            printf("\rReceived: %s", HOSTRxBuffer);
            q = 0;
            command = 0;
            goto CONTINUE1;
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (isalpha(ch) || ch == ' ') command = ch;                
                putchar(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( (isdigit(ch) || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof(NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'P':
                        if (q) PID[0].kP = PID[1].kP = PID[2].kP = PID[3].kP = floValue;
                        break;
                    case 'I':
                        if (q) PID[0].kI = PID[1].kI = PID[2].kI = PID[3].kI = floValue;
                        break;
                    case 'D':
                        if (q) PID[0].kD = PID[1].kD = PID[2].kD = PID[3].kD = floValue;
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = PID[1].PWMoffset = PID[2].PWMoffset = PID[3].PWMoffset = (long) floValue;
                        break;
                    case 'R':
                        runMode = REMOTE;
                        printf("REMOTE MODE ON");
                        break;
                    case 'L':
                        runMode = LOCAL;
                        printf("LOCAL ON");
                        break;       
                    case 'J':
                        JogPWM = (long) floValue;
                        printf("\rJOG ON: %d", JogPWM);
                        runMode = JOG;                        
                        break;                        
                    case 'H':
                        initHBridgeSPI();
                        ReadHBridgeData(HBridgeData);
                        printf("\r#%d: HBridge: %02X, %02X, %02X, %02X", SPIcounter++, HBridgeData[0], HBridgeData[1], HBridgeData[2], HBridgeData[3]);
                        SpiChnClose(SPI_CHANNEL);
                        break;
                    case ' ':
                        if (runMode) 
                        {                            
                            previousRunMode = runMode;
                            runMode = HALTED; 
                            printf("\rHALT");
                        }
                        else
                        {
                            runMode = previousRunMode;
                            if (runMode == LOCAL) printf("\rAD MODE");
                            else if (runMode == REMOTE) printf("\rREMOTE MODE");
                            else if (runMode == JOG) printf("\rJOG MODE");
                            else printf("\rMODE ERROR");
                        }
                        break;
                    case 'M':
                        if (displayFlag)
                        {
                            displayFlag = false;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = true;
                            printf("\rDisplay ON");
                        }   
                        break;
                    case 'Z':
                        ResetPID();
                        printf("\rPID reset = true");
                        break;
                    case 'T':
                        if (ADdisplay)
                        {
                            ADdisplay = false;
                            printf("\rPot display OFF");
                        }
                        else
                        {
                            ADdisplay = true;
                            printf("\rPot display ON");
                        }
                        break;
                    default:
                        printf("\rCommand: %c", command);
                        break;
                } // end switch command                
                printf("\rkP = %0.1f, kI = %0.3f, kD = %0.1f, Offset: %d\r", PID[0].kP, PID[0].kI, PID[0].kD, PID[0].PWMoffset);
                CONTINUE1: continue;
            } // End if command             
        } // End if HOSTRxBufferFull
        
        /*
        if (DATABufferFull)
        {
            DATABufferFull = false;
            success = processPacket(DATARxBuffer, &dataCommand, &dataSubCommand, arrServoPos, &numServos);
            result = success;
        }
        */
    } // End while(1))
} // End main())








/******************************************************************************
 *	Change Notice Interrupt Service Routine
 *
 *   Note: Switch debouncing is not performed.
 *   Code comes here if SW2 (CN16) PORTD.RD7 is pressed or released.
 *   The user must read the IOPORT to clear the IO pin change notice mismatch
 *	condition first, then clear the change notice interrupt flag.
 ******************************************************************************/

void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) 
{    
    // Step #1 - always clear the mismatch condition first
    // dummy = PORTReadBits(IOPORT_B, BIT_4 | BIT_2) & 0x0F;
    PortBRead = PORTB;// & 0x14;
    PortBflag = true;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();
}






unsigned short decodePacket(unsigned char *ptrInPacket, unsigned char *ptrData)
{
    unsigned short i, j;
    unsigned char escapeFlag = FALSE;
    unsigned char startFlag = false;
    unsigned char ch;

    j = 0;
    for (i = 0; i < MAXBUFFER; i++) 
    {
        ch = ptrInPacket[i];
        // Escape flag not active
        if (!escapeFlag) 
        {
            if (ch == STX) 
            {
                if (!startFlag) 
                {
                    startFlag = true;
                    j = 0;
                }
                else return (0);
            } 
            else if (ch == ETX) 
                return (j);
            else if (ch == DLE)
                escapeFlag = TRUE;
            else if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return (0);
        } 
        // Escape flag active
        else 
        {
            escapeFlag = FALSE;
            if (ch == ETX-1) ch = ETX;            
            if (j < MAXBUFFER)
                ptrData[j++] = ch;
            else return(0);
        }
    }
    return (j);
}




long PIDcontrol(long servoID, struct PIDtype *PID)
{
    short Error;     
    static short previousPosition = 0;
    short actualPosition;
    short commandPosition; 
    short pastError;
    long totalDerError = 0;
    long derError;
    static short errIndex = 0;
    float PCorr = 0, ICorr = 0, DCorr = 0;    
    static short displayCounter = 0;
    static unsigned short quadCurrent = QUAD_NONE;
    static unsigned short quadPrevious = QUAD_NONE;       
    unsigned char sensorError = false;
    short i;
       
    if (servoID != 0) return 0;  
    
    if (PID[servoID].ADActual < 341) quadCurrent = QUAD_ONE;
    else if (PID[servoID].ADActual < 682) quadCurrent = QUAD_TWO;
    else quadCurrent = QUAD_THREE;
    
    if (quadCurrent==QUAD_THREE && (quadPrevious==QUAD_ONE || previousPosition < 0))
        actualPosition = PID[servoID].ADActual - 1024;
    else if (quadCurrent==QUAD_ONE && (quadPrevious==QUAD_THREE || previousPosition > 1023))
        actualPosition = PID[servoID].ADActual + 1024;
    else actualPosition = PID[servoID].ADActual;
    
    quadPrevious = quadCurrent;
    previousPosition = actualPosition; 
    
    actualPosition = PID[servoID].ADActual;
    commandPosition = PID[servoID].ADCommand;
    Error = actualPosition - commandPosition;    
    PID[servoID].error[errIndex] = Error;
    errIndex++; 
    if (errIndex >= FILTERSIZE) errIndex = 0;                                  
         
    if (!PID[servoID].saturation) PID[servoID].sumError = PID[servoID].sumError + (long)Error; 
        
    totalDerError = 0;
    for (i = 0; i < FILTERSIZE; i++)
        totalDerError = totalDerError + PID[servoID].error[i];
    derError = totalDerError / FILTERSIZE;
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;        
    
    PCorr = ((float)Error) * -PID[servoID].kP;    
    ICorr = ((float)PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float)derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection == 0) PID[servoID].PWMvalue = 0;
    else if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
           
    if (abs(Error) < 4) 
    {
        PID[servoID].PWMvalue = 0;
        if (PID[servoID].ErrorCounter > 0) PID[servoID].ErrorCounter--;
    }
    else PID[servoID].ErrorCounter = 100;
    if (PID[servoID].ErrorCounter == 0) PID[servoID].sumError = 0;
        
    if (PID[servoID].PWMvalue > PWM_MAX) 
    {
        PID[servoID].PWMvalue = PWM_MAX;
        PID[servoID].saturation = true;
    }
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
    {
        PID[servoID].PWMvalue = -PWM_MAX;
        PID[servoID].saturation = true;
    }
    else PID[servoID].saturation = false;
    
    if (actualPosition > 1024 && PID[servoID].PWMvalue > 0) PID[servoID].PWMvalue = 0;
    if (actualPosition < 0 && PID[servoID].PWMvalue < 0) PID[servoID].PWMvalue = 0;
    
    if (PID[servoID].ADActual < 10) 
    {
        PID[servoID].PWMvalue = 0;
        PID[servoID].sumError = 0;        
        sensorError = true;
    }    
    
    if (servoID == 0 && displayFlag) 
    {
        displayCounter++;
        if (displayCounter >= 20)
        {
            displayCounter = 0;
            if (sensorError) printf("\rSENSOR ERROR - ACTUAL: %d", PID[servoID].ADActual);
            else printf("\rCOM: %d ACT: %d ERR: %d SUM: %d P: %0.1f D: %0.1f I: %0.1f PWM: %d ", commandPosition, actualPosition, Error, PID[servoID].sumError, PCorr, DCorr, ICorr, PID[servoID].PWMvalue);
        }
    }
    PID[servoID].reset = false;   
    
    return 1;
}

    /*
    EEPROM_WP = 0;
    initI2C(I2C1);   
    EepromWriteBlock(I2C1, EEPROM_ID, EEPROMmemoryAddress, strEEmesssageOut, EEMessageLength);
    DelayMs(10);
    EEPROM_WP = 1;        
    EepromReadBlock(I2C1, EEPROM_ID, EEPROMmemoryAddress, strEEmesssageIn, EEMessageLength); 
    strEEmesssageIn[EEMessageLength] = '\0';
    printf("\rEEprom Message In: %s", strEEmesssageIn);   
    */

void ClearCopyBuffer()
{
    int i;
    for (i = 0; i < MAXBUFFER; i++)
    {
        RS485RxBufferCopy[i] = '\0';
        // RS485RxBuffer[i] = '\0';
    }
}


unsigned char processPacketData(short packetDataLength, unsigned char *ptrPacketData, short *numServos, short *ptrServoPositions, unsigned char *command, unsigned char *subCommand)
{
    MConvertType servoValue;    
    short j, i = 0;  
    
    if (!CheckCRC(ptrPacketData, packetDataLength)) return false;    
    *command = ptrPacketData[i++];
    *subCommand = ptrPacketData[i++];
    *numServos = ptrPacketData[i++];
    
    
    if (*numServos > MAXSERVOS) return false;
    j = 0;
    while(j < *numServos)
    {
        servoValue.b[0] = ptrPacketData[i++];
        servoValue.b[1] = ptrPacketData[i++];
        ptrServoPositions[j++] = servoValue.integer;
    }
    return true;
}

void ResetPID()
{
    int i, j;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0; // For 53:1 ratio Servo City motor
        PID[i].kP = KP;
        PID[i].kI = KI;
        PID[i].kD = KD;
        PID[i].PWMoffset = PWM_OFFSET;
        PID[i].PWMvalue = 0;
        PID[i].ADActual = 0;
        PID[i].ADCommand = 0;
        PID[i].reset = true;
        PID[i].saturation = false;
        PID[i].ErrorCounter = 0;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
}

void PrintServoData(short numServos, short *ptrServoPositions, unsigned char command, unsigned char subCommand)
{
    int i;
    printf("\rOK! Com: %d, Sub: %d, servos %d: ", command, subCommand, numServos);
    for (i = 0; i < 10; i++) printf("%d, ", ptrServoPositions[i]);
}




void setMotorOne(short TargetDutyCycle) 
{
    short dutyCycle;    

    // Set PWMs to 0:
    if (TargetDutyCycle == 0) PWM1 = PWM3 = 0;

    // Negative PWM is counterclockwise.     
    else if (TargetDutyCycle < 0) 
    {
        dutyCycle = 0 - TargetDutyCycle;
        if (dutyCycle > PWM_MAX) dutyCycle = PWM_MAX;
        PWM1 = dutyCycle;
        PWM3 = 0;
    }
    else
    {
        dutyCycle = TargetDutyCycle;
        if (dutyCycle > PWM_MAX) dutyCycle = PWM_MAX;
        PWM3 = dutyCycle;
        PWM1 = 0;        
    }
}


// RS485 UART interrupt handler it is set at priority level 2
void __ISR(RS485_VECTOR, ipl2) IntRS485UartHandler(void) 
{
    unsigned char ch;
    static unsigned short RS485RxIndex = 0;

    if (RS485bits.OERR || RS485bits.FERR) {
        if (UARTReceivedDataIsAvailable(RS485uart))
            ch = UARTGetDataByte(RS485uart);
        RS485bits.OERR = 0;
        RS485RxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(RS485uart))) {
        INTClearFlag(INT_SOURCE_UART_RX(RS485uart));
        if (UARTReceivedDataIsAvailable(RS485uart)) {
            ch = UARTGetDataByte(RS485uart);
            {
                if (ch == LF || ch == 0);
                else if (ch == CR) 
                {
                    if (RS485RxIndex < (MAXBUFFER-1)) 
                    {
                        RS485RxBuffer[RS485RxIndex] = CR;
                        RS485RxBuffer[RS485RxIndex + 1] = '\0'; 
                        RS485RxBufferFull = true;
                    }
                    RS485RxIndex = 0;
                }                
                else 
                {
                    if (RS485RxIndex < (MAXBUFFER-1))
                        RS485RxBuffer[RS485RxIndex++] = ch;                    
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(RS485uart))) {
        INTClearFlag(INT_SOURCE_UART_TX(RS485uart));
    }
}


#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char ch, inByte;
static unsigned long i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            inByte = UARTGetDataByte(HOSTuart);
            ch = toupper(inByte);
            if (ch != 0 && ch != '\n') {            
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
                if ('\r' == ch) 
                {
                    HOSTRxBufferFull = true;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}

void InitializeSystem(void) 
{
	int i; 
    
    SYSTEMConfigPerformance(80000000);
    
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    ConfigAd();       
   
    
    // I/O Ports:
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);  // DIR4 
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_2);  // SW1-3
    PORTSetPinsDigitalOut(IOPORT_B, BIT_5);  // RS485_ENABLE
    PORTSetPinsDigitalIn(IOPORT_C, BIT_13);  // SW4
    PORTSetPinsDigitalOut(IOPORT_C, BIT_1);  // TEST_OUT
    PORTSetPinsDigitalIn(IOPORT_D, BIT_7 | BIT_10 | BIT_11 | BIT_13);  // SW5, MOTOR DIR #5, #3, #2
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8 | BIT_9);  // EE_WR, LED
    PORTSetPinsDigitalIn(IOPORT_E, BIT_9 | BIT_5 | BIT_7 | BIT_8);  // ENCODER DIRECTION INPUTS 1-4
    PORTSetPinsDigitalOut(IOPORT_G, BIT_12);  // MOTOR DIR #1
    
    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip     

    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip     
    
    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;    
    PR2 = 4000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip   
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);                    

    // Set up PWM OC1
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;
    
    // Set up PWM OC5
    OC5CON = 0x00;
    OC5CONbits.OC32 = 0; // 16 bit PWM
    OC5CONbits.ON = 1; // Turn on PWM
    OC5CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC5CONbits.OCM2 = 1; // PWM runMode enabled, no fault pin
    OC5CONbits.OCM1 = 1;
    OC5CONbits.OCM0 = 0;
    OC5RS = 0;

   #define SYS_FREQ 80000000      
    
    // Set up HOST UART    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure HOST UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(HOSTuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up RS485 UART    
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(RS485uart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 57600);
    // ActualRS485BaudRate = UARTSetDataRate(RS485uart, SYS_FREQ, 2000000);
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_ENABLED); 
    INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(RS485uart), INT_SUB_PRIORITY_LEVEL_0);  
    
 
    // Set up XBEE UART    
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualXBEEBaudRate = UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    // ActualXBEEBaudRate = UARTSetDataRate(XBEEuart, SYS_FREQ, 2000000);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE UART Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_ENABLED); 
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);  
    
    
    
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
   
}//end UserInit

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{        
    static int intCounter = 0; 
    static int LEDcounter = 2000;
    static int memoryCounter = 625; 
    
    mT2ClearIntFlag(); // clear the interrupt flag    
    
    LEDcounter++;
    if (LEDcounter >= 2000)
    {
        LEDcounter = 0;
        if (LED) LED = 0;
        else LED = 1;
    }
    
    intCounter++;
    if (intCounter >= 50) // 50
    {
        intCounter = 0;
        intFlag = true;
    }
    
    if (memoryCounter) memoryCounter--;
    if (!memoryCounter)
    {
        memoryCounter=625;
        memoryFlag = true;
        if (timeout)
        {
            timeout--;
            // if (timeout == 0) LED = 0;
        }
    }
}

// RS485 UART interrupt handler it is set at priority level 2
void __ISR(XBEE_VECTOR, ipl2) IntXBEEUartHandler(void) 
{
    unsigned char ch;
    static unsigned short XBEERxIndex = 0;

    if (XBEEbits.OERR || XBEEbits.FERR) {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
        XBEERxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));
        if (UARTReceivedDataIsAvailable(XBEEuart)) {
            ch = UARTGetDataByte(XBEEuart);
            {
                if (ch == LF || ch == 0);
                else if (ch == CR) 
                {
                    if (XBEERxIndex < (MAXBUFFER-1)) 
                    {
                        XBEERxBuffer[XBEERxIndex] = CR;
                        XBEERxBuffer[XBEERxIndex + 1] = '\0'; 
                        XBEERxBufferFull = true;
                    }
                    XBEERxIndex = 0;
                }                
                else 
                {
                    if (XBEERxIndex < (MAXBUFFER-1))
                        XBEERxBuffer[XBEERxIndex++] = ch;                    
                }
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
    }
}

void __ISR(_ADC_VECTOR, ipl2) ADHandler(void) 
{
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
    ADready = true;
}


void ConfigAd(void) 
{
    // ---- configure and enable the ADC ----    
    // mPORTBSetPinsAnalogIn(BIT_3 | BIT_4 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$    
    mPORTBSetPinsAnalogIn(BIT_3 | BIT_4 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 | BIT_13 | BIT_14 | BIT_15); // $$$$    

    // ensure the ADC is off before setting the configuration
    CloseADC10();

// define setup parameters for OpenADC10
// Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

// ADC ref external | disable offset test | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_10 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

// Use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_10 | ADC_CONV_CLK_32Tcy

// #define PARAM4 ENABLE_AN3_ANA | ENABLE_AN4_ANA | ENABLE_AN8_ANA | ENABLE_AN9_ANA | ENABLE_AN10_ANA | ENABLE_AN11_ANA | ENABLE_AN12_ANA | ENABLE_AN13_ANA | ENABLE_AN14_ANA | ENABLE_AN15_ANA        
#define PARAM4 ENABLE_AN3_ANA | ENABLE_AN4_ANA | ENABLE_AN8_ANA | ENABLE_AN9_ANA | ENABLE_AN10_ANA | ENABLE_AN11_ANA | ENABLE_AN12_ANA | ENABLE_AN13_ANA | ENABLE_AN14_ANA | ENABLE_AN15_ANA        
// #define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7
    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}
