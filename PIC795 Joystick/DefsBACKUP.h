/* 
 * File:   Defs.h
 * Author: Jim
 *
 * Created on December 15, 2014, 12:50 PM
 * 8-5-18 JBS: Updated to work for SNAD PIC or UBW32 boards.
 * 8-5-19 Replaced SPICON1 with SPICON
 */

#ifndef DEFS_H
#define	DEFS_H

#define USE_UBW32
#define SPI_CHANNEL 2

#define FALSE 0
#define TRUE !FALSE
#define false FALSE
#define true TRUE

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

typedef union
{
	unsigned char b[2];
	unsigned short integer;
} MConvertType;

#define MAX_SERVOS 64

/*
#define NUMPOTS 4

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR
#define HOST_VECTOR _UART_2_VECTOR


#define RS485uart UART5

#define XBEE_SLEEP PORTBbits.RB15
#define TEST_OUT PORTBbits.RB0
#define TRIG_OUT PORTBbits.RB1

#define TRIG_HI PORTSetBits(IOPORT_B, BIT_1)
#define TRIG_LOW PORTClearBits(IOPORT_B, BIT_1)

#define XBEE_SLEEP_ON PORTSetBits(IOPORT_B, BIT_0)
#define XBEE_SLEEP_OFF PORTClearBits(IOPORT_B, BIT_0)

#define TEST_HI PORTSetBits(IOPORT_B, BIT_0)
#define TEST_LOW PORTClearBits(IOPORT_B, BIT_0)


#define START 1

#define MAXBUFFER 256 // 512
#define MAXHOSTBUFFER 128

#define XBEE_SLEEP PORTBbits.RB15

#define MAXRANDOM (RAND_MAX+1)
#define true	TRUE
#define false 	FALSE

#define RS485_CTRL PORTGbits.RG0
*/

/*
#ifdef USE_UBW32
    // Description: SD-SPI Chip Select and TRIS bits
    #define SD_CS               LATEbits.LATE4 // $$$$
    #define SD_CS_TRIS          TRISEbits.TRISE4 
    // Description: SD-SPI Card Detect and TRIS bits
    #define SD_CD               PORTGbits.RG9 
    #define SD_CD_TRIS          TRISGbits.TRISG9 
    // Description: SD-SPI Write Protect input and TRIS bits
    #define SD_WE_TRIS          TRISAbits.TRISA0 
    #define SD_WE               PORTAbits.RA0
#else // For SNAD PIC BOARD!!!:
    // Description: SD-SPI Chip Select and TRIS bits
    #define SD_CS               LATGbits.LATG9
    #define SD_CS_TRIS          TRISGbits.TRISG9 
    // Description: SD-SPI Card Detect and TRIS bits
    #define SD_CD               PORTEbits.RE8 
    #define SD_CD_TRIS          TRISEbits.TRISE8
    // Description: SD-SPI Write Protect - doesn't exist on SNAD PIC, 0 = NO write protect
    #define SD_WE 0
#endif
*/

/*
        // Registers for the SPI module you want to use
        //#define MDD_USE_SPI_1  $$$$
        #define MDD_USE_SPI_2
        #define USE_SD_INTERFACE_WITH_SPI


		//SPI Configuration
		#define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
        #define SPI_START_CFG_2     (SPI_ENABLE)

        // Define the SPI frequency
        #define SPI_FREQUENCY			(20000000)


            // Description: The main SPI control register
            #define SPICON             SPI2CON
            // Description: The SPI status register
            #define SPISTAT             SPI2STAT
            // Description: The SPI Buffer
            #define SPIBUF              SPI2BUF
            // Description: The receive buffer full bit in the SPI status register
            #define SPISTAT_RBF         SPI2STATbits.SPIRBF
            // Description: The bitwise define for the SPI control register (i.e. _____bits)
            #define SPICONbits         SPI2CONbits
            // Description: The bitwise define for the SPI status register (i.e. _____bits)
            #define SPISTATbits         SPI2STATbits
            // Description: The enable bit for the SPI module
            #define SPIENABLE           SPI2CONbits.ON
            // Description: The definition for the SPI baud rate generator register (PIC32)
            #define SPIBRG			    SPI2BRG

            // Tris pins for SCK/SDI/SDO lines

            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISGbits.TRISG6
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISGbits.TRISG7
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISGbits.TRISG8
            //SPI library functions
            #define putcSPI             putcSPI2
            #define getcSPI             getcSPI2
            #define OpenSPI(config1, config2)   OpenSPI2(config1, config2)
*/
  
  
/*
        #define USE_SD_INTERFACE_WITH_SPI
        #define MDD_USE_SPI_2

		//SPI Configuration
		#define SPI_START_CFG_1     (PRI_PRESCAL_64_1 | SEC_PRESCAL_8_1 | MASTER_ENABLE_ON | SPI_CKE_ON | SPI_SMP_ON)
        #define SPI_START_CFG_2     (SPI_ENABLE)

        // Define the SPI frequency
        #define SPI_FREQUENCY			(20000000)

            // Description: SD-SPI Chip Select Output bit
            #define SD_CS               LATCbits.LATC4 // LATBbits.LATB9
            // Description: SD-SPI Chip Select TRIS bit
            #define SD_CS_TRIS          TRISCbits.TRISC4 // TRISBbits.TRISB9

            // Description: SD-SPI Card Detect Input bit
            #define SD_CD               PORTGbits.RG9 // PORTGbits.RG0
            // Description: SD-SPI Card Detect TRIS bit
            #define SD_CD_TRIS          TRISGbits.TRISG9 // TRISGbits.TRISG0

            // Description: SD-SPI Write Protect Check Input bit
            #define SD_WE               PORTAbits.RA0 // PORTGbits.RG1
            // Description: SD-SPI Write Protect Check TRIS bit
            #define SD_WE_TRIS          TRISAbits.TRISA0 // TRISGbits.TRISG1

            // Description: The main SPI control register
            #define SPICON             SPI2CON
            // Description: The SPI status register
            #define SPISTAT             SPI2STAT
            // Description: The SPI Buffer
            #define SPIBUF              SPI2BUF
            // Description: The receive buffer full bit in the SPI status register
            #define SPISTAT_RBF         SPI2STATbits.SPIRBF
            // Description: The bitwise define for the SPI control register (i.e. _____bits)
            #define SPICONbits         SPI2CONbits
            // Description: The bitwise define for the SPI status register (i.e. _____bits)
            #define SPISTATbits         SPI2STATbits
            // Description: The enable bit for the SPI module
            #define SPIENABLE           SPI2CONbits.ON
            // Description: The definition for the SPI baud rate generator register (PIC32)
            #define SPIBRG			    SPI2BRG

            // Tris pins for SCK/SDI/SDO lines

            // Description: The TRIS bit for the SCK pin
            #define SPICLOCK            TRISGbits.TRISG6
            // Description: The TRIS bit for the SDI pin
            #define SPIIN               TRISGbits.TRISG7
            // Description: The TRIS bit for the SDO pin
            #define SPIOUT              TRISGbits.TRISG8
            //SPI library functions
            #define putcSPI             putcSPI2
            #define getcSPI             getcSPI2
            #define OpenSPI(config1, config2)   OpenSPI2(config1, config2)
        
*/


#endif	/* DEFS_H */

