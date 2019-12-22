/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 SPI - 3-wire Master Incremented Data
 *
 * This example shows how SPI master talks to SPI slave using 3-wire mode.
 * Incrementing data is sent by the master starting at 0x01. Received data is
 * expected to be same as the previous transmission.  eUSCI RX ISR is used to
 * handle communication with the CPU, normally in LPM0. Because all execution 
 * after LPM0 is in ISRs, initialization waits for DCO to stabilize against 
 * ACLK.
 *
 * Note that in this example, EUSCIB0 is used for the SPI port. If the user
 * wants to use EUSCIA for SPI operation, they are able to with the same APIs
 * with the EUSCI_AX parameters.
 *
 * ACLK = ~32.768kHz, MCLK = SMCLK = DCO 3MHz
 *
 * Use with SPI Slave Data Echo code example.
 *
 *                MSP432P4111
 *              -----------------
 *             |                 |
 *             |                 |
 *             |                 |
 *             |             P1.6|-> Data Out (UCB0SIMO)
 *             |                 |
 *             |             P1.7|<- Data In (UCB0SOMI)
 *             |                 |
 *             |             P1.5|-> Serial Clock Out (UCB0CLK)
 *******************************************************************************/
//******************************************************************************

//

//      MCU setup:

//      MCLK: 48MHz, SMCLK: 3MHz, ACLK: 32768Hz

//      Flash:  1 waitstate

//      SPI clock 3MHz

//

//      GPIO usage:

//      - P1.5 O  SPI clock

//      - P1.6 O  SPI SIMO

//      - P1.7 I  SPI SOMI

//      - P4.1 O  Camera chip select

//

//

//      Used peripherals:

//      - USCIB0 - SPI master for camera

//

//      Used MSP432P4 driver library

//

//   Created with CCS Version: 9.2.0.00013

//

//******************************************************************************



//---------------------------

//  Includes

//---------------------------

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/inc/msp432p4111.h>


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>




//---------------------------

//  Defines

//---------------------------



// Defines to abstract GPIOs for camera

#define CAM_CS_LO    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1)    // CAM_CS_LO would set pin low
#define CAM_CS_HI    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1)   // CAM_CS_HI would set pin high





//---------------------------

//  Globals

/* Definitions for LCD Driver */
#define char1 16    // Digit A1 - L16
#define char2 32    // Digit A2 - L32
#define char3 40    // Digit A3 - L40
#define char4 36    // Digit A4 - L36
#define char5 28    // Digit A5 - L28
#define char6 44    // Digit A6 - L44

/* LCD memory map for numeric digits (Byte Access) */
const char digit[10][4] =
{
    {0xC, 0xF, 0x8, 0x2},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x0, 0x6, 0x0, 0x2},  /* "1" */
    {0xB, 0xD, 0x0, 0x0},  /* "2" */
    {0x3, 0xF, 0x0, 0x0},  /* "3" */
    {0x7, 0x6, 0x0, 0x0},  /* "4" */
    {0x7, 0xB, 0x0, 0x0},  /* "5" */
    {0xF, 0xB, 0x0, 0x0},  /* "6" */
    {0x4, 0xE, 0x0, 0x0},  /* "7" */
    {0xF, 0xF, 0x0, 0x0},  /* "8" */
    {0x7, 0xF, 0x0, 0x0}   /* "9" */
};

/* LCD memory map for uppercase letters (Byte Access) */
const char alphabetBig[26][4] =
{
    {0xF, 0xE, 0x0, 0x0},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0x1, 0xF, 0x0, 0x5},  /* "B" */
    {0xC, 0x9, 0x0, 0x0},  /* "C" */
    {0x0, 0xF, 0x0, 0x5},  /* "D" */
    {0xF, 0x9, 0x0, 0x0},  /* "E" */
    {0xF, 0x8, 0x0, 0x0},  /* "F" */
    {0xD, 0xB, 0x0, 0x0},  /* "G" */
    {0xF, 0x6, 0x0, 0x0},  /* "H" */
    {0x0, 0x9, 0x0, 0x5},  /* "I" */
    {0x8, 0x7, 0x0, 0x0},  /* "J" */
    {0xE, 0x0, 0x2, 0x2},  /* "K" */
    {0xC, 0x1, 0x0, 0x0},  /* "L" */
    {0xC, 0x6, 0x0, 0xA},  /* "M" */
    {0xC, 0x6, 0x2, 0x8},  /* "N" */
    {0xC, 0xF, 0x0, 0x0},  /* "O" */
    {0xF, 0xC, 0x0, 0x0},  /* "P" */
    {0xC, 0xF, 0x2, 0x0},  /* "Q" */
    {0xF, 0xC, 0x2, 0x0},  /* "R" */
    {0x7, 0xB, 0x0, 0x0},  /* "S" */
    {0x0, 0x8, 0x0, 0x5},  /* "T" */
    {0xC, 0x7, 0x0, 0x0},  /* "U" */
    {0xC, 0x0, 0x8, 0x2},  /* "V" */
    {0xC, 0x6, 0xA, 0x0},  /* "W" */
    {0x0, 0x0, 0xA, 0xA},  /* "X" */
    {0x0, 0x0, 0x0, 0xB},  /* "Y" */
    {0x0, 0x9, 0x8, 0x2}   /* "Z" */
};

static void showChar(char c, int position);

/* Configuration Structure for LCD */
LCD_F_Config lcdConf =
{
    .clockSource = LCD_F_CLOCKSOURCE_ACLK,
    .clockDivider = LCD_F_CLOCKDIVIDER_8,
    .clockPrescaler = LCD_F_CLOCKPRESCALER_1,
    .muxRate = LCD_F_4_MUX,
    .waveforms = LCD_F_STANDARD_WAVEFORMS,
    .segments = LCD_F_SEGMENTS_ENABLED
};


static volatile uint8_t RXData = 0;
static uint8_t TXData = 0;

static const uint8_t black_white[6][2] = {

                                     {0xff, 0x00},
                                     {0x7c, 0x00},
                                     {0x7d, 0x18},
                                     {0x7c, 0x05},
                                     {0x7d, 0x80},
                                     {0x7d, 0x80}



};

static const uint8_t JPEG_INIT[191][2] = {

                                        { 0xff, 0x00 },
                                          { 0x2c, 0xff },
                                          { 0x2e, 0xdf },
                                          { 0xff, 0x01 },
                                          { 0x3c, 0x32 },
                                          { 0x11, 0x00 },
                                          { 0x09, 0x02 },
                                          { 0x04, 0x28 },
                                          { 0x13, 0xe5 },
                                          { 0x14, 0x48 },
                                          { 0x2c, 0x0c },
                                          { 0x33, 0x78 },
                                          { 0x3a, 0x33 },
                                          { 0x3b, 0xfB },
                                          { 0x3e, 0x00 },
                                          { 0x43, 0x11 },
                                          { 0x16, 0x10 },
                                          { 0x39, 0x92 },
                                          { 0x35, 0xda },
                                          { 0x22, 0x1a },
                                          { 0x37, 0xc3 },
                                          { 0x23, 0x00 },
                                          { 0x34, 0xc0 },
                                          { 0x36, 0x1a },
                                          { 0x06, 0x88 },
                                          { 0x07, 0xc0 },
                                          { 0x0d, 0x87 },
                                          { 0x0e, 0x41 },
                                          { 0x4c, 0x00 },
                                          { 0x48, 0x00 },
                                          { 0x5B, 0x00 },
                                          { 0x42, 0x03 },
                                          { 0x4a, 0x81 },
                                          { 0x21, 0x99 },
                                          { 0x24, 0x40 },
                                          { 0x25, 0x38 },
                                          { 0x26, 0x82 },
                                          { 0x5c, 0x00 },
                                          { 0x63, 0x00 },
                                          { 0x61, 0x70 },
                                          { 0x62, 0x80 },
                                          { 0x7c, 0x05 },
                                          { 0x20, 0x80 },
                                          { 0x28, 0x30 },
                                          { 0x6c, 0x00 },
                                          { 0x6d, 0x80 },
                                          { 0x6e, 0x00 },
                                          { 0x70, 0x02 },
                                          { 0x71, 0x94 },
                                          { 0x73, 0xc1 },
                                          { 0x12, 0x40 },
                                          { 0x17, 0x11 },
                                          { 0x18, 0x43 },
                                          { 0x19, 0x00 },
                                          { 0x1a, 0x4b },
                                          { 0x32, 0x09 },
                                          { 0x37, 0xc0 },
                                          { 0x4f, 0x60 },
                                          { 0x50, 0xa8 },
                                          { 0x6d, 0x00 },
                                          { 0x3d, 0x38 },
                                          { 0x46, 0x3f },
                                          { 0x4f, 0x60 },
                                          { 0x0c, 0x3c },
                                          { 0xff, 0x00 },
                                          { 0xe5, 0x7f },
                                          { 0xf9, 0xc0 },
                                          { 0x41, 0x24 },
                                          { 0xe0, 0x14 },
                                          { 0x76, 0xff },
                                          { 0x33, 0xa0 },
                                          { 0x42, 0x20 },
                                          { 0x43, 0x18 },
                                          { 0x4c, 0x00 },
                                          { 0x87, 0xd5 },
                                          { 0x88, 0x3f },
                                          { 0xd7, 0x03 },
                                          { 0xd9, 0x10 },
                                          { 0xd3, 0x82 },
                                          { 0xc8, 0x08 },
                                          { 0xc9, 0x80 },
                                          { 0x7c, 0x00 },
                                          { 0x7d, 0x00 },
                                          { 0x7c, 0x03 },
                                          { 0x7d, 0x48 },
                                          { 0x7d, 0x48 },
                                          { 0x7c, 0x08 },
                                          { 0x7d, 0x20 },
                                          { 0x7d, 0x10 },
                                          { 0x7d, 0x0e },
                                          { 0x90, 0x00 },
                                          { 0x91, 0x0e },
                                          { 0x91, 0x1a },
                                          { 0x91, 0x31 },
                                          { 0x91, 0x5a },
                                          { 0x91, 0x69 },
                                          { 0x91, 0x75 },
                                          { 0x91, 0x7e },
                                          { 0x91, 0x88 },
                                          { 0x91, 0x8f },
                                          { 0x91, 0x96 },
                                          { 0x91, 0xa3 },
                                          { 0x91, 0xaf },
                                          { 0x91, 0xc4 },
                                          { 0x91, 0xd7 },
                                          { 0x91, 0xe8 },
                                          { 0x91, 0x20 },
                                          { 0x92, 0x00 },
                                          { 0x93, 0x06 },
                                          { 0x93, 0xe3 },
                                          { 0x93, 0x05 },
                                          { 0x93, 0x05 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x04 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x93, 0x00 },
                                          { 0x96, 0x00 },
                                          { 0x97, 0x08 },
                                          { 0x97, 0x19 },
                                          { 0x97, 0x02 },
                                          { 0x97, 0x0c },
                                          { 0x97, 0x24 },
                                          { 0x97, 0x30 },
                                          { 0x97, 0x28 },
                                          { 0x97, 0x26 },
                                          { 0x97, 0x02 },
                                          { 0x97, 0x98 },
                                          { 0x97, 0x80 },
                                          { 0x97, 0x00 },
                                          { 0x97, 0x00 },
                                          { 0xc3, 0xed },
                                          { 0xa4, 0x00 },
                                          { 0xa8, 0x00 },
                                          { 0xc5, 0x11 },
                                          { 0xc6, 0x51 },
                                          { 0xbf, 0x80 },
                                          { 0xc7, 0x10 },
                                          { 0xb6, 0x66 },
                                          { 0xb8, 0xA5 },
                                          { 0xb7, 0x64 },
                                          { 0xb9, 0x7C },
                                          { 0xb3, 0xaf },
                                          { 0xb4, 0x97 },
                                          { 0xb5, 0xFF },
                                          { 0xb0, 0xC5 },
                                          { 0xb1, 0x94 },
                                          { 0xb2, 0x0f },
                                          { 0xc4, 0x5c },
                                          { 0xc0, 0x64 },
                                          { 0xc1, 0x4B },
                                          { 0x8c, 0x00 },
                                          { 0x86, 0x3D },
                                          { 0x50, 0x00 },
                                          { 0x51, 0xC8 },
                                          { 0x52, 0x96 },
                                          { 0x53, 0x00 },
                                          { 0x54, 0x00 },
                                          { 0x55, 0x00 },
                                          { 0x5a, 0xC8 },
                                          { 0x5b, 0x96 },
                                          { 0x5c, 0x00 },
                                          { 0xd3, 0x00 },   //{ 0xd3, 0x7f },
                                          { 0xc3, 0xed },
                                          { 0x7f, 0x00 },
                                          { 0xda, 0x00 },
                                          { 0xe5, 0x1f },
                                          { 0xe1, 0x67 },
                                          { 0xe0, 0x00 },
                                          { 0xdd, 0x7f },
                                          { 0x05, 0x00 },

                                          { 0x12, 0x40 },
                                          { 0xd3, 0x04 },   //{ 0xd3, 0x7f },
                                          { 0xc0, 0x16 },
                                          { 0xC1, 0x12 },
                                          { 0x8c, 0x00 },
                                          { 0x86, 0x3d },
                                          { 0x50, 0x00 },
                                          { 0x51, 0x2C },
                                          { 0x52, 0x24 },
                                          { 0x53, 0x00 },
                                          { 0x54, 0x00 },
                                          { 0x55, 0x00 },
                                          { 0x5A, 0x2c },
                                          { 0x5b, 0x24 },
                                          { 0x5c, 0x00 },
                                          { 0xff, 0xff },

};

static const uint8_t YUV_422[10][2] = {

                                       { 0xFF, 0x00 },
                                         { 0x05, 0x00 },
                                         { 0xDA, 0x10 },
                                         { 0xD7, 0x03 },
                                         { 0xDF, 0x00 },
                                         { 0x33, 0x80 },
                                         { 0x3C, 0x40 },
                                         { 0xe1, 0x77 },
                                         { 0x00, 0x00 },
                                         { 0xff, 0xff },
};

static const uint8_t JPEG[9][2] = {


                                   { 0xe0, 0x14 },
                                     { 0xe1, 0x77 },
                                     { 0xe5, 0x1f },
                                     { 0xd7, 0x03 },
                                     { 0xda, 0x10 },
                                     { 0xe0, 0x00 },
                                     { 0xFF, 0x01 },
                                     { 0x04, 0x08 },
                                     { 0xff, 0xff },


};

static const uint8_t JPEG_160x120[40][2] = {

                                          { 0xff, 0x01 },
                                            { 0x12, 0x40 },
                                            { 0x17, 0x11 },
                                            { 0x18, 0x43 },
                                            { 0x19, 0x00 },
                                            { 0x1a, 0x4b },
                                            { 0x32, 0x09 },
                                            { 0x4f, 0xca },
                                            { 0x50, 0xa8 },
                                            { 0x5a, 0x23 },
                                            { 0x6d, 0x00 },
                                            { 0x39, 0x12 },
                                            { 0x35, 0xda },
                                            { 0x22, 0x1a },
                                            { 0x37, 0xc3 },
                                            { 0x23, 0x00 },
                                            { 0x34, 0xc0 },
                                            { 0x36, 0x1a },
                                            { 0x06, 0x88 },
                                            { 0x07, 0xc0 },
                                            { 0x0d, 0x87 },
                                            { 0x0e, 0x41 },
                                            { 0x4c, 0x00 },
                                            { 0xff, 0x00 },
                                            { 0xe0, 0x04 },
                                            { 0xc0, 0x64 },
                                            { 0xc1, 0x4b },
                                            { 0x86, 0x35 },
                                            { 0x50, 0x92 },
                                            { 0x51, 0xc8 },
                                            { 0x52, 0x96 },
                                            { 0x53, 0x00 },
                                            { 0x54, 0x00 },
                                            { 0x55, 0x00 },
                                            { 0x57, 0x00 },
                                            { 0x5a, 0x28 },
                                            { 0x5b, 0x1e },
                                            { 0x5c, 0x00 },
                                            { 0xe0, 0x00 },
                                            { 0xff, 0xff },

};

static const uint8_t OV2640_176x144_JPEG[40][2] =
{
  { 0xff, 0x01 },
  { 0x12, 0x40 },
  { 0x17, 0x11 },
  { 0x18, 0x43 },
  { 0x19, 0x00 },
  { 0x1a, 0x4b },
  { 0x32, 0x09 },
  { 0x4f, 0xca },
  { 0x50, 0xa8 },
  { 0x5a, 0x23 },
  { 0x6d, 0x00 },
  { 0x39, 0x12 },
  { 0x35, 0xda },
  { 0x22, 0x1a },
  { 0x37, 0xc3 },
  { 0x23, 0x00 },
  { 0x34, 0xc0 },
  { 0x36, 0x1a },
  { 0x06, 0x88 },
  { 0x07, 0xc0 },
  { 0x0d, 0x87 },
  { 0x0e, 0x41 },
  { 0x4c, 0x00 },
  { 0xff, 0x00 },
  { 0xe0, 0x04 },
  { 0xc0, 0x64 },
  { 0xc1, 0x4b },
  { 0x86, 0x35 },
  { 0x50, 0x92 },
  { 0x51, 0xc8 },
  { 0x52, 0x96 },
  { 0x53, 0x00 },
  { 0x54, 0x00 },
  { 0x55, 0x00 },
  { 0x57, 0x00 },
  { 0x5a, 0x2c },
  { 0x5b, 0x24 },
  { 0x5c, 0x00 },
  { 0xe0, 0x00 },
  { 0xff, 0xff },
};


//---------------------------



// SPI0 Configuration Parameter Structure // Here you select clock source, input frequency, SPI clock frequency and the different settings // Check driverlib API documentation which you can find in Resource explorer or online
    const eUSCI_SPI_MasterConfig spiMasterConfig = {

    EUSCI_B_SPI_CLOCKSOURCE_SMCLK, // Clock source

    8000000, // Frequency of clock source

    8000000, // Desired SPI clock frequency

    EUSCI_B_SPI_MSB_FIRST, // MSB or LSB first

    EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT, // Data/clock relationship see TRM page 940

    EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // Clock polarity when inactive see TRM page 940

    EUSCI_B_SPI_3PIN // 3 pin SPI mode

};





//---------------------------

//  Prototypes

//---------------------------

 void ArduCam_SPI_Tx(uint8_t cmd, uint8_t data)

 {

     CAM_CS_LO;      // ArduCam CS low



     while(!(EUSCI_B0->IFG & SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));

     SPI_transmitData(EUSCI_B0_BASE, cmd);

     // The check on UCTXIFG avoids overwriting data in the tx buffer before it is transferred to the shift register



     while(!(EUSCI_B0->IFG & SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));          // Check if SPI Tx buffer is empty

     SPI_transmitData(EUSCI_B0_BASE, data);



     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);     // Check if SPI is busy

     CAM_CS_HI;      // ArduCam CS high

     // The check for SPI_isBusy avoids removing the CS too early



 }   // ArduCam_SPI_Tx

 uint8_t ArduCam_SPI_Rx(uint8_t cmd) {

     volatile uint8_t data = 0x00;
     uint8_t t;

     CAM_CS_LO;


     while(!(EUSCI_B0->IFG & SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
     SPI_transmitData(EUSCI_B0_BASE, cmd);


     while(!(EUSCI_B0->IFG & SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT)));

     data = SPI_receiveData(EUSCI_B0_BASE);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

     CAM_CS_HI;

     return data;


 }

 void ardu_capture() {

     uint16_t capture_cmd = 0x8401;         //command sent to ArduCam to tell it to beginc capturing a single frame.
     uint16_t clear_flag_cmd = 0x4101;      //command to clear the capture done flag, allowing for a new capture command to begin
     uint8_t poll_done_cmd = 0x41;          //command for polling the capture done flag
     uint8_t capture_done_flag = 0x00;      //variable for capturing the capture done flag
     uint8_t i;                             //variable for iterating

     //Lets make sure our capture is reset;

     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x41);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x01);


     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;




     CAM_CS_LO;     //chip select to enable arducam
     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));     //wait for msp432 to be ready
     SPI_transmitData(EUSCI_B0_BASE, 0x84);                  //transmit capture command

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));     //wait for msp432 to be ready
     SPI_transmitData(EUSCI_B0_BASE, 0x02);                  //transmit capture command


     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;


     while(capture_done_flag != 0x08) {

         CAM_CS_LO;

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, 0x41);     //keep sending dummy byte to drive clock
         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, 0x41);     //keep sending dummy byte to drive clock

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0));
         capture_done_flag = SPI_receiveData(EUSCI_B0_BASE);
         capture_done_flag = capture_done_flag & 0x08;

         while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
         CAM_CS_HI;

     }

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;
     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x41);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x01);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

     CAM_CS_HI;

     return;



 }

 void ardu_readCommand(uint16_t *byte_map[120][160][2]){

     uint8_t read_command = 0x3D;
     uint8_t i;
     uint8_t j;
     uint8_t k;


     for(i = 0; i < 120; i++){

         for(j = 0; j < 160; j++){

             CAM_CS_LO;

             while(!(SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));       //send read command to read byte from frame buffer
             SPI_transmitData(EUSCI_B0_BASE, 0x3D);

             while(!(SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));       //send dummy byte to drive clock
             SPI_transmitData(EUSCI_B0_BASE, 0x00);

             while(!(SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT) && SPI_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
             (*byte_map)[i][j][0] = SPI_receiveData(EUSCI_B0_BASE);

             while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
             CAM_CS_LO;


         }

     }

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

     CAM_CS_HI;

 }

 void init_ArduCam() {

     uint8_t i;
     uint8_t j;

     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0xff);


     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x01);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;


     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x12);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x80);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;


     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x07);


     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x80);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;

     CAM_CS_LO;


     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x07);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x00);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

     CAM_CS_HI;

     for(i = 0; i < 190; i ++){

         CAM_CS_LO;

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG_INIT[i][0]);



         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG_INIT[i][1]);



         while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

         CAM_CS_HI;


     }

     for(i = 0; i <9; i++) {

         CAM_CS_LO;

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, YUV_422[i][0]);

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, YUV_422[i][1]);



         while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

         CAM_CS_HI;

     }

     for(i = 0; i < 8; i++) {

         CAM_CS_LO;

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG[i][0]);

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG[i][1]);

         while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

         CAM_CS_HI;

     }

     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0xff);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x01);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;

     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x15);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x00);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;



     for(i = 0; i < 39; i++) {


         CAM_CS_LO;

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG_160x120[i][0]);

         while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
         SPI_transmitData(EUSCI_B0_BASE, JPEG_160x120[i][1]);

         while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

         CAM_CS_HI;

     }

     CAM_CS_LO;

      while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
      SPI_transmitData(EUSCI_B0_BASE, 0x01);

      while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
      SPI_transmitData(EUSCI_B0_BASE, 0x00);

      while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

      CAM_CS_HI;

     return;

 };

 void process_frame(uint8_t frame[120][160], uint32_t *skew) {    //an array for storing how "skewed" our image is. [0] is up, [1] down, [2] left, and [3] right.

     uint8_t i;
     uint8_t j;


     uint32_t temp = 0;     //used for each loop to calculate its skew. Then copied into array.

     for(i = 0; i < 60; i++){       //for processing how "up" our image is skewed

         for(j = 0; j < 160; j++){

             temp = temp + frame[i][j] * (60 - i);

         }

     }

     skew[0] = temp;

     temp = 0;


     for(i = 60; i < 120; i++){     //for processing how "down" our image is skewed

         for(j = 0; j < 160; j++){

             temp = temp + frame[i][j] * (i - 59);

         }

     }

     skew[1] = temp;

     temp = 0;

     for(i = 0; i < 120; i++) {     //for processing how "left" our image is

         for(j = 0; j < 80; j ++) {

             temp = temp + frame[i][j] * (80 - j);

         }

     }

     skew[2] = temp;

     temp = 0;

     for(i = 0; i < 120; i++) {

         for(j = 60; j < 160; j++) {

             temp = temp + frame[i][j] * (j - 79);

         }

     }

     skew[3] = temp;

     return;

 };

void movements(uint32_t skew[4], uint8_t* motor_commands) {

     uint32_t temp = 0;     //temporary variable for doing calculations


     if(skew[0] > skew[1]) {    //we need to adjust our collimation upwards, from the image's perspective

         temp = skew[0] - skew[1];
         temp = temp /  2448000;
         temp = temp * 100;

         motor_commands[0] = temp;


     }

     else{  //we need to adjust our collimation downwards, from the image's perspective

         temp = skew [1] - skew[0];
         temp = temp / 2448000;
         temp = temp * 100;

         motor_commands[1] = temp;


     }

     temp = 0;

     if(skew[2] > skew[3]) {        //we need to adjust our collimation rightwards, from the image's perspective

         temp = skew[2] - skew [3];         //find our how skewed exactly we are to the left;
         temp = temp / 2448000;             //find our how skewed we are in comparison to the maximum amount of skewed we can be. Max skewed would assume right completely black
                                            //and left completely white, giving difference of 2448000. This would of course mean our image is off center instead of our collimation image being actually skewed in relation to
                                            //itself, but for now we'll assume our image is completely centered on our camera, and any difference is due to our collimation being skewed, instead of artifacts in the image
         temp = temp * 100;                 //get a percentage.

         motor_commands[2] = temp;

     }

     else {     //we need to adjust our collimation leftwards, from the image's perspective


         temp = skew[3] - skew[2];
         temp = temp / 2448000;
         temp = temp * 100;

         motor_commands[3] = temp;

     }

     return;

 }

 static void showChar(char c, int position)
 {
     uint8_t ii;
     if (c == ' ')
     {
         for (ii=0; ii<4; ii++)
         {
             LCD_F->M[position+ii] = 0x00;
         }
     }
     else if (c >= '0' && c <= '9')
     {
         for (ii=0; ii<4; ii++)
         {
             LCD_F->M[position+ii] = digit[c-48][ii];
         }
     }
     else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))
     {
         for (ii=0; ii<4; ii++)
         {
             LCD_F->M[position+ii] = alphabetBig[c-65][ii];
         }
     }
     else
     {
         LCD_F->M[position] = 0xFF;
     }
 }

 void ardu_clear_fifo() {

     CAM_CS_LO;

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x84);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x01);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);

     CAM_CS_HI;

 }

 uint32_t ardu_read_fifo_length() {

     uint32_t len1, len2, len3, length = 0;

     CAM_CS_LO;
     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x42);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     SPI_transmitData(EUSCI_B0_BASE, 0x42);

     while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
     len1 = SPI_receiveData(EUSCI_B0_BASE);

     while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
     CAM_CS_HI;

     CAM_CS_LO;
      while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
      SPI_transmitData(EUSCI_B0_BASE, 0x43);

      while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
      SPI_transmitData(EUSCI_B0_BASE, 0x43);

      while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
      len2 = SPI_receiveData(EUSCI_B0_BASE);

      while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
      CAM_CS_HI;

      CAM_CS_LO;
       while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
       SPI_transmitData(EUSCI_B0_BASE, 0x44);

       while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
       SPI_transmitData(EUSCI_B0_BASE, 0x44);

       while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
       len3 = SPI_receiveData(EUSCI_B0_BASE) & 0x7f;

       while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
       CAM_CS_HI;

       length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;

       return length;



 }

 void ardu_set_bw() {

     uint8_t i;

     for(i = 0; i < 6; i++){

        CAM_CS_LO;

        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        SPI_transmitData(EUSCI_B0_BASE, black_white[i][0]);

        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        SPI_transmitData(EUSCI_B0_BASE, black_white[i][1]);

        while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
        CAM_CS_HI;

     }


 }



//---------------------------

// Main C-entry

//---------------------------

void main(void)

{

     P3->SEL1 |= 0xF2;
     P6->SEL1 |= 0x0C;
     P7->SEL1 |= 0xF0;
     P8->SEL1 |= 0xFC;
     P9->SEL1 |= 0xFF;
     P10->SEL1 |= 0x3F;

     /* Setting ACLK to the reference oscillator */
     CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

     /* Initializing the LCD_F module */
     LCD_F_initModule(&lcdConf);

     /* Clearing out all memory */
     LCD_F_clearAllMemory();

     /* Initializing all of our pins and setting the relevant COM lines */
     LCD_F_setPinsAsLCDFunction(LCD_F_SEGMENT_LINE_0, LCD_F_SEGMENT_LINE_3);
     LCD_F_setPinAsLCDFunction(LCD_F_SEGMENT_LINE_6);
     LCD_F_setPinsAsLCDFunction(LCD_F_SEGMENT_LINE_16, LCD_F_SEGMENT_LINE_19);
     LCD_F_setPinsAsLCDFunction(LCD_F_SEGMENT_LINE_26, LCD_F_SEGMENT_LINE_47);
     LCD_F_setPinAsCOM(LCD_F_SEGMENT_LINE_26, LCD_F_MEMORY_COM0);
     LCD_F_setPinAsCOM(LCD_F_SEGMENT_LINE_27, LCD_F_MEMORY_COM1);
     LCD_F_setPinAsCOM(LCD_F_SEGMENT_LINE_6, LCD_F_MEMORY_COM2);
     LCD_F_setPinAsCOM(LCD_F_SEGMENT_LINE_3, LCD_F_MEMORY_COM3);

     /* Turing the LCD_F module on */
     LCD_F_turnOn();

     /* 123ABC */


    uint16_t capture = 0x8402;
    uint16_t clear_flag = 0x4101;
    volatile uint8_t capture_flag = 0x00;
    uint8_t poll_flag = 0x41;
    uint16_t test_write = 0x8008;
    volatile uint8_t test_read = 0xFF;
    uint32_t length = 0;
    uint8_t frame[160][120][3];
    uint32_t receive_buffer;

    char buff[2];

    sprintf(buff, "%X", poll_flag);

    showChar('0', char3);
    showChar('X', char4);
    showChar(buff[0], char5);
    showChar(buff[1], char6);



    uint32_t Cnt;
    uint32_t i;
    int j;

    uint16_t LedStat = 0;



    /* Halting WDT  */

    MAP_WDT_A_holdTimer();



    // Set GPIO for LED1 (red)

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    //set GPIO for LED2 (red)

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);

    //set GPIO for LED2 (green)

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);

    //set GPIO for LED2 (Blue)

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);


    // Camera chip select on P4.1. Set to output and drive '0'

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);



    // Set SMCLK/HSMCLK to 6 MHz

    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);

    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);



    // Output MCLK on P4.3 and HSMCLK on P4.4

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN3 | GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);



    // Configure CLK, MOSI & MISO for SPI0 (EUSCI_B0)

    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN5 | GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);



    // Configuring SPI module

    MAP_SPI_initMaster(EUSCI_B0_BASE, &spiMasterConfig);



    // Enable the SPI module

    MAP_SPI_enableModule(EUSCI_B0_BASE);

    SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
    SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_TRANSMIT_INTERRUPT);


    init_ArduCam();
    //ardu_set_bw();



    while(1)

    {

       ardu_clear_fifo();

       ardu_capture();

       length = ardu_read_fifo_length();

       MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);

       //CAM_CS_LO;

       //while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
       //SPI_transmitData(EUSCI_B0_BASE, 0x3C);



       for(Cnt = 0; Cnt < 160; Cnt++) {

           for(i = 0; i < 120; i++){

               for(j = 0; j < 3; j++) {

                   CAM_CS_LO;

                   while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
                   SPI_transmitData(EUSCI_B0_BASE, 0x3D);



                   while(!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
                   SPI_transmitData(EUSCI_B0_BASE, 0x00);


                   while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0));
                   receive_buffer = SPI_receiveData(EUSCI_B0_BASE);

                   sprintf(buff, "%X", receive_buffer);
                   //showChar('0', char3);
                   showChar('X', char4);
                   showChar(buff[0], char5);
                   showChar(buff[1], char6);
                   frame[Cnt][i][3] = receive_buffer;

                   while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
                   CAM_CS_HI;


               }


           }

           char temp[3];
           sprintf(temp, "%u", Cnt);

           if(Cnt < 10) {

               showChar('0', char1);
               showChar('0', char2);
               showChar(temp[0], char3);

           }

           else if(Cnt < 100) {

               showChar('0', char2);
               showChar(temp[0], char2);
               showChar(temp[1], char3);

           }

           else {

               showChar(temp[0], char1);
               showChar(temp[1], char2);
               showChar(temp[2], char3);


           }

       }

       //while(SPI_isBusy(EUSCI_B0_BASE) == EUSCI_SPI_BUSY);
       //CAM_CS_HI;

       MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);

       MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);

       uint32_t skew[4] = {0, 0, 0, 0};
       process_frame(frame, skew);
       uint8_t motor_commands[4] = {0, 0, 0, 0};
       movements(skew, motor_commands); //0 up 1 down 2 right 3 left



       char commands_1[2];
       char commands_2[2];

       if(motor_commands[0] > motor_commands[1]) {


           if(motor_commands[2] > motor_commands[3]) {

               sprintf(commands_1,"%d", motor_commands[0]);
               sprintf(commands_2, "%d", motor_commands[2]);

               showChar('U', char1);
               showChar(commands_1[0], char2);
               showChar(commands_1[1], char3);
               showChar('R', char4);
               showChar(commands_2[0], char5);
               showChar(commands_2[1], char6);

           }

           else{

               sprintf(commands_1, "%d", motor_commands[0]);
               sprintf(commands_2, "%d", motor_commands[3]);

               showChar('U', char1);
               showChar(commands_1[0], char2);
               showChar(commands_1[1], char3);
               showChar('L', char4);
               showChar(commands_2[0], char5);
               showChar(commands_2[1], char6);

           }

       }

       else {

           if(motor_commands[2] > motor_commands[3]) {

               sprintf(commands_1,"%d", motor_commands[1]);
               sprintf(commands_2,"%d", motor_commands[2]);

               showChar('D', char1);
               showChar(commands_1[0], char2);
               showChar(commands_1[1], char3);
               showChar('R', char4);
               showChar(commands_2[0], char5);
               showChar(commands_2[1], char6);

           }

           else {

               sprintf(commands_1, "%d", motor_commands[1]);
               sprintf(commands_2, "%d", motor_commands[3]);

               showChar('D', char1);
               showChar(commands_1[0], char2);
               showChar(commands_1[1], char3);
               showChar('L', char4);
               showChar(commands_2[0], char5);
               showChar(commands_2[1], char6);

           }

       }

       //sprintf(moves, "%X", )


        // Toggle LED


        for(Cnt=0; Cnt < 500000; Cnt++)

        {

            __no_operation();

        }



        if(LedStat == 0)

        {

            LedStat = 1;

            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //            CAM_CS_HI;

        }

        else

        {

            LedStat = 0;

            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //            CAM_CS_LO;

        }

        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1);

    }   // while(1)



}   // main



