/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_dma.h"
#include "fsl_i2c.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"
#include "fsl_wm8904.h"
#include "ring_buffer.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEMO_I2C (I2C2)
#define DEMO_I2C_MASTER_CLOCK_FREQUENCY (12000000)
#define DEMO_I2S_TX (I2S0)
#define DEMO_I2S_RX (I2S1)
#define DEMO_DMA (DMA0)
#define DEMO_I2S_TX_CHANNEL (13)
#define DEMO_I2S_RX_CHANNEL (14)
#define I2S_CLOCK_DIVIDER (CLOCK_GetAudioPllOutFreq() / 48000U / 16U / 2U)

typedef union
{
	uint32_t Data;
	int16_t Channel[2];

}I2S_FIFO_Data_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void FLEXCOMM6_DriverIRQHandler(void);
void FLEXCOMM7_DriverIRQHandler(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static i2s_config_t s_TxConfig;
static i2s_config_t s_RxConfig;
volatile uint32_t NextSampleOut = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Transmit Interrupt - send data to headphones */
void FLEXCOMM6_DriverIRQHandler(void)
{
	if (DEMO_I2S_TX->FIFOINTSTAT & I2S_FIFOINTSTAT_TXLVL_MASK)
	{
		/*
			NextSampleOut Holds the last value from the I2S RX Interrupt.
			It is also ready in the "packed" FIFO format
		*/
		DEMO_I2S_TX->FIFOWR = NextSampleOut;

		/* Clear TX level interrupt flag */
		DEMO_I2S_TX->FIFOSTAT = I2S_FIFOSTAT_TXLVL(1U);
	}
}

void FLEXCOMM7_DriverIRQHandler(void)
{
	register float LeftChannel;
	register float RightChannel;
	I2S_FIFO_Data_t FIFO_Data;

	/* Clear RX level interrupt flag */
	I2S1->FIFOSTAT = I2S_FIFOSTAT_RXLVL(1U);

	/*
		Read the Recieve FIFO.   Data is packed as two samples in one 32-bit word.  We will immediately store the data
		in a variable that is used is the transmit routine to send incoming data back out.
	*/
	FIFO_Data.Data = I2S1->FIFORD;
	NextSampleOut = FIFO_Data.Data; //dump the data back out!

	/*
	In the configuration for this lab,  2 channels of data are packed
	in one 32-bit word.  The Right Channel is in the upper 16-bits and the Left-Channel in the lower
	Notice between we can use a "union" (I2S_FIFO_Data_t) to read the data in as 32-bit and access it as two 16-bit signed numbers.
	*/

	LeftChannel = (float)(FIFO_Data.Channel[0])/32768.0f;
	RightChannel = (float)(FIFO_Data.Channel[1])/32768.0f;

	/*
	Do something with the Left and Right channel here

	*/
}

int main(void)
{
    i2c_master_config_t i2cConfig;

    wm8904_config_t codecConfig;
    wm8904_handle_t codecHandle;

    pll_config_t audio_pll_config = {

        .desiredRate = 24576000U, .inputRate = 12000000U,
    };

    pll_setup_t audio_pll_setup;

    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    /* USART0 clock */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* I2C clock */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);

    /* Initialize AUDIO PLL clock */
    CLOCK_SetupAudioPLLData(&audio_pll_config, &audio_pll_setup);
    audio_pll_setup.flags = PLL_SETUPFLAG_POWERUP | PLL_SETUPFLAG_WAITLOCK;
    CLOCK_SetupAudioPLLPrec(&audio_pll_setup, audio_pll_setup.flags);

    /* I2S clocks */
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM6);
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM7);

    /* Attach AUDIO PLL clock to MCLK for I2S, no divider */
    CLOCK_AttachClk(kAUDIO_PLL_to_MCLK);
    SYSCON->MCLKDIV = SYSCON_MCLKDIV_DIV(0U);
    SYSCON->MCLKIO = 1U;

    /* reset FLEXCOMM for I2C */
    RESET_PeripheralReset(kFC2_RST_SHIFT_RSTn);

    /* reset FLEXCOMM for I2S */
    RESET_PeripheralReset(kFC6_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kFC7_RST_SHIFT_RSTn);

    /* Enable interrupts for I2S */
    EnableIRQ(FLEXCOMM6_IRQn);
    EnableIRQ(FLEXCOMM7_IRQn);

    /* Initialize the rest */
    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    PRINTF("Configure I2C\r\n");

    /*
     * enableMaster = true;
     * baudRate_Bps = 100000U;
     * enableTimeout = false;
     */
    I2C_MasterGetDefaultConfig(&i2cConfig);
    i2cConfig.baudRate_Bps = WM8904_I2C_BITRATE;
    I2C_MasterInit(DEMO_I2C, &i2cConfig, DEMO_I2C_MASTER_CLOCK_FREQUENCY);

    PRINTF("Configure WM8904 codec\r\n");

    WM8904_GetDefaultConfig(&codecConfig);
    codecHandle.i2c = DEMO_I2C;
    if (WM8904_Init(&codecHandle, &codecConfig) != kStatus_Success)
    {
        PRINTF("WM8904_Init failed!\r\n");
    }
    /* Initial volume kept low for hearing safety. */
    /* Adjust it to your needs, 0x0006 for -51 dB, 0x0039 for 0 dB etc. */
    WM8904_SetVolume(&codecHandle, 0x001E, 0x001E);

    PRINTF("Configure I2S\r\n");

    /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = true;
     * pack48 = false;
     */
    I2S_TxGetDefaultConfig(&s_TxConfig);
    s_TxConfig.divider = I2S_CLOCK_DIVIDER;

    /*
     * masterSlave = kI2S_MasterSlaveNormalSlave;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * watermark = 4;
     * txEmptyZero = false;
     * pack48 = true;
     */
    I2S_RxGetDefaultConfig(&s_RxConfig);

    s_RxConfig.watermark = 2;
    s_TxConfig.watermark = 2;

    I2S_TxInit(DEMO_I2S_TX, &s_TxConfig);
    I2S_RxInit(DEMO_I2S_RX, &s_RxConfig);

    /* Enable I2S Interrupts on FIFO Watermarks */
    NVIC_SetPriority(FLEXCOMM6_IRQn,0);
	NVIC_SetPriority(FLEXCOMM7_IRQn,0);

	NVIC_EnableIRQ(FLEXCOMM6_IRQn);
	NVIC_EnableIRQ(FLEXCOMM7_IRQn);

	I2S0->FIFOWR = 0xFFFFFFFF;
	I2S0->FIFOWR = 0xFFFFFFFF;

	I2S_EnableInterrupts(DEMO_I2S_TX, kI2S_TxLevelFlag);
	I2S_EnableInterrupts(DEMO_I2S_RX, kI2S_RxLevelFlag);

	I2S_Enable(I2S0);
	I2S_Enable(I2S1);

    while (1)
    {
    }
}
