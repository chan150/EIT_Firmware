/**********************************

Authors: Jean Rintoul , Mindseye Biomedical LLC; Andrew Grosser, SF Product Labs LLC;
This software is modified from an original Analog Devices example, 
It's original copyright notice is included below. 

All code additions to this project are under a 
Creative Commons Non-Commercial License((CC BY-NC-SA 4.0)). 

If you are a person who wants to experiment/hack or publish etc, please go ahead and do it! 
We'd love an acknowledgement.

If you are planning a commercial enterprise and have used this work in your development process, 
please get in contact with Mindseye Biomedical LLC (jean2mindseyebiomedical.com)
We'd love to License the technomlogy to you under a separate reasonable agreement,  
or even help/assist/coordinate your development under a consulting agreement. 

Please support Open Source Hardware! Imagine a world where we shared knowledge, 
to enable each other to reach greater heights. 

All contributions are much appreciated! Donations, Code Contributions are all welcome. 


Copyright (c) 2014 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.  By using 
this software you agree to the terms of the associated Analog Devices Software 
License Agreement.

*********************************************************************************/

//#ifdef _MISRA_RULES
//#pragma diag(push)
//#pragma diag(suppress:misra_rules_all:"suppress all MISRA rules for test")
//#endif /* _MISRA_RULES */

#include <stdio.h>
#include <math.h>
#include <stddef.h>  // for 'NULL'
#include <stdio.h>   // for scanf
#include <string.h>  // for strncmp
#include <stdint.h>

#define ARM_MATH_CM3 1
#include "arm_math.h"
#include "test_common.h"
#include "afe.h"
#include "afe_lib.h"
#include "uart.h"
#include "sequences.h"
#include "lookup.h" 
#include "gpio.h"
#include "modes.h"

#include <ADuCM350.h>
#include <cdefADuCM350.h>

/* Pre-calculated multiplexer pin register masks for 32 electrode indices (0-31) */
static uint16_t mx1_set_lut[32];
static uint16_t mx1_clr_lut[32];
static uint16_t mx2_set_lut[32];
static uint16_t mx2_clr_lut[32];
static uint16_t mx3_gp1_set_lut[32];
static uint16_t mx3_gp1_clr_lut[32];
static uint16_t mx3_gp3_set_lut[32];
static uint16_t mx3_gp3_clr_lut[32];
static uint16_t mx4_set_lut[32];
static uint16_t mx4_clr_lut[32];

void init_multiplexer_lut(void) {
    for (int val = 0; val < 32; val++) {
        // MX1 (A-): Port 1, Pins 4, 3, 2, 1, 0 (bit 4, 3, 2, 1, 0)
        uint16_t mx1_set = 0;
        uint16_t mx1_clr = 0;
        for (int i = 0; i < 5; i++) {
            int pin = 4 - i;
            int bit = (val >> (4 - i)) & 1;
            if (bit) {
                mx1_set |= (1 << pin);
            } else {
                mx1_clr |= (1 << pin);
            }
        }
        mx1_set_lut[val] = mx1_set;
        mx1_clr_lut[val] = mx1_clr;

        // MX2 (V-): Port 1, Pins 10, 9, 8, 7, 6 (bit 4, 3, 2, 1, 0)
        uint16_t mx2_set = 0;
        uint16_t mx2_clr = 0;
        for (int i = 0; i < 5; i++) {
            int pin = 10 - i;
            int bit = (val >> (4 - i)) & 1;
            if (bit) {
                mx2_set |= (1 << pin);
            } else {
                mx2_clr |= (1 << pin);
            }
        }
        mx2_set_lut[val] = mx2_set;
        mx2_clr_lut[val] = mx2_clr;

        // MX3 (A+): Port 3 Pin 13 (A4) and Port 1 Pins 15, 14, 13, 12 (A3, A2, A1, A0)
        uint16_t mx3_gp1_set = 0;
        uint16_t mx3_gp1_clr = 0;
        uint16_t mx3_gp3_set = 0;
        uint16_t mx3_gp3_clr = 0;
        
        if ((val >> 4) & 1) {
            mx3_gp3_set |= (1 << 13);
        } else {
            mx3_gp3_clr |= (1 << 13);
        }
        
        for (int i = 1; i < 5; i++) {
            int pin = 15 - (i - 1);
            int bit = (val >> (4 - i)) & 1;
            if (bit) {
                mx3_gp1_set |= (1 << pin);
            } else {
                mx3_gp1_clr |= (1 << pin);
            }
        }
        mx3_gp1_set_lut[val] = mx3_gp1_set;
        mx3_gp1_clr_lut[val] = mx3_gp1_clr;
        mx3_gp3_set_lut[val] = mx3_gp3_set;
        mx3_gp3_clr_lut[val] = mx3_gp3_clr;

        // MX4 (V+): Port 2 Pins 7, 8, 9, 10, 11 (bit 4, 3, 2, 1, 0)
        uint16_t mx4_set = 0;
        uint16_t mx4_clr = 0;
        for (int i = 0; i < 5; i++) {
            int pin = 7 + i;
            int bit = (val >> (4 - i)) & 1;
            if (bit) {
                mx4_set |= (1 << pin);
            } else {
                mx4_clr |= (1 << pin);
            }
        }
        mx4_set_lut[val] = mx4_set;
        mx4_clr_lut[val] = mx4_clr;
    }
}

static inline void multiplex_adg732_fast(const int16_t *e) {
    uint16_t e0 = e[0]; // A+
    uint16_t e1 = e[1]; // A-
    uint16_t e2 = e[2]; // V+
    uint16_t e3 = e[3]; // V-

    *pREG_GPIO1_GPSET = mx1_set_lut[e1] | mx2_set_lut[e3] | mx3_gp1_set_lut[e0];
    *pREG_GPIO1_GPCLR = mx1_clr_lut[e1] | mx2_clr_lut[e3] | mx3_gp1_clr_lut[e0];

    *pREG_GPIO2_GPSET = mx4_set_lut[e2];
    *pREG_GPIO2_GPCLR = mx4_clr_lut[e2];

    *pREG_GPIO3_GPSET = mx3_gp3_set_lut[e0];
    *pREG_GPIO3_GPCLR = mx3_gp3_clr_lut[e0];
}

//#if defined ( __ICCARM__ )  // IAR compiler...
///* Apply ADI MISRA Suppressions */
//#define ASSERT_ADI_MISRA_SUPPRESSIONS
//#include "misra.h"
//#endif

/* Macro to enable the returning of AFE data using the UART */
/*      1 = return AFE data on UART                         */
/*      0 = return AFE data on SW (Std Output)              */
#define USE_UART_FOR_DATA           (1)
/* Helper macro for printing strings to UART or Std. Output */
#define PRINT(s)                    test_print(s)

/* Helper to do 64 bit division - Required for Cortex M3 (32 bit only division)*/
#define DIV64(a,b,c) __divmoddi4(a,b,c)

/* Size of Tx and Rx buffers */
#define RX_BUFFER_SIZE     2
#define TX_BUFFER_SIZE     100

/* Rx and Tx buffers */
static uint8_t RxBuffer[RX_BUFFER_SIZE];
static uint8_t TxBuffer[TX_BUFFER_SIZE];

/* UART Handle */
ADI_UART_HANDLE      hUartDevice;
ADI_AFE_DEV_HANDLE   hDevice; 
    
/* Custom fixed-point type used for final results,              */
/* to keep track of the decimal point position.                 */
/* Signed number with 28 integer bits and 4 fractional bits.    */
typedef union {
    int32_t     full;
    struct {
        uint8_t fpart:4;
        int32_t ipart:28;
    } parts;
} fixed32_t;


/* Function prototypes */
q15_t                   arctan                  (q15_t imag, q15_t real);
fixed32_t calculate_magnitude(q31_t magnitude_1, q31_t magnitude_2, uint32_t res);
fixed32_t               calculate_phase         (q15_t phase_rcal, q15_t phase_z);
void                    convert_dft_results     (int16_t *dft_results, q15_t *dft_results_q15, q31_t *dft_results_q31);
void                    convert_dft_results_single(int16_t *dft_results, q15_t *dft_results_q15, q31_t *dft_results_q31);
q31_t                   compute_goertzel_magnitude(int16_t *samples, int num_samples, float target_frequency, float sampling_rate);
void                    sprintf_fixed32         (char *out, fixed32_t in);
void                    print_MagnitudePhase    (char *text, fixed32_t magnitude, fixed32_t phase);
void                    test_print              (char *pBuffer);
ADI_UART_RESULT_TYPE    uart_Init               (void);
ADI_UART_RESULT_TYPE    uart_UnInit             (void);
void                    delay                   (uint32_t counts);
extern int32_t          adi_initpinmux          (void);
void                    multiplex_adg732        (ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq,uint32_t n_el);
void                    time_series             (ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq);
void                    bioimpedance_spectroscopy     (ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq);
void                    init_GPIO_ports         (void);
fixed32_t               calculate_bipolar_magnitude     (q31_t magnitude_rcal, q31_t magnitude_z);
void                    init_mode_tetramux        (void);
void                    init_mode_bis             (void);
void                    init_mode_bipolar       (void);
void                    time_series_bipolar(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq);
fixed32_t               calculate_bipolar_magnitude     (q31_t magnitude_rcal, q31_t magnitude_z);
void                    bipolar_adg732(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq,uint32_t n_el);

#if defined (DEBUG) && defined (__GNUC__) && !defined (NDEBUG)
	extern void initialise_monitor_handles(void);
#endif

int main(void) {
#if defined (DEBUG) && defined (__GNUC__) && !defined (NDEBUG)
	initialise_monitor_handles();
#endif

    ADI_UART_RESULT_TYPE uartResult;
    int16_t  rxSize;
    int16_t  mode = 0; 
    
    /* Flag which indicates whether to stop the program */
    _Bool bStopFlag = false;



    /* Initialize system */
    SystemInit();


    /* NVIC initialization */
    NVIC_SetPriorityGrouping(12);


    /* Change the system clock source to HFXTAL and change clock frequency to 16MHz     */
    /* Requirement for AFE (ACLK)                                                       */
    //SystemTransitionClocks(ADI_SYS_CLOCK_TRIGGER_MEASUREMENT_OFF);


    // Change HCLK clock divider to 1 for a 16MHz clock
       if (ADI_SYS_SUCCESS != SetSystemClockDivider(ADI_SYS_CLOCK_CORE, 1))
       {
           test_Fail("SetSystemClockDivider() failed");
       }

       // Change PCLK clock divider to 1 for a 16MHz clock
       if (ADI_SYS_SUCCESS != SetSystemClockDivider(ADI_SYS_CLOCK_UART, 1))
       {
           test_Fail("SetSystemClockDivider() failed");
   }


       /* Change the system clock source to HFXTAL and change clock frequency to 16MHz     */
      /* Requirement for AFE (ACLK)                                                       */
      //SystemTransitionClocks(ADI_SYS_CLOCK_TRIGGER_MEASUREMENT_ON); //AG

      /* SPLL with 32MHz used, need to divide by 2 */
      //SetSystemClockDivider(ADI_SYS_CLOCK_UART, 2); //AG

    /* Test initialization */
    test_Init();

    /* Initialize multiplexer LUT */
    init_multiplexer_lut();

    /* Initialize static pinmuxing */
    adi_initpinmux();
    

    /* Initialize the UART */
    if (ADI_UART_SUCCESS != uart_Init())
    {
        FAIL("uart_Init");
    }    
    
    PRINT("\n*******************\n");    
    PRINT("OpenEIT:\n");
    PRINT("Press:\n a for Time-Series,\n b for Bioimpedance Spectroscopy,\n c for Imaging followed by return key.\n");
    PRINT("*******************\n");     
    
    /* UART processing loop */
    while(bStopFlag == false)
    {
        rxSize = 2;
        /* Read a character */
        uartResult = adi_UART_BufRx(hUartDevice, RxBuffer, &rxSize);

        /* Select 1,2,3 to enter into a different mode. */
        if(RxBuffer[0] == 'a' && RxBuffer[1] == '\r' )  // Time-Series
        {
          mode = 1;
          PRINT("MODE 1: Time Series Impedance data\n");
          init_mode_tetramux();
          /* Stop the program upon receiving carriage return */
          bStopFlag = true;      

        }          
        else if (RxBuffer[0] == 'b' && RxBuffer[1] == '\r' )  // Bioimpedance Spectroscopy
        {
          mode = 2;
          PRINT("MODE 2: Bioimpedance Spectroscopy\n");
          init_mode_bis();
          /* Stop the program upon receiving carriage return */
          bStopFlag = true;
        }
        else if (RxBuffer[0] == 'c'  && RxBuffer[1] == '\r' )  // Tetrapolar Imaging 
        {
          mode = 3;
          PRINT("MODE 3: Imaging\n");
          init_mode_tetramux();
          /* Stop the program upon receiving carriage return */
          bStopFlag = true;    
          
        }
        else if (RxBuffer[0] == 'd'  && RxBuffer[1] == '\r' )  // Bipolar Imaging 
        {
          mode = 4;
          PRINT("MODE 4: Bipolar Imaging(faster for tank phantoms)\n");
          init_mode_bipolar();
          /* Stop the program upon receiving carriage return */
          bStopFlag = true;    
          
        }        
        else if (RxBuffer[0] == 'e'  && RxBuffer[1] == '\r' )  // Bipolar Time series Imaging 
        {
          mode = 5;
          PRINT("MODE 5: Bipolar Time-series)\n");
          init_mode_bipolar();
          /* Stop the program upon receiving carriage return */
          bStopFlag = true;    
          
        }                
        else { 
           PRINT("Press a for Time-Series, b for Bioimpedance Spectroscopy, c for Imaging followed by return key.\n");
        }
        adi_UART_BufFlush(hUartDevice);
        
     }
    
    PRINT("Completed Initialization\n\n");
    /* main processing loop */
    while (bStopFlag == true) // running
    {
      if (mode == 1) {  // time series
        time_series(hDevice, seq_afe_fast_meas_4wire);
      }
      else if (mode == 2) {  // bioimpedance spectroscopy
        bioimpedance_spectroscopy(hDevice, seq_afe_fast_acmeasBioZ_4wire);
      }
      else if (mode == 3) {  // 32 electrode imaging
        uint32_t n_el = 8;
        /* Perform the multiplex adg732 Tetrapolar Impedance measurements */
        multiplex_adg732(hDevice, seq_afe_fast_meas_4wire, n_el);
      }
      else if (mode == 4) {
        uint32_t n_el = 16;
        bipolar_adg732(hDevice, seq_fast_2wire_bipolar, n_el);
      }
      else if (mode == 5) {
        time_series_bipolar(hDevice, seq_fast_2wire_bipolar);
      }
      else {
        PRINT("no mode chosen\n");
      }
      
    }  // END OF WHILE LOOP 

    
    /* AFE Power Down */
    if (ADI_AFE_SUCCESS != adi_AFE_PowerDown(hDevice)) 
    {
        FAIL("PowerDown");
    }

    /* Uninitialize the AFE API */
    if (ADI_AFE_SUCCESS != adi_AFE_UnInit(hDevice)) 
    {
        FAIL("Uninit");
    }
        
    /* Close the UART */
    uartResult = adi_UART_UnInit(hUartDevice);
    if (ADI_UART_SUCCESS != uartResult)
    {
        FAIL("adi_UART_UnInit");
    }


}



/* Arctan Implementation                                                                                    */
/* =====================                                                                                    */
/* Arctan is calculated using the formula:                                                                  */
/*                                                                                                          */
/*      y = arctan(x) = 0.318253 * x + 0.003314 * x^2 - 0.130908 * x^3 + 0.068542 * x^4 - 0.009159 * x^5    */
/*                                                                                                          */
/* The angle in radians is given by (y * pi)                                                                */
/*                                                                                                          */
/* For the fixed-point implementation below, the coefficients are quantized to 16-bit and                   */
/* represented as 1.15                                                                                      */
/* The input vector is rotated until positioned between 0 and pi/4. After the arctan                        */
/* is calculated for the rotated vector, the initial angle is restored.                                     */
/* The format of the output is 1.15 and scaled by PI. To find the angle value in radians from the output    */
/* of this function, a multiplication by PI is needed.                                                      */

const q15_t coeff[5] = {
    (q15_t)0x28BD,     /*  0.318253 */
    (q15_t)0x006D,     /*  0.003314 */
    (q15_t)0xEF3E,     /* -0.130908 */
    (q15_t)0x08C6,     /*  0.068542 */
    (q15_t)0xFED4,     /* -0.009159 */
};

q15_t arctan(q15_t imag, q15_t real) {
    q15_t       t;
    q15_t       out;
    uint8_t     rotation; /* Clockwise, multiples of PI/4 */
    int8_t      i;

    if ((q15_t)0 == imag) {
        /* Check the sign*/
        if (real & (q15_t)0x8000) {
            /* Negative, return -PI */
            return (q15_t)0x8000;
        }
        else {
            return (q15_t)0;
        }
    }
    else {

        rotation = 0;
        /* Rotate the vector until it's placed in the first octant (0..PI/4) */
        if (imag < 0) {
            imag      = -imag;
            real      = -real;
            rotation += 4;
        }
        if (real <= 0) {
            /* Using 't' as temporary storage before its normal usage */
            t         = real;
            real      = imag;
            imag      = -t;
            rotation += 2;
        }
        if (real <= imag) {
            /* The addition below may overflow, drop 1 LSB precision if needed. */
            /* The subtraction cannot underflow.                                */
            t = real + imag;
            if (t < 0) {
                /* Overflow */
                t         = imag - real;
                real      = (q15_t)(((q31_t)real + (q31_t)imag) >> 1);
                imag      = t >> 1;
            }
            else {
                t         = imag - real;
                real      = (real + imag);
                imag      = t;              
            }
            rotation += 1;
        }

        /* Calculate tangent value */
        t = (q15_t)((q31_t)(imag << 15) / real);

        out = (q15_t)0;

        for (i = 4; i >=0; i--) {
            out += coeff[i];
            arm_mult_q15(&out, &t, &out, 1);
        }
        
        /* Rotate back to original position, in multiples of pi/4 */
        /* We're using 1.15 representation, scaled by pi, so pi/4 = 0x2000 */
        out += (rotation << 13);

        return out;
    }
}

/* This function performs dual functionality:                                           */
/* - open circuit check: the real and imaginary parts can be non-zero but very small    */
/*   due to noise. If they are within the defined thresholds, overwrite them with 0s,   */
/*   this will indicate an open.                                                        */
/* - convert the int16_t to q15_t and q31_t formats, needed for the magnitude and phase */
/*   calculations. */
void convert_dft_results(int16_t *dft_results, q15_t *dft_results_q15, q31_t *dft_results_q31) {
    int8_t      i;

    for (i = 0; i < (DFT_RESULTS_COUNT / 2); i++) {
        if ((dft_results[i] < DFT_RESULTS_OPEN_MAX_THR) &&
            (dft_results[i] > DFT_RESULTS_OPEN_MIN_THR) &&               /* real part */
            (dft_results[2 * i + 1] < DFT_RESULTS_OPEN_MAX_THR) &&
            (dft_results[2 * i + 1] > DFT_RESULTS_OPEN_MIN_THR)) {       /* imaginary part */

            /* Open circuit, force both real and imaginary parts to 0 */
            dft_results[i]           = 0;
            dft_results[2 * i + 1]   = 0;
        }
    }

    /*  Convert to 1.15 format */
    for (i = 0; i < DFT_RESULTS_COUNT; i++) {
        dft_results_q15[i] = (q15_t)dft_results[i];
    }

    /*  Convert to 1.31 format */
    arm_q15_to_q31(dft_results_q15, dft_results_q31, DFT_RESULTS_COUNT);

}

void convert_dft_results_single(int16_t *dft_results, q15_t *dft_results_q15, q31_t *dft_results_q31) {
    if ((dft_results[0] < DFT_RESULTS_OPEN_MAX_THR) &&
        (dft_results[0] > DFT_RESULTS_OPEN_MIN_THR) &&               /* real part */
        (dft_results[1] < DFT_RESULTS_OPEN_MAX_THR) &&
        (dft_results[1] > DFT_RESULTS_OPEN_MIN_THR)) {       /* imaginary part */

        /* Open circuit, force both real and imaginary parts to 0 */
        dft_results[0]           = 0;
        dft_results[1]           = 0;
    }

    /*  Convert to 1.15 format */
    dft_results_q15[0] = (q15_t)dft_results[0];
    dft_results_q15[1] = (q15_t)dft_results[1];

    /*  Convert to 1.31 format */
    arm_q15_to_q31(dft_results_q15, dft_results_q31, 2);
}

q31_t compute_goertzel_magnitude(int16_t *samples, int num_samples, float target_frequency, float sampling_rate) {
    float omega = (2.0f * 3.1415926535f * target_frequency) / sampling_rate;
    float sine = sinf(omega);
    float cosine = cosf(omega);
    float coeff = 2.0f * cosine;

    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        q0 = coeff * q1 - q2 + (float)samples[i];
        q2 = q1;
        q1 = q0;
    }

    float real = q1 - q2 * cosine;
    float imag = q2 * sine;
    float magnitude = sqrtf(real * real + imag * imag);

    /* Scale magnitude to match the 2048-point hardware DFT magnitude output scale */
    float scaled_mag = magnitude * (2048.0f / (float)num_samples);
    return (q31_t)scaled_mag;
}

/* Calculates magnitude.                                */      
/* performs the calculation:                            */
/*      magnitude = magnitude_1 / magnitude_2 * res     */
/* Output in custom fixed-point format (28.4)           */
fixed32_t calculate_magnitude(q31_t magnitude_1, q31_t magnitude_2, uint32_t res) {
    q63_t       magnitude;
    fixed32_t   out;

    magnitude = (q63_t)0;
    if ((q63_t)0 != magnitude_2) {
        magnitude = ((q63_t)magnitude_1 * (q63_t)res) << 5;
        /* Shift up for additional precision and rounding */
        magnitude = magnitude / (q63_t)magnitude_2;
        /* Rounding */
        magnitude = (magnitude + 1) >> 1;
    }

    /* Saturate if needed */
    if (magnitude &  0xFFFFFFFF00000000) {
        /* Cannot be negative */
        out.full = 0x7FFFFFFF;
    }
    else {
        out.full = magnitude & 0xFFFFFFFF;
    }
        
    return out;
}

/* Calculates calibrated magnitude.                                     */
/* The input values are the measured RCAL magnitude (magnitude_rcal)    */
/* and the measured magnitude of the unknown impedance (magnitude_z).   */
/* Performs the calculation:                                            */
/*      magnitude = magnitude_rcal / magnitude_z * RCAL                 */
/* Output in custom fixed-point format (28.4).                          */
fixed32_t calculate_bipolar_magnitude(q31_t magnitude_rcal, q31_t magnitude_z) {
    q63_t       magnitude;
    fixed32_t   out;

    magnitude = (q63_t)0;
    if ((q63_t)0 != magnitude_z) {
        magnitude = (q63_t)magnitude_rcal * (q63_t)RCAL;
        /* Shift up for additional precision and rounding */
        magnitude = (magnitude << 5) / (q63_t)magnitude_z;
        /* Rounding */
        magnitude = (magnitude + 1) >> 1;
    }

    /* Saturate if needed */
    if (magnitude &  0xFFFFFFFF00000000) {
        /* Cannot be negative */
        out.full = 0x7FFFFFFF;
    }
    else {
        out.full = magnitude & 0xFFFFFFFF;
    }
        
    return out;
}
/* Calculates phase.                                        */                     
/* performs the calculation:                                */
/*      phase = (phase_2 - phase_1) * PI / (2 * PI) * 180   */
/*            = (phase_2 - phase_1) * 180                   */
/* Output in custom fixed-point format (28.4).              */
fixed32_t calculate_phase(q15_t phase_1, q15_t phase_2) {
    q63_t       phase;
    fixed32_t   out;

    /* Multiply by 180 to convert to degrees */
    phase = ((q63_t)(phase_2 - phase_1) * (q63_t)180);                  
    /* Round and convert to fixed32_t */
    out.full = ((phase + (q63_t)0x400) >> 11) & 0xFFFFFFFF;

    return out;
}


/* Simple conversion of a fixed32_t variable to string format. */
void sprintf_fixed32(char *out, fixed32_t in) {
    fixed32_t   tmp;
    
    if (in.full < 0) {
        tmp.parts.fpart = (16 - in.parts.fpart) & 0x0F;
        tmp.parts.ipart = in.parts.ipart;
        if (0 != in.parts.fpart) {
            tmp.parts.ipart++;
        }
        if (0 == tmp.parts.ipart) {
            sprintf(out, "-0.%04d", tmp.parts.fpart * FIXED32_LSB_SIZE);
        }
        else {
            sprintf(out, "%d.%04d", tmp.parts.ipart, tmp.parts.fpart * FIXED32_LSB_SIZE);
        }
    }
    else {
        sprintf(out, "%d.%04d", in.parts.ipart, in.parts.fpart * FIXED32_LSB_SIZE);
    }

}

/* Helper function for printing fixed32_t (magnitude & phase) results */
void print_MagnitudePhase(char *text, fixed32_t magnitude, fixed32_t phase) {
    char                msg[MSG_MAXLEN_M3];
    char                tmp[MSG_MAXLEN_M3];

    sprintf(msg, "    %s = (", text);
    /* Magnitude */
    sprintf_fixed32(tmp, magnitude);
    strcat(msg, tmp);
    strcat(msg, ", ");
    /* Phase */
    sprintf_fixed32(tmp, phase);
    strcat(msg, tmp);
    strcat(msg, ")\r\n");

    PRINT(msg);
}


/* Helper function for printing a string to UART or Std. Output */
void test_print (char *pBuffer) {
#if (1 == USE_UART_FOR_DATA)
    int16_t size;
    /* Print to UART */
    size = strlen(pBuffer);
    adi_UART_BufTx(hUartDevice, pBuffer, &size);

#elif (0 == USE_UART_FOR_DATA)
    /* Print  to console */
    printf(pBuffer);
#endif /* USE_UART_FOR_DATA */
}

/* Initialize the UART, set the baud rate and enable */
ADI_UART_RESULT_TYPE uart_Init (void) {
  
    /* UART return code */
    ADI_UART_RESULT_TYPE uartResult;
    ADI_UART_INIT_DATA   initData;
    ADI_UART_GENERIC_SETTINGS_TYPE  Settings;
    /*
     * Initialize UART
     */
    initData.pRxBufferData = RxBuffer;
    initData.RxBufferSize = RX_BUFFER_SIZE;
    initData.pTxBufferData = TxBuffer;
    initData.TxBufferSize = TX_BUFFER_SIZE;

    /* Open UART driver */
    uartResult = adi_UART_Init(ADI_UART_DEVID_0, &hUartDevice, &initData);
    if (ADI_UART_SUCCESS != uartResult)
    {
        test_Fail("adi_UART_Init() failed");
    }

    Settings.BaudRate = ADI_UART_BAUD_115200;
    Settings.bBlockingMode = true;
    Settings.bInterruptMode = true;
    Settings.Parity = ADI_UART_PARITY_NONE;
    Settings.WordLength = ADI_UART_WLS_8;
    Settings.bDmaMode = false;
          
    /* config UART */
    uartResult =  adi_UART_SetGenericSettings(hUartDevice, &Settings);
    if (ADI_UART_SUCCESS != uartResult)
    {
        test_Fail("adi_UART_SetGenericSettings() failed");
    }

    /* enable UART */
    uartResult = adi_UART_Enable(hUartDevice, true);
    if (ADI_UART_SUCCESS != uartResult)
    {
        test_Fail("adi_UART_Enable(true) failed");
    }

    return uartResult;
}

/* Uninitialize the UART */
ADI_UART_RESULT_TYPE uart_UnInit (void) {
    ADI_UART_RESULT_TYPE    result = ADI_UART_SUCCESS;
    
    /* Uninitialize the UART API */
    if (ADI_UART_SUCCESS != (result = adi_UART_UnInit(hUartDevice)))
    {
        return result;
    }
    
    return result;
}

void delay(uint32_t count)
{
    while(count>0)
    {
        count--;
    }
}

/******************************************************************************
    Main loop for tetrapolar bioimpedance spectroscopy 

*****************************************************************************/
void bioimpedance_spectroscopy(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq) {
  
  int16_t             dft_results[DFT_RESULTS_COUNT];
  q15_t               dft_results_q15[DFT_RESULTS_COUNT];
  q31_t               dft_results_q31[DFT_RESULTS_COUNT];
  q31_t               magnitude[DFT_RESULTS_COUNT / 2];
  int8_t              j,k;    
  uint32_t            rtiaAndGain;  

  for (j = 0; j < MULTIFREQUENCY_ARRAY_SIZE; j++)   /* Here we start an outer frequency loop. */ 
  {    
    //seq_afe_fast_acmeasBioZ_4wire
    /* recalculate FCW here, based on number in array. */
    uint64_t FREQ_MOD = multifrequency[j];
    uint32_t FCW_MOD  = ((uint32_t)(((uint64_t)FREQ_MOD << 26) / 16000000 + 0.5));
    
    char                stringfrequency[MSG_MAXLEN_M2];
    sprintf(stringfrequency, "%s;", stringfreqs[j]);
    
    if (j >= MULTIFREQUENCY_ARRAY_SIZE) {  // reset J 
      j = 0;
    }
    
    /* Update FCW in the sequence */
    seq_afe_fast_acmeasBioZ_4wire[3] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_FCW, FCW_MOD);
    /* Update sine amplitude in the sequence */
    seq_afe_fast_acmeasBioZ_4wire[4] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_AMPLITUDE, SINE_AMPLITUDE);
    
    /* Recalculate CRC in software for the AC measurement, because we changed   */
    /* FCW and sine amplitude settings                                          */
    adi_AFE_EnableSoftwareCRC(hDevice, true);
    
    int max = 1; // do only 5  runs of each frequency. I am not successfully changing frequency. 

    for (k = 0; k < max ; k++)  
    {   
      
      fixed32_t           magnitude_result[DFT_RESULTS_COUNT / 2 - 1]={0};
      if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq, (uint16_t *)dft_results, DFT_RESULTS_COUNT)) 
      {
        PRINT("Impedance Measurement FAILED");
      }         
      
      /* Convert DFT results to 1.15 and 1.31 formats.  */
      convert_dft_results(dft_results, dft_results_q15, dft_results_q31);
      
      /* Magnitude calculation */
      /* Use CMSIS function */
      arm_cmplx_mag_q31(dft_results_q31, magnitude, DFT_RESULTS_COUNT / 2);
      
      /* Calculate final magnitude value, calibrated with RTIA the gain of the instrumenation amplifier */
      rtiaAndGain = (uint32_t)((RTIA * 1.5) / INST_AMP_GAIN);
      magnitude_result[0] = calculate_magnitude(magnitude[1], magnitude[0], rtiaAndGain);
      // 
      char                msg[MSG_MAXLEN_M2] = {0};
      char                tmp[300] = {0};  
      sprintf(msg, "%s:", "magnitudes");
      strcat(msg,stringfrequency);  
      sprintf_fixed32(tmp, magnitude_result[0]);
      strcat(msg,tmp);
      strcat(msg," \r\n");       
      PRINT(msg);
      
    }
  }  
}

/******************************************************************************
    Main loop for bipolar time series measurements. 

*****************************************************************************/
void time_series_bipolar(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq) {
  
      int16_t             dft_results[DFT_RESULTS_COUNT];
      q15_t               dft_results_q15[DFT_RESULTS_COUNT];
      q31_t               dft_results_q31[DFT_RESULTS_COUNT];
      q31_t               magnitude[DFT_RESULTS_COUNT / 2];
      fixed32_t           magnitude_result[DFT_RESULTS_COUNT / 2 - 1]={0};      
      char                msg[MSG_MAXLEN_M1] = {0};
      char                tmp[300] = {0};        
      int8_t              i;
          
      /* Perform the Impedance measurement */
      if (adi_AFE_RunSequence(hDevice, seq_fast_2wire_bipolar, (uint16_t *)dft_results, DFT_RESULTS_COUNT)) 
      {
        PRINT("Impedance Measurement");
      }
      /* Convert DFT results to 1.15 and 1.31 formats.  */
      convert_dft_results(dft_results, dft_results_q15, dft_results_q31);
      
      /* Magnitude calculation */
      /* Use CMSIS function */
      arm_cmplx_mag_q31(dft_results_q31, magnitude, DFT_RESULTS_COUNT / 2);
      
      /* Calculate final magnitude values, calibrated with RCAL. */
      for (i = 0; i < DFT_RESULTS_COUNT / 2 - 1; i++) 
      {
        magnitude_result[i] = calculate_bipolar_magnitude(magnitude[0], magnitude[i + 1]);
      }
      
      sprintf_fixed32(tmp, magnitude_result[0]);
      strcat(msg,tmp);
      strcat(msg," \r\n");       
      PRINT(msg);
}

/******************************************************************************
    Main loop for tetrapolar time series measurements. 

*****************************************************************************/
void time_series(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq) {
  
    int16_t             dft_results[DFT_RESULTS_COUNT];
    q15_t               dft_results_q15[DFT_RESULTS_COUNT];
    q31_t               dft_results_q31[DFT_RESULTS_COUNT];
    q31_t               magnitude[DFT_RESULTS_COUNT / 2];
    fixed32_t           magnitude_result[DFT_RESULTS_COUNT / 2 - 1]={0};
    uint32_t            rtiaAndGain;
    
    /* Calculate final magnitude value, calibrated with RTIA the gain of the instrumenation amplifier */
    rtiaAndGain = (uint32_t)((RTIA * 1.5) / INST_AMP_GAIN);      
    
    char                msg[MSG_MAXLEN_M1] = {0};
    char                tmp[300] = {0};  
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq, (uint16_t *)dft_results, DFT_RESULTS_COUNT)) 
    {
      PRINT("Impedance Measurement FAILED");
    }         
    
    /* Convert DFT results to 1.15 and 1.31 formats.  */
    convert_dft_results(dft_results, dft_results_q15, dft_results_q31);
    
    /* Magnitude calculation */
    /* Use CMSIS function */
    arm_cmplx_mag_q31(dft_results_q31, magnitude, DFT_RESULTS_COUNT / 2);
    
    /* Calculate final magnitude value, calibrated with RTIA the gain of the instrumenation amplifier */
    rtiaAndGain = (uint32_t)((RTIA * 1.5) / INST_AMP_GAIN);
    magnitude_result[0] = calculate_magnitude(magnitude[1], magnitude[0], rtiaAndGain);
    sprintf_fixed32(tmp, magnitude_result[0]);
    strcat(msg,tmp);
    strcat(msg," \r\n");       
    PRINT(msg);  
  
}


/******************************************************************************
    Main code for the imaging function with 32 electrodes. 
  
*****************************************************************************/
void multiplex_adg732(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq,uint32_t n_el) {
  
   // int 32 of the sequence, no of measures based on the sequence entered. 
   // i.e. n_el if 8, 16, 32, we can pick which sequence. 
   // 
    uint32_t            numberofmeasures;
    if (n_el == 8) {
      numberofmeasures = 48;
    }  
    else if (n_el == 16) {
      numberofmeasures = 224;
    }
    else if (n_el == 32) {
      numberofmeasures = 960;
    }
    else {
      numberofmeasures = 0;
      PRINT("number of measures is 0\n");
    }
    
    uint32_t            rtiaAndGain;
    /* Calculate final magnitude value, calibrated with RTIA the gain of the instrumenation amplifier */
    rtiaAndGain = (uint32_t)((RTIA * 1.5) / INST_AMP_GAIN);
      
    PRINT("magnitudes: ");
    
    /* Enable software CRC so the driver dynamically calculates the CRC of the sequences */
    adi_AFE_EnableSoftwareCRC(hDevice, true);
    
    q31_t cached_rcal_mag = 0;
    int16_t adc_samples[160] = {0};
    
    // NUMBEROFMEASURES is determined by which electrode configuration: 8,16 or 32. 
    for (uint32_t econf = 0;econf<numberofmeasures;econf++) {    
                
      char                tmp[300] = {0};      
      fixed32_t           magnitude_result[1] = {0};
      
      // This is where we select the electrode sequence. i.e. 8,16 or 32 adjacent or opposition.  
      int16_t* e;
      if (n_el == 8) {
        e = (int16_t *)electrode_configuration_8_opposition[econf];
      }  
      else if (n_el == 16) {
        e = (int16_t *)electrode_configuration_16_opposition[econf];
      }
      else if (n_el == 32) {
        e = (int16_t *)electrode_configuration_32_opposition[econf];
      }
      else {
        e = (int16_t *)electrode_configuration_32_opposition[econf];
      }
      
      multiplex_adg732_fast(e); 
      
      if (econf == 0 || cached_rcal_mag == 0) {
        /* 1. Measure TIA (RCAL) raw samples */
        seq_afe_raw_adc_only_4wire[1] = 0xA0000002; /* TIA channel */
        if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_raw_adc_only_4wire, (uint16_t *)adc_samples, 160)) 
        {
          PRINT("FAILED TIA Measurement");
        }
        cached_rcal_mag = compute_goertzel_magnitude(adc_samples, 160, 50000.0f, 160000.0f);
      }
      
      /* 2. Measure AN_A (electrode) raw samples */
      seq_afe_raw_adc_only_4wire[1] = 0xA0000208; /* AN_A channel */
      if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_raw_adc_only_4wire, (uint16_t *)adc_samples, 160)) 
      {
        PRINT("FAILED Impedance Measurement");
      }
      q31_t measured_mag = compute_goertzel_magnitude(adc_samples, 160, 50000.0f, 160000.0f);
      
      magnitude_result[0] = calculate_magnitude(measured_mag, cached_rcal_mag, rtiaAndGain);
        
      sprintf_fixed32(tmp, magnitude_result[0]);
      strcat(tmp,",");
      
      PRINT(tmp);
    } // END  e_config for loop. 
    
    /* Restore to default behavior (Software CRC disabled) */
    adi_AFE_EnableSoftwareCRC(hDevice, false);
    
    PRINT("\r\n"); 
    adi_UART_BufFlush(hUartDevice);
}
/******************************************************************************
    Main code for the imaging function bipolar measurements(not tetrapolar). 
  
*****************************************************************************/
void bipolar_adg732(ADI_AFE_DEV_HANDLE  hDevice, const uint32_t *const seq,uint32_t n_el) {
  
   // int 32 of the sequence, no of measures based on the sequence entered. 
   // i.e. n_el if 8, 16, 32, we can pick which sequence. 
   // 
    uint32_t            numberofmeasures;
    if (n_el == 8) {
      numberofmeasures = 48;
    }  
    else if (n_el == 16) {
      numberofmeasures = 224;
    }
    else if (n_el == 32) {
      numberofmeasures = 960;
    }
    else {
      numberofmeasures = 0;
      PRINT("number of measures is 0\n");
    }

    PRINT("magnitudes: ");
    
    /* Enable software CRC so the driver dynamically calculates the CRC of the sequences */
    adi_AFE_EnableSoftwareCRC(hDevice, true);
    
    q31_t cached_rcal_mag = 0;
    int16_t adc_samples[160] = {0};
    
    // NUMBEROFMEASURES is determined by which electrode configuration: 8,16 or 32. 
    for (uint32_t econf = 0;econf<numberofmeasures;econf++) {    
                
      char                tmp[300] = {0};      
      fixed32_t           magnitude_result[1] = {0};
      
      // This is where we select the electrode sequence. i.e. 8,16 or 32 adjacent or opposition.  
      int16_t* e;
      if (n_el == 8) {
        e = (int16_t *)electrode_configuration_8_opposition[econf];
      }  
      else if (n_el == 16) {
        e = (int16_t *)electrode_configuration_16_opposition[econf];
      }
      else if (n_el == 32) {
        e = (int16_t *)electrode_configuration_32_opposition[econf];
      }
      else {
        e = (int16_t *)electrode_configuration_32_opposition[econf];
      }
      
      multiplex_adg732_fast(e); 
      
      if (econf == 0 || cached_rcal_mag == 0) {
        /* 1. Measure TIA (RCAL) raw samples */
        seq_afe_raw_adc_only_4wire[1] = 0xA0000002; /* TIA channel */
        if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_raw_adc_only_4wire, (uint16_t *)adc_samples, 160)) 
        {
          PRINT("FAILED TIA Measurement");
        }
        cached_rcal_mag = compute_goertzel_magnitude(adc_samples, 160, 50000.0f, 160000.0f);
      }
      
      /* 2. Measure AN_A (electrode) raw samples */
      seq_afe_raw_adc_only_4wire[1] = 0xA0000208; /* AN_A channel */
      if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_raw_adc_only_4wire, (uint16_t *)adc_samples, 160)) 
      {
        PRINT("FAILED Impedance Measurement");
      }
      q31_t measured_mag = compute_goertzel_magnitude(adc_samples, 160, 50000.0f, 160000.0f);
      
      magnitude_result[0] = calculate_bipolar_magnitude(cached_rcal_mag, measured_mag);
        
      sprintf_fixed32(tmp, magnitude_result[0]);
      strcat(tmp,",");
      
      PRINT(tmp);
    } // END  e_config for loop. 
    
    /* Restore to default behavior (Software CRC disabled) */
    adi_AFE_EnableSoftwareCRC(hDevice, false);
    
    PRINT("\r\n"); 
    adi_UART_BufFlush(hUartDevice);
}
/***************************

    This initializes bipolar measurement parameters

****************************/
void init_mode_bipolar(void) {
  
        /* Initialize GPIO */
    if (ADI_GPIO_SUCCESS != adi_GPIO_Init())
    {
        PRINT("adi_GPIO_Init");
    }
    
    /* Enable GPIO output drivers */
    init_GPIO_ports();
        
    /* Initialize the AFE API */
    if (adi_AFE_Init(&hDevice)) 
    {
        PRINT("adi_AFE_Init");
    }

    /* AFE power up */
    if (adi_AFE_PowerUp(hDevice)) 
    {
        PRINT("adi_AFE_PowerUp");
    }

    /* Excitation Channel Power-Up */
    if (adi_AFE_ExciteChanPowerUp(hDevice)) 
    {
        PRINT("adi_AFE_ExciteChanPowerUp");
    }

    /* TIA Channel Calibration */
    if (adi_AFE_TiaChanCal(hDevice)) 
    {
        PRINT("adi_AFE_TiaChanCal");
    }

    /* Excitation Channel Calibration (Attenuation Enabled) */
    if (adi_AFE_ExciteChanCalAtten(hDevice)) 
    {
        PRINT("adi_AFE_ExciteChanCalAtten");
    }

    /* Update FCW in the sequence */
     seq_afe_poweritup[3] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_FCW, FCW);
     /* Update sine amplitude in the sequence */
     seq_afe_poweritup[4] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_AMPLITUDE, SINE_AMPLITUDE_BIPOLAR);

    /* Recalculate CRC in software for the AC measurement, because we changed   */
    /* FCW and sine amplitude settings                                          */
    adi_AFE_EnableSoftwareCRC(hDevice, true);
    int16_t             dft_results[DFT_RESULTS_COUNT];
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_poweritup_bipolar, (uint16_t *)dft_results, DFT_RESULTS_COUNT)) 
    {
      PRINT("AFE PROBLEM! ");
    }          
    PRINT("FINISHED SET UP \n");
    
  
};
/***************************

    This initializes bioimpedance spectroscopy parameters. 

****************************/
void init_mode_bis(void) {
  
   uint32_t            offset_code;
   uint32_t            gain_code;       
  
    /* Initialize the AFE API */
    if (ADI_AFE_SUCCESS != adi_AFE_Init(&hDevice)) 
    {
        FAIL("Init");
    }

    /* Set RCAL and RTIA values */
    if (ADI_AFE_SUCCESS != adi_AFE_SetRcal(hDevice, RCAL)) 
    {
        FAIL("adi_AFE_SetRcal");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_SetRtia(hDevice, RTIA)) 
    {
        FAIL("adi_AFE_SetTia");
    }

    /* AFE power up */
    if (ADI_AFE_SUCCESS != adi_AFE_PowerUp(hDevice)) 
    {
        FAIL("adi_AFE_PowerUp");
    }

    /* Delay to ensure Vbias is stable */
    delay(2000000);                                                             

    /* Temp Channel Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_TempSensChanCal(hDevice)) 
    {
        PRINT("adi_AFE_TempSensChanCal");
    }
    
    /* Auxiliary Channel Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_AuxChanCal(hDevice)) 
    {
        PRINT("adi_AFE_AuxChanCal");
    }    

    /* Excitation Channel Power-Up */
    if (ADI_AFE_SUCCESS != adi_AFE_ExciteChanPowerUp(hDevice)) 
    {
        FAIL("adi_AFE_ExciteChanPowerUp");
    }

    /* TempCal results will be used to set the TIA calibration registers. These */
    /* values will ensure the ratio between current and voltage is exactly 1.5  */
    if (ADI_AFE_SUCCESS != adi_AFE_ReadCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_GAIN_TEMP_SENS, &gain_code))
    {
        FAIL("adi_AFE_ReadCalibrationRegister, gain");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_WriteCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_GAIN_TIA, gain_code))
    {
        FAIL("adi_AFE_WriteCalibrationRegister, gain");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_ReadCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_OFFSET_TEMP_SENS, &offset_code))
    {
        FAIL("adi_AFE_ReadCalibrationRegister, offset");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_WriteCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_OFFSET_TIA, offset_code))
    {
        FAIL("adi_AFE_WriteCalibrationRegister, offset");
    }    
  
  
}
/***************************
    This initializes 32 electrode imaging parameters. 

****************************/
void init_mode_tetramux(void) {

   uint32_t            offset_code;
   uint32_t            gain_code;     

    /* Initialize GPIO */
    if (ADI_GPIO_SUCCESS != adi_GPIO_Init())
    {
        PRINT("adi_GPIO_Init");
    }
    
    /* Enable GPIO output drivers */
    init_GPIO_ports();
        
    /* Initialize the AFE API */
    if (ADI_AFE_SUCCESS != adi_AFE_Init(&hDevice)) 
    {
        PRINT("Init");
    }

    /* Set RCAL and RTIA values */
    if (ADI_AFE_SUCCESS != adi_AFE_SetRcal(hDevice, RCAL)) 
    {
        PRINT("adi_AFE_SetRcal");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_SetRtia(hDevice, RTIA)) 
    {
        PRINT("adi_AFE_SetTia");
    }

    /* AFE power up */
    if (ADI_AFE_SUCCESS != adi_AFE_PowerUp(hDevice)) 
    {
        PRINT("adi_AFE_PowerUp");
    }

    /* Delay to ensure Vbias is stable */
    delay(2000000);                                                             

    // This writes into some registers // 
    /* Temp Channel Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_TempSensChanCal(hDevice)) 
    {
        PRINT("adi_AFE_TempSensChanCal");
    }
    
    /* Auxiliary Channel Calibration */
    if (ADI_AFE_SUCCESS != adi_AFE_AuxChanCal(hDevice)) 
    {
        PRINT("adi_AFE_AuxChanCal");
    }    

    /* Excitation Channel Power-Up */
    if (ADI_AFE_SUCCESS != adi_AFE_ExciteChanPowerUp(hDevice)) 
    {
        PRINT("adi_AFE_ExciteChanPowerUp");
    }
    
    /* TempCal results will be used to set the TIA calibration registers. These */
    /* values will ensure the ratio between current and voltage is exactly 1.5  */
    if (ADI_AFE_SUCCESS != adi_AFE_ReadCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_GAIN_TEMP_SENS, &gain_code))
    {
        PRINT("adi_AFE_ReadCalibrationRegister, gain");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_WriteCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_GAIN_TIA, gain_code))
    {
        PRINT("adi_AFE_WriteCalibrationRegister, gain");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_ReadCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_OFFSET_TEMP_SENS, &offset_code))
    {
        PRINT("adi_AFE_ReadCalibrationRegister, offset");
    }
    if (ADI_AFE_SUCCESS != adi_AFE_WriteCalibrationRegister(hDevice, ADI_AFE_CAL_REG_ADC_OFFSET_TIA, offset_code))
    {
        PRINT("adi_AFE_WriteCalibrationRegister, offset");
    }

    /* Update FCW in the sequence */
    seq_afe_poweritup[3] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_FCW, FCW);
    /* Update sine amplitude in the sequence */
    seq_afe_poweritup[4] = SEQ_MMR_WRITE(REG_AFE_AFE_WG_AMPLITUDE, SINE_AMPLITUDE);
      
    /* Recalculate CRC in software for the AC measurement, because we changed   */
    /* FCW and sine amplitude settings.                                         */
    adi_AFE_EnableSoftwareCRC(hDevice, true);
    int16_t             dft_results[DFT_RESULTS_COUNT]     = {0};
    if (ADI_AFE_SUCCESS != adi_AFE_RunSequence(hDevice, seq_afe_poweritup, (uint16_t *)dft_results, DFT_RESULTS_COUNT)) 
    {
     PRINT("AFE PROBLEM!");
    }          
  
}

void init_GPIO_ports(void) {
  
    // Enable all 4 multiplexer ports.  
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (ADI_GPIO_PORT_1,ADI_GPIO_PIN_5, false))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(ADI_GPIO_PORT_1,ADI_GPIO_PIN_5, true))
    {
        PRINT("adi_GPIO_SetOutputEnable (1.5) FAILED ");
    }
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(ADI_GPIO_PORT_1,ADI_GPIO_PIN_5, true))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }    

        
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (ADI_GPIO_PORT_1,ADI_GPIO_PIN_11, false))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }    
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(ADI_GPIO_PORT_1,ADI_GPIO_PIN_11, true))
    {
        PRINT("adi_GPIO_SetOutputEnable (1.11) FAILED");
    }    
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(ADI_GPIO_PORT_1,ADI_GPIO_PIN_11, true))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }        
    
    
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (ADI_GPIO_PORT_3,ADI_GPIO_PIN_12, false))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(ADI_GPIO_PORT_3,ADI_GPIO_PIN_12, true))
    {
        PRINT("adi_GPIO_SetOutputEnable (3.12) FAILED ");
    }
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(ADI_GPIO_PORT_3,ADI_GPIO_PIN_12, true))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }         
    
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (ADI_GPIO_PORT_2,ADI_GPIO_PIN_6, false))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(ADI_GPIO_PORT_2,ADI_GPIO_PIN_6, true))
    {
        PRINT("adi_GPIO_SetOutputEnable (2.6) FAILED");
    }        
    if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(ADI_GPIO_PORT_2,ADI_GPIO_PIN_6, true))
    {
      PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
    }         
    
    PinMap m1_portpin;  
    PinMap m2_portpin;  
    PinMap m3_portpin;  
    PinMap m4_portpin;  
    char                porttest[MSG_MAXLEN_M3] = {0};
    sprintf(porttest, "%s:", "port test failure list: ");     
    
    /* with error messages for every single  GPIO enable */ 
    for (int i=0;i<5;i++) {   
      
        char    ptmp[50]={0};
        m1_portpin = m1_configuration[i]; // first one is a4,a3,a2,a1,a0. 
        m2_portpin = m2_configuration[i]; // second one is        
        m3_portpin = m3_configuration[i]; // third one is          
        m4_portpin = m4_configuration[i]; // fourth one is 
        
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (m1_portpin.Port, m1_portpin.Pins, false)) {
          PRINT ("Problem with setting pull up enable");
        }
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(m1_portpin.Port, m1_portpin.Pins, true))
        {
          sprintf(ptmp,"%d . %d, ", m1_portpin.Port, m1_portpin.Pins);
          strcat(porttest,ptmp);
        }              
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(m1_portpin.Port, m1_portpin.Pins, true))
        {
          PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
        }
  
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (m2_portpin.Port, m2_portpin.Pins, false)) {
          PRINT ("Problem with setting pull up enable");
        }        
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(m2_portpin.Port, m2_portpin.Pins, true))
        {
          sprintf(ptmp,"%d . %d, ", m2_portpin.Port, m2_portpin.Pins);
          strcat(porttest,ptmp);
        }     
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(m2_portpin.Port, m2_portpin.Pins, true))
        {
          PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
        }        
        
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (m3_portpin.Port, m3_portpin.Pins, false)) {
          PRINT ("Problem with setting pull up enable");
        }
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(m3_portpin.Port, m3_portpin.Pins, true))
        {
          sprintf(ptmp,"%d . %d, ", m3_portpin.Port, m3_portpin.Pins);
          strcat(porttest,ptmp);
        }               
                if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(m3_portpin.Port, m3_portpin.Pins, true))
        {
          PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
        }
        
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetPullUpEnable (m4_portpin.Port, m4_portpin.Pins, false)) {
          PRINT ("Problem with setting pull up enable");
        }
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetOutputEnable(m4_portpin.Port, m4_portpin.Pins, true))
        {
          sprintf(ptmp,"%d . %d, ", m4_portpin.Port, m4_portpin.Pins);
          strcat(porttest,ptmp);
        }            
        if (ADI_GPIO_SUCCESS != adi_GPIO_SetInputEnable(m4_portpin.Port, m4_portpin.Pins, true))
        {
          PRINT("adi_GPIO_SetInputEnable (led_DISPLAY)");
        }        
        
     }
     PRINT(porttest);
}


//#if defined ( __ICCARM__ )  // IAR compiler...
///* Revert ADI MISRA Suppressions */
//#define REVERT_ADI_MISRA_SUPPRESSIONS
//#include "misra.h"
//#endif
