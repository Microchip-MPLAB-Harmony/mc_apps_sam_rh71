/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    mc_app.c

  Summary:
    This file contains the sensored FOC motor control algorithm.

  Description:
    This file contains the sensored FOC motor control algorithm
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2020 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "definitions.h"                // SYS function prototypes
#include "mc_app.h"
#include "userparams.h"
#include "CMSIS/Core/Include/core_cm7.h"
#include "X2Cscope.h"
#include "math.h"

/******************************************************************************/
/* Defines                                                                    */
/******************************************************************************/
/* Define the number of slow loop to wait before stopping the motor if there was no activity */
#define MOTOR_ACTIVITY_SLOW_LOOP_COUNT_60_SEC  (12000U)
#define NOP() asm("NOP");
/******************************************************************************/
/* Local Function Prototype                                                   */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorAngleCalc(void);
__STATIC_INLINE void MCAPP_MotorCurrentControl( void );
static void MCAPP_ADCOffsetCalibration(void);
static void MCAPP_MotorControlParamInit(void);
__STATIC_INLINE bool MCAPP_SlowLoopTimeIsFinished(void);
__STATIC_INLINE void MCAPP_SlowControlLoop(void);
static void MCAPP_SwitchStartDebounce(MC_APP_STATE state);
static void MCAPP_SwitchDecrDebounce(void);
static void MCAPP_SwitchIncrDebounce(void);

#if(TORQUE_MODE == false)
__STATIC_INLINE void MCAPP_SpeedRamp(void);
#endif

/******************************************************************************/
/*                   Structures                                               */
/******************************************************************************/
/*  Motor control algorithm control structure  */
static MCAPP_CONTROL_PARAM gCtrlParam;
static MCAPP_FOC_PARAM gfocParam;
static MCAPP_DATA gMCAPPData;
static MCAPP_POSITION_CALC gPositionCalc;

/******************************************************************************/
/*                   Global Variables                                         */
/******************************************************************************/
static float speed_ref_filtered = 0.0f;
static uint32_t phaseCurrentUOffset;
static uint32_t phaseCurrentVOffset;

/* Global variables for Decimation Filters for channel U and V */
static volatile __attribute__ ((tcm)) MCAPP_SINC3 gCurrentU = {0};
static volatile __attribute__ ((tcm)) MCAPP_SINC3 gCurrentV = {0};
static volatile __attribute__ ((tcm)) uint32_t sinc3_count = 0;
static volatile __attribute__ ((tcm)) uint32_t currentUActive = 0;
static volatile __attribute__ ((tcm)) uint32_t currentVActive = 0;
static volatile uint32_t sinc3_out_sample_count = 0U;

/* Encoder last measure of speed in electrical rad per sec */
static volatile float speed_elec_rad_per_sec;

/* Motor speed target in electrical rad per sec */
static float motor_speed_target_elec_rad_per_sec = 400.0f;

/* Count of slow loop since last user activity */
static uint32_t motor_activity_count = 0U;

static uintptr_t dummyforMisra;
/*****************ISR Functions *******************************/

/******************************************************************************/
/* Function name: MCAPP_MotorCurrentControl                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Executes one PI iteration for each of the three loops Id,Iq,Speed          */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorCurrentControl( void )
{
    if( 0U == gCtrlParam.fieldAlignmentFlag )
    {
        /* switch to close loop */
        gCtrlParam.changeMode = false;
        gCtrlParam.openLoop = false;
    }
    
    if( gCtrlParam.openLoop == true )
    {
        /* OPENLOOP:  force rotating angle,Vd,Vq */
        if( gCtrlParam.changeMode == true )
        {
            /* just changed to open loop */
            gCtrlParam.changeMode = false;

            /* IqRef & IdRef not used */
            gCtrlParam.iqRef = 0.0f;
            gCtrlParam.idRef = 0.0f;

            /* re-init vars for initial speed ramp */
            gCtrlParam.startup_lock_count = 0U;
            gCtrlParam.startup_angle_ramp_rads_per_sec = 0.0f;
        }

        /* q current reference is equal to the velocity reference
         * while d current reference is equal to 0
         * for maximum startup torque, set the q current to maximum acceptable
         * value represents the maximum peak value 	 */

        gCtrlParam.iqRef = Q_CURRENT_REF_OPENLOOP*(float)gCtrlParam.direction;

        /* PI control for Iq torque control loop */
        gPIParmQ.inMeas = gMCLIBCurrentDQ.iq;
        gPIParmQ.inRef  = gCtrlParam.iqRef;
        MCLIB_PIControl(&gPIParmQ);
        gMCLIBVoltageDQ.vq    = gPIParmQ.out;

        /* PI control for Id flux control loop */
        gPIParmD.inMeas = gMCLIBCurrentDQ.id;
        gPIParmD.inRef  = gCtrlParam.idRef;
        MCLIB_PIControl(&gPIParmD);
        gMCLIBVoltageDQ.vd = gPIParmD.out;
    }
    else
    {
        /* Closed Loop Vector Control */
        if( gCtrlParam.changeMode == true )
        {
            /* Just switched from open loop to close loop */
            gCtrlParam.changeMode = false;
            /* Load velocity control loop with Iq reference for smooth transition */
            gPIParmQref.dSum = 0.0f;
            gPIParmD.inRef = 0.0f;
            gCtrlParam.idRef = 0.0f;
            gCtrlParam.sync_cnt = 0U;

            // Set default target speed rad/sec
            motor_speed_target_elec_rad_per_sec = 400.0f;
        }

#if(TORQUE_MODE == true)
        /* If TORQUE MODE is enabled then skip the velocity control loop */
        gCtrlParam.iqRef = Q_CURRENT_REF_OPENLOOP*gCtrlParam.direction;
#endif

        /* PI control for Id flux control loop */
        gPIParmD.inMeas = gMCLIBCurrentDQ.id;          /* This is in Amps */
        gPIParmD.inRef  = gCtrlParam.idRef;       /* This is in Amps */
        MCLIB_PIControl(&gPIParmD);

        gfocParam.lastVd = gMCLIBVoltageDQ.vd;
        gMCLIBVoltageDQ.vd    = gPIParmD.out;          /* This is in %. It should be converted to volts, multiply with (DC/sqrt(3)) */

        /* PI control for Iq torque control */
        gPIParmQ.inMeas = gMCLIBCurrentDQ.iq;          /* This is in Amps */
        gPIParmQ.inRef  = gCtrlParam.iqRef;       /* This is in Amps */
        MCLIB_PIControl(&gPIParmQ);
        /* This is in %. If should be converted to volts, multiply with (VDC/sqrt(3))  */
        gMCLIBVoltageDQ.vq    = gPIParmQ.out;
    }
}

/******************************************************************************/
/* Function name: MCAPP_MotorAngleCalc                                        */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Generate the start sin waves feeding the motor's terminals    */
/* in open loop control, forcing the motor to align and to start speeding up  */
/******************************************************************************/
__STATIC_INLINE void MCAPP_MotorAngleCalc(void)
{
    
    if(gCtrlParam.openLoop == true)
    {
        /* begin with the lock sequence, for field alignment */
        if (gCtrlParam.startup_lock_count < (uint32_t)LOCK_COUNT_FOR_LOCK_TIME)
        {
            gCtrlParam.startup_lock_count++;
            gPositionCalc.rotor_angle_rad_per_sec = (float)(M_PI);
        }
        else
        {
            if(gCtrlParam.startup_lock_count < 2U*(uint32_t)LOCK_COUNT_FOR_LOCK_TIME)
            {
                gCtrlParam.startup_lock_count++;
                gPositionCalc.rotor_angle_rad_per_sec = ((float)M_PI + ((float)M_PI_2 * (float)gCtrlParam.direction));
            }
            else
            {
                /* Reset field alignment Flag */
                gCtrlParam.fieldAlignmentFlag = 0U;
                
                /* switch to close loop */
                gCtrlParam.changeMode = true;
                gCtrlParam.openLoop = false;
                /* Start QDEC timer */
                TC1_QuadratureStart();
                gPositionCalc.QDECcntZ = 0u;
                gPositionCalc.prev_position_count=0;
                gPositionCalc.posCompensation = 0u;
                speed_ref_filtered=0.0f;
                /* the angle set after alignment */
                gPositionCalc.rotor_angle_rad_per_sec = 0.0f;
                gCtrlParam.open_loop_stab_counter = 0U;
            }
        }
    }
    else
    {
        /* Switched to closed loop..*/
        gPositionCalc.QDECcnt = (uint16_t)((TC1_REGS->TC_CHANNEL[0].TC_CV)& 0xFFFFu);        
        if((gPositionCalc.QDECcnt>QDEC_UPPER_THRESHOLD) && (gPositionCalc.QDECcntZ<QDEC_LOWER_THRESHOLD))
        {
            gPositionCalc.posCompensation += QDEC_UNDERFLOW;
        } else if((gPositionCalc.QDECcntZ>QDEC_UPPER_THRESHOLD) && (gPositionCalc.QDECcnt<QDEC_LOWER_THRESHOLD))
        {
            gPositionCalc.posCompensation += QDEC_OVERFLOW;           
        } else{ 
            /* No Operation*/
        }        
        
        gPositionCalc.posCompensation = gPositionCalc.posCompensation % ENCODER_PULSES_PER_EREV;
        gPositionCalc.posCntTmp = (uint32_t)((uint32_t)gPositionCalc.QDECcnt + (uint32_t)gPositionCalc.posCompensation);  
        gPositionCalc.posCnt =(uint16_t) (gPositionCalc.posCntTmp % ENCODER_PULSES_PER_EREV);
        gPositionCalc.rotor_angle_rad_per_sec = ((float)gPositionCalc.posCnt) * (2.0f * (float)M_PI / (float)ENCODER_PULSES_PER_EREV);
        gPositionCalc.QDECcntZ = gPositionCalc.QDECcnt;
    }

    /* Limit rotor angle range to 0 to 2*M_PI for lookup table */
    if(gPositionCalc.rotor_angle_rad_per_sec > (2.0f*(float)M_PI))
    {
      gfocParam.angle = gPositionCalc.rotor_angle_rad_per_sec - (2.0f*(float)M_PI);
    }
    else if(gPositionCalc.rotor_angle_rad_per_sec < 0.0f)
    {
      gfocParam.angle = 2.0f*(float)M_PI + gPositionCalc.rotor_angle_rad_per_sec;
    }
    else
    {
      gfocParam.angle = gPositionCalc.rotor_angle_rad_per_sec;
    }
}

/******************************************************************************/
/* Function name: MCAPP_ADCOffsetCalibration                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Measures motor phase current offsets.Limits the offset if it  */
/*              exceeds the predetermined range.                              */
/******************************************************************************/

static void MCAPP_ADCOffsetCalibration(void)
{
    uint32_t AdcSampleCounter = 0u;
    uint32_t phaseUOffsetBuffer = 0u;
    uint32_t phaseVOffsetBuffer = 0u;

    for(AdcSampleCounter = 0u; AdcSampleCounter < CURRENTS_OFFSET_SAMPLES; AdcSampleCounter++)
    {
        /* Wait next sample */
        uint32_t sample = sinc3_out_sample_count;
        do
        {
            NOP();
        } while (sinc3_out_sample_count == sample);

        phaseUOffsetBuffer += gCurrentU.sinc3_out;
        phaseVOffsetBuffer += gCurrentV.sinc3_out;
    }

    phaseCurrentUOffset = phaseUOffsetBuffer/CURRENTS_OFFSET_SAMPLES;
    phaseCurrentVOffset = phaseVOffsetBuffer/CURRENTS_OFFSET_SAMPLES;

    /* Limit motor phase A current offset calibration to configured Min/Max levels. */
    if(phaseCurrentUOffset >  CURRENT_OFFSET_MAX)
    {
        phaseCurrentUOffset = CURRENT_OFFSET_MAX;
    }
    else if(phaseCurrentUOffset <  CURRENT_OFFSET_MIN)
    {
        phaseCurrentUOffset = CURRENT_OFFSET_MIN;
    }
    else
    {
        /* No Operation*/
    }

    /* Limit motor phase B current offset calibration to configured Min/Max levels. */
    if(phaseCurrentVOffset >  CURRENT_OFFSET_MAX)
    {
        phaseCurrentVOffset = CURRENT_OFFSET_MAX;
    }
    else if(phaseCurrentVOffset <  CURRENT_OFFSET_MIN)
    {
        phaseCurrentVOffset = CURRENT_OFFSET_MIN;
    }
    else
    {
        /* No Operation*/
    }
}

/******************************************************************************/
/* Function name: MCAPP_PIOutputInit                                          */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialize PI structure                                       */
/******************************************************************************/
void MCAPP_PIOutputInit( MCLIB_PI *pParm)
{
    pParm->dSum = 0.0f;
    pParm->out = 0.0f;
}

/******************************************************************************/
/* Function name: MCAPP_MotorPIParamInit                                      */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description:                                                               */
/* Initialize control parameters: PI coefficients for D,Q and                 */
/* Velocity control loop.                                                     */
/******************************************************************************/
void MCAPP_MotorPIParamInit(void)
{
    /**************** PI D Term ***********************************************/
    gPIParmD.kp = D_CURRCNTR_PTERM;
    gPIParmD.ki = D_CURRCNTR_ITERM;
    gPIParmD.kc = D_CURRCNTR_CTERM;
    gPIParmD.outMax = D_CURRCNTR_OUTMAX;
    gPIParmD.outMin = -D_CURRCNTR_OUTMAX;

    MCAPP_PIOutputInit(&gPIParmD);

    /**************** PI Q Term ************************************************/
    gPIParmQ.kp = Q_CURRCNTR_PTERM;
    gPIParmQ.ki = Q_CURRCNTR_ITERM;
    gPIParmQ.kc = Q_CURRCNTR_CTERM;
    gPIParmQ.outMax = Q_CURRCNTR_OUTMAX;
    gPIParmQ.outMin = -Q_CURRCNTR_OUTMAX;

    MCAPP_PIOutputInit(&gPIParmQ);

    /**************** PI Velocity Control **************************************/
    gPIParmQref.kp = SPEEDCNTR_PTERM;
    gPIParmQref.ki = SPEEDCNTR_ITERM;
    gPIParmQref.kc = SPEEDCNTR_CTERM;
    gPIParmQref.outMax = SPEEDCNTR_OUTMAX;
    gPIParmQref.outMin = -SPEEDCNTR_OUTMAX;

    MCAPP_PIOutputInit(&gPIParmQref);
}

/******************************************************************************/
/* Function name: MCAPP_MotorControlParamInit                                 */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Initialize control loop variables.                            */
/******************************************************************************/
static void MCAPP_MotorControlParamInit(void)
{
    /* Parameter initialization for FOC */
    MCAPP_MotorPIParamInit();

    gCtrlParam.openLoop
            = true;
    gCtrlParam.changeMode = false;
    gCtrlParam.startup_lock_count = 0U;
    gCtrlParam.open_loop_stab_counter = 0U;
	gCtrlParam.startup_angle_ramp_rads_per_sec = 0.0f;
    gMCLIBPosition.angle = 0.0f;
    gMCLIBSVPWM.period = MAX_DUTY;
    gCtrlParam.motorStatus = MOTOR_STATUS_STOPPED;
    gMCLIBCurrentDQ.id = 0.0f;
    gMCLIBCurrentDQ.iq = 0.0f;
    gCtrlParam.rampIncStep = SPEED_RAMP_INC_SLOW_LOOP;
    gCtrlParam.velRef = 0.0f;
    MCAPP_PIOutputInit(&gPIParmD);
    MCAPP_PIOutputInit(&gPIParmQ);
    MCAPP_PIOutputInit(&gPIParmQref);

    gPositionCalc.rotor_angle_rad_per_sec = 0.0f;
    gPositionCalc.elec_rotation_count = 0U;
    gPositionCalc.prev_position_count = 0;
    gPositionCalc.present_position_count = 0;
    speed_ref_filtered = 0.0f;
}

/******************************************************************************/
/* Function name: MCAPP_SlowLoopTimeIsFinished                                */
/* Function parameters: None                                                  */
/* Function return: Bool( True - executes slow control loop )                 */
/* Description: To be used in a while loop; It returns true                   */
/******************************************************************************/
__STATIC_INLINE bool MCAPP_SlowLoopTimeIsFinished(void)
{
    bool retval = false;
    if(SLOW_LOOP_TIME_PWM_COUNT <= gCtrlParam.sync_cnt)
    {
        gCtrlParam.sync_cnt = 0U;
        retval = true;
    }
    return( retval );
}

#if(TORQUE_MODE == false)
/******************************************************************************/
/* Function name: MCAPP_Spee                                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Increment/Decrements speed reference based upon ramp          */
/*              configuration in "userparams.h" file.                         */
/******************************************************************************/
__STATIC_INLINE void MCAPP_SpeedRamp(void)
{
   if(gCtrlParam.endSpeed > (gCtrlParam.velRef + gCtrlParam.rampIncStep))
   {
      gCtrlParam.velRef += gCtrlParam.rampIncStep;
   }
   else if(gCtrlParam.endSpeed < (gCtrlParam.velRef - gCtrlParam.rampIncStep))
   {
      gCtrlParam.velRef -= gCtrlParam.rampIncStep;
   }
   else
   {
      gCtrlParam.velRef = gCtrlParam.endSpeed;
   }
}
#endif

/******************************************************************************/
/* Function name: MCAPP_PWMDutyUpdate                                         */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Configure duty cycle of WPM channels based on control loop    */
/******************************************************************************/
__STATIC_INLINE void MCAPP_PWMDutyUpdate(uint32_t duty_PhU, uint32_t duty_PhV, uint32_t duty_PhW)
{
    PWM0_ChannelDutySet(PWM_CHANNEL_0, (uint16_t)duty_PhU);
    PWM0_ChannelDutySet(PWM_CHANNEL_1, (uint16_t)duty_PhV);
    PWM0_ChannelDutySet(PWM_CHANNEL_2, (uint16_t)duty_PhW);
}

/******************************************************************************/
/* Function name: MCAPP_ControlLoopISR                                        */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: TC interrupt is used for executing fast                       */
/*              control loop. Angle is obtained from quadrature encoder.      */
/* ISR Timings -                                                              */
/*               TBD                                                          */
/******************************************************************************/

void MCAPP_ControlLoopISR(TC_COMPARE_STATUS status, uintptr_t context)
{    
    float phaseCurrentU;
    float phaseCurrentV;
    float temp;
    X2Cscope_Update();
   
   /* PB17 GPIO is used for timing measurement. - Set High*/
    PIOB_REGS->PIO_SODR = (uint32_t)((uint32_t)1U << (17U & 0x1FU));

 	/* Weight average on 4 last samples */
    temp = 2.0f * (float)gCurrentU.sinc3_out;
    temp += 4.0f * (float)gCurrentU.sinc3_out_p;
    temp += 3.0f * (float)gCurrentU.sinc3_out_pp;
    temp += (float)gCurrentU.sinc3_out_ppp;
    phaseCurrentU = ((float)temp / 10.0f);
	
    temp = 2.0f * (float)gCurrentV.sinc3_out;
    temp +=4.0f * (float)gCurrentV.sinc3_out_p;
    temp +=3.0f * (float)gCurrentV.sinc3_out_pp;
    temp += (float)gCurrentV.sinc3_out_ppp;
    phaseCurrentV = ((float)temp / 10.0f);

    /* Remove the offset from measured motor currents */
    phaseCurrentU = phaseCurrentU - (float)(phaseCurrentUOffset);
    phaseCurrentV = phaseCurrentV - (float)(phaseCurrentVOffset);

    /* Non Inverting amplifiers for current sensing */
    gMCLIBCurrentABC.ia  = phaseCurrentU * (float)(0.000112);
    gMCLIBCurrentABC.ib  = phaseCurrentV * (float)(0.000112);

    /* Clarke transform */
    MCLIB_ClarkeTransform(&gMCLIBCurrentABC, &gMCLIBCurrentAlphaBeta);

	/* Park transform */
    MCLIB_ParkTransform(&gMCLIBCurrentAlphaBeta, &gMCLIBPosition, &gMCLIBCurrentDQ);

    /* Calculate control values  */
    MCAPP_MotorCurrentControl();

    /* Calculate park angle */
    MCAPP_MotorAngleCalc();
    
    gMCLIBPosition.angle = gfocParam.angle;

    /* Calculate qSin,qCos from qAngle  */
    MCLIB_SinCosCalc( &gMCLIBPosition );

    /* Calculate qValpha, qVbeta from qSin,qCos,qVd,qVq */
    MCLIB_InvParkTransform(&gMCLIBVoltageDQ, &gMCLIBPosition, &gMCLIBVoltageAlphaBeta);
    
    /* Calculate and set PWM duty cycles from Vr1,Vr2,Vr3 */
    MCLIB_SVPWMGen(&gMCLIBVoltageAlphaBeta, &gMCLIBSVPWM);

    MCAPP_PWMDutyUpdate(gMCLIBSVPWM.dPWM1, gMCLIBSVPWM.dPWM2, gMCLIBSVPWM.dPWM3);
  
    /* sync count for slow control loop execution */
    gCtrlParam.sync_cnt++;

    /* PB17 GPIO is used for timing measurement. - Set Low*/
    PIOB_REGS->PIO_CODR = ((uint32_t)1U <<(17U & 0x1FU));
}

/******************************************************************************/
/* Function name: MCAPP_SlowControlLoop                                       */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Slow control loop is in sync with fast control loop           */
/*              ADCEOCInterruptHandler().Slow loop execution frequency should */
/*              be in multiple of PWM frequency and be configured by          */
/*              "SLOW_LOOP_TIME_SEC" MACRO in "userparms.h file.              */
/*               Speed ramp and speed control loop is executed from this loop */
/******************************************************************************/
__STATIC_INLINE void MCAPP_SlowControlLoop(void)
{
    int16_t pos_count_diff;

#if(TORQUE_MODE == false)

    if(gCtrlParam.openLoop == false)
    {
        /* PB18 GPIO is used for timing measurement. - Set Low*/
        PIOB_REGS->PIO_SODR = (uint32_t)((uint32_t)1U << (18U & 0x1FU));
        gCtrlParam.endSpeed = motor_speed_target_elec_rad_per_sec;
 
        /* Speed Ramp */
        MCAPP_SpeedRamp();

        /* Speed Calculation from Encoder */
        gPositionCalc.present_position_count = (int16_t)(TC1_REGS->TC_CHANNEL[0].TC_CV);
        if( ( gCtrlParam.oldStatus == MOTOR_STATUS_STOPPED ) && ( gCtrlParam.motorStatus == MOTOR_STATUS_RUNNING ))
        {
           gPositionCalc.prev_position_count = gPositionCalc.present_position_count;
        }
        pos_count_diff = gPositionCalc.present_position_count - gPositionCalc.prev_position_count;
        speed_elec_rad_per_sec = ((float)pos_count_diff * 2.0f*(float)M_PI)/((float)ENCODER_PULSES_PER_EREV *SLOW_LOOP_TIME_SEC );
        gPositionCalc.prev_position_count = gPositionCalc.present_position_count;
            
        /* Execute the velocity control loop */
        gPIParmQref.inMeas = speed_elec_rad_per_sec;
        gPIParmQref.inRef  = gCtrlParam.velRef*(float)gCtrlParam.direction;
        MCLIB_PIControl(&gPIParmQref);
        gCtrlParam.iqRef = gPIParmQref.out;
        gCtrlParam.oldStatus = gCtrlParam.motorStatus;
        /* PB18 GPIO is used for timing measurement. - Set Low*/
        PIOB_REGS->PIO_CODR = (uint32_t)((uint32_t)1U << (18U & 0x1FU));
    }
#endif	// End of #if(TORQUE_MODE == false)
}

/******************************************************************************/
/* Function name: MCAPP_MotorStart                                                 */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Enables fast control loop and starts the PWM's.               */
/******************************************************************************/
void MCAPP_MotorStart(void)
{
    /* Stop QDEC Timer */
    /* Initialize motor control variables */
    MCAPP_MotorControlParamInit();
    
    /* ADC conversion start */
    TC0_CH1_TimerStart();
    TC3_CH0_CaptureStart();
    TC3_CH1_CaptureStart();

    // Skip first 10 samples
    uint32_t current_count = sinc3_out_sample_count;
    while (sinc3_out_sample_count < (current_count+10U) )
    {
        /*Skip first 10 samples*/
    }

    MCAPP_ADCOffsetCalibration();

    //Enable peripheral control of the PWM low pins : PA4, PA5, PA6
    PIOA_REGS->PIO_MSKR = 0x70U;
	PIOA_REGS->PIO_CFGR = 0x3U;
    gCtrlParam.motorStatus = MOTOR_STATUS_RUNNING;

    /* Clear fault before start */
    PWM0_FaultStatusClear(PWM_FAULT_ID_1);

    /* Enable PWM channels. */
    PWM0_ChannelsStart(PWM_CHANNEL_0_MASK);

    gMCAPPData.mcState = MC_APP_STATE_RUNNING;
}

/******************************************************************************/
/* Function name: MCAPP_Median_filter                                         */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Compute median filter on the three input numbers              */
/******************************************************************************/
static uint32_t MCAPP_Median_filter(uint32_t a, uint32_t b, uint32_t c)
{
    if (a>b)
    {
        if (b>c){
            return b;}
        else if (a>c){
            return c;}
        else{
            return a;}
    }
    else
    {
        if (a>c){
            return a;}
        else if (b>c){
            return c;}
        else{
            return b;}
    }
}

/******************************************************************************/
/* Function name: MCAPP_CurrentSNSCountISR                                    */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: TC interrupt is used for executing current SNS count loop.    */
/* ISR Timings - Get current TC counter and execute the decimation filter.    */
/******************************************************************************/
void __attribute__ ((tcm)) MCAPP_CurrentSNSCountISR(TC_TIMER_STATUS status, uintptr_t context)
{
    uint32_t temp1, temp2, temp3;
    /* PB28 GPIO is used for timing measurement. - Set High*/    
    PIOB_REGS->PIO_SODR =(uint32_t)((uint32_t)1U << (28U & 0x1FU));

    currentUActive = TC3_REGS->TC_CHANNEL[0].TC_CV;
    currentVActive = TC3_REGS->TC_CHANNEL[1].TC_CV;

    //Advance median filter delay line - channel U
    gCurrentU.s1_out_pp = gCurrentU.s1_out_p;
    gCurrentU.s1_out_p = gCurrentU.sinc1_out;
    
    //Advance median filter delay line - channel V
    gCurrentV.s1_out_pp = gCurrentV.s1_out_p;
    gCurrentV.s1_out_p = gCurrentV.sinc1_out;
    
    //Calculate delta for channel U
    temp3 = currentUActive;
    gCurrentU.sinc1_out = temp3 - gCurrentU.sinc1_prevq;
    gCurrentU.sinc1_prevq = temp3;

    // Limit sinc1_out value in case of counter error
    if (gCurrentU.sinc1_out > 200U)
    {
        gCurrentU.sinc1_out = 200U;
    }
    temp2 =gCurrentU.intg2;
    gCurrentU.intg3 = (gCurrentU.intg3 + temp2);
    gCurrentU.intg2 = (temp2 + gCurrentU.intg1);
    temp1=gCurrentU.sinc1_out;
    gCurrentU.intg1 = (gCurrentU.intg1 + temp1);
    
    temp2=gCurrentU.s1_out_pp;
    // Calculate median for channel U
    gCurrentU.sinc1_out = MCAPP_Median_filter(temp1, temp2, gCurrentU.s1_out_p);
    
    //Calculate delta for channel V
    temp3 = currentVActive;
    gCurrentV.sinc1_out = temp3 - gCurrentV.sinc1_prevq;
    gCurrentV.sinc1_prevq = currentVActive;
    
    // Limit sinc1_out value in case of counter error
    if (gCurrentV.sinc1_out > 200U)
    {
        gCurrentV.sinc1_out = 200U;
    }
    temp2=gCurrentV.intg2;
    gCurrentV.intg3 = (gCurrentV.intg3 + temp2);
    gCurrentV.intg2 = (temp2 + gCurrentV.intg1);
    temp1=gCurrentV.sinc1_out;
    gCurrentV.intg1 = (gCurrentV.intg1 + temp1);
    temp2=gCurrentV.s1_out_pp;
    // Calculate median for channel V
    gCurrentV.sinc1_out = MCAPP_Median_filter(temp1, temp2, gCurrentV.s1_out_p);
   
    sinc3_count++;
    if (sinc3_count >= 5U)
    {
        sinc3_count = 0U;
        
        //Average 3 sample delay line - channel U
        gCurrentU.sinc3_out_ppp = gCurrentU.sinc3_out_pp;
        gCurrentU.sinc3_out_pp = gCurrentU.sinc3_out_p;
        gCurrentU.sinc3_out_p = gCurrentU.sinc3_out;

        //Average 3 sample delay line - channel V
        gCurrentV.sinc3_out_ppp = gCurrentV.sinc3_out_pp;
        gCurrentV.sinc3_out_pp = gCurrentV.sinc3_out_p;
        gCurrentV.sinc3_out_p = gCurrentV.sinc3_out;

        temp3=gCurrentU.intg3;
        temp2=gCurrentU.der2;
        temp1=gCurrentU.der1;
        gCurrentU.sinc3_out = (temp3 - temp1 - temp2 - gCurrentU.der3);
        gCurrentU.der3 = (temp3 - temp1 - temp2);
        gCurrentU.der2 = (temp3 - temp1);
        gCurrentU.der1 = (temp3);
        
        temp3=gCurrentV.intg3;
        temp2=gCurrentV.der2;
        temp1=gCurrentV.der1;
        gCurrentV.sinc3_out = (temp3 - temp1 - temp2 - gCurrentV.der3);
        gCurrentV.der3 = (temp3 - temp1 - temp2);
        gCurrentV.der2 = (temp3 - temp1);
        gCurrentV.der1 = (temp3);
        
        sinc3_out_sample_count++;
    }

    /* PA28 GPIO is used for timing measurement. - Set Low*/
    PIOB_REGS->PIO_CODR = (uint32_t)((uint32_t)1U << (28U & 0x1FU));
}

/******************************************************************************/
/* Function name: MotorStop                                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Stops PWM and disables fast control loop.                    */
/******************************************************************************/
void MCAPP_MotorStop(void)
{
    //Disable peripheral control of the PWM low pins : PA4, PA5, PA6
    PIOA_REGS->PIO_MSKR = 0x70U;
    PIOA_REGS->PIO_CFGR = 0x0U;

    /* Disables PWM channels. */
    PWM0_ChannelsStop((PWM_CHANNEL_MASK)((uint8_t)PWM_CHANNEL_0_MASK | (uint8_t)PWM_CHANNEL_1_MASK | (uint8_t)PWM_CHANNEL_2_MASK));
	
	/* Reset algorithm specific variables for next iteration.*/
	MCAPP_MotorControlParamInit();
    
    // Stop current measures
    TC0_CH1_TimerStop();
    TC3_CH0_CaptureStop();
    TC3_CH1_CaptureStop();
	
    gPIParmQref.inMeas = 0.0f;
    gPIParmQref.inRef = 0.0f;
    gPIParmQref.dSum = 0.0f;
    gCtrlParam.endSpeed = 0.0f;
	gCtrlParam.velRef = 0.0f;
	gCtrlParam.motorStatus = MOTOR_STATUS_STOPPED;
    gCtrlParam.oldStatus = MOTOR_STATUS_STOPPED;
    gMCAPPData.mcState = MC_APP_STATE_STOP;
    speed_ref_filtered = 0.0f;
}

/******************************************************************************/
/* Function name: MCAPP_LedDirectionUpdate                                    */
/* Function parameters: ledOn                                                 */
/* Function return: None                                                      */
/* Description: Set LED direction information                                 */
/******************************************************************************/
static void MCAPP_LedDirectionUpdate(bool ledOn)
{
    if (ledOn == true)
    {
        if (gMCAPPData.mcDirection == MC_APP_DIRECTION_FORWARD)
        {
            LED_BI_GREEN_Set();
            LED_BI_RED_Clear();
        }
        else
        {
            LED_BI_GREEN_Clear();
            LED_BI_RED_Set();
        }
    }
    else
    {
        LED_BI_GREEN_Clear();
        LED_BI_RED_Clear();
    }
}

/******************************************************************************/
/* Function name: MCAPP_SwitchStartDebounce                                        */
/* Function parameters: state                                                 */
/* Function return: None                                                      */
/* Description: Switch button debounce logic                                  */
/******************************************************************************/
static void MCAPP_SwitchStartDebounce(MC_APP_STATE state)
{
    if (!(bool)SWITCH_START_Get())
    {
        gMCAPPData.switchStartCount++;
        if (gMCAPPData.switchStartCount >= 0xFFU)            
        {
           gMCAPPData.switchStartCount = 0U;
           gMCAPPData.switchStartState = MC_APP_SWITCH_PRESSED;
        }           
    }
    if (gMCAPPData.switchStartState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_START_Get())
        {
            gMCAPPData.switchStartCount = 0U;
            gMCAPPData.switchStartState = MC_APP_SWITCH_RELEASED;
            gMCAPPData.mcState = state;
            motor_activity_count = 0U;
            LED3_Clear();
        }
    }
}

/******************************************************************************/
/* Function name: MCAPP_SpeedIncrease                                         */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Increase motor speed target in elec rad per sec               */
/******************************************************************************/
static void MCAPP_SpeedIncrease(void)
{
    if (motor_speed_target_elec_rad_per_sec < 800.0f)
    {
        motor_speed_target_elec_rad_per_sec += 100.0f;
    }
    else
    {
        motor_speed_target_elec_rad_per_sec = 800.0f;
    }
}

/******************************************************************************/
/* Function name: MCAPP_SpeedDecrease                                         */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Decrease motor speed target in elec rad per sec               */
/******************************************************************************/
static void MCAPP_SpeedDecrease(void)
{
    if (motor_speed_target_elec_rad_per_sec > 100.0f)
    {
        motor_speed_target_elec_rad_per_sec -= 100.0f;
    }
    else
    {
        motor_speed_target_elec_rad_per_sec = 100.0f;
    }
}

/******************************************************************************/
/* Function name: MCAPP_SwitchIncrDebounce                                    */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Switch speed increase button debounce logic                   */
/******************************************************************************/
static void MCAPP_SwitchIncrDebounce(void)
{
    if (!(bool)SWITCH_INCR_Get())
    {
        gMCAPPData.switchIncrCount++;
        if (gMCAPPData.switchIncrCount >= 0xFFU)            
        {
           gMCAPPData.switchIncrCount = 0U;
           gMCAPPData.switchIncrState = MC_APP_SWITCH_PRESSED;
        }           
    }
    if (gMCAPPData.switchIncrState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_INCR_Get())
        {
            gMCAPPData.switchIncrCount = 0U;
            gMCAPPData.switchIncrState = MC_APP_SWITCH_RELEASED;
            MCAPP_SpeedIncrease();
            motor_activity_count = 0U;
            LED3_Clear();
        }
    }
}

/******************************************************************************/
/* Function name: MCAPP_SwitchDecrDebounce                                    */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Switch speed decrease button debounce logic                   */
/******************************************************************************/
static void MCAPP_SwitchDecrDebounce(void)
{
    if (!(bool)SWITCH_DECR_Get())
    {
        gMCAPPData.switchDecrCount++;
        if (gMCAPPData.switchDecrCount >= 0xFFU)            
        {
           gMCAPPData.switchDecrCount = 0U;
           gMCAPPData.switchDecrState = MC_APP_SWITCH_PRESSED;
        }           
    }
    if (gMCAPPData.switchDecrState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_DECR_Get())
        {
            gMCAPPData.switchDecrCount = 0U;
            gMCAPPData.switchDecrState = MC_APP_SWITCH_RELEASED;
            MCAPP_SpeedDecrease();
            motor_activity_count = 0U;
            LED3_Clear();
        }
    }
}

/******************************************************************************/
/* Function name: MCAPP_SwitchResetDebounce                                   */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Switch Reset button debounce logic                            */
/******************************************************************************/
static void MCAPP_SwitchResetDebounce(void)
{
    if (!(bool)SWITCH_RESET_Get())
    {
        gMCAPPData.switchResetCount++;
        if (gMCAPPData.switchResetCount >= 0xFFU)            
        {
           gMCAPPData.switchResetCount = 0U;
           gMCAPPData.switchResetState = MC_APP_SWITCH_PRESSED;
        }           
    }
    if (gMCAPPData.switchResetState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_RESET_Get())
        {
            gMCAPPData.switchResetCount = 0U;
            gMCAPPData.switchResetState = MC_APP_SWITCH_RELEASED;
            
            /* Perform reset action : To be implemented, temporary toggle LED2 */
            LED2_Toggle();
        }
    }
}

/******************************************************************************/
/* Function name: MCAPP_SwitchDirectionDebounce                               */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Switch direction button debounce logic                        */
/******************************************************************************/
static void MCAPP_SwitchDirectionDebounce(void)
{
    if (!(bool)SWITCH_DIRECTION_Get())
    {
        gMCAPPData.switchDirectionCount++;
        if (gMCAPPData.switchDirectionCount >= 0xFFU)            
        {
           gMCAPPData.switchDirectionCount = 0U;
           gMCAPPData.switchDirectionState = MC_APP_SWITCH_PRESSED;
        }           
    }
    if (gMCAPPData.switchDirectionState == MC_APP_SWITCH_PRESSED)
    {
        if ((bool)SWITCH_DIRECTION_Get())
        {
            gMCAPPData.switchDirectionCount = 0U;
            gMCAPPData.switchDirectionState = MC_APP_SWITCH_RELEASED;
 
            if (gMCAPPData.mcState == MC_APP_STATE_WAIT_START)
            {
                if (gMCAPPData.mcDirection == MC_APP_DIRECTION_FORWARD)
                {
                    gMCAPPData.mcDirection = MC_APP_DIRECTION_REVERSE;
                    gCtrlParam.direction = -1;
                }
                else
                {
                    gMCAPPData.mcDirection = MC_APP_DIRECTION_FORWARD;
                    gCtrlParam.direction = 1;
                }
                MCAPP_LedDirectionUpdate(true);
            }
        }
    }
}

/******************************************************************************/
/* Function name: MCAPP_Tasks                                                  */
/* Function parameters: None                                                  */
/* Function return: None                                                      */
/* Description: Motor application state machine                               */
/******************************************************************************/
void MCAPP_Tasks(void)
{
  switch (gMCAPPData.mcState)
  {
      case MC_APP_STATE_INIT:
                      /* Set field alignment flag */
            gCtrlParam.fieldAlignmentFlag = 1U;
          //Disable peripheral control of the PWM low pins : PA4, PA5, PA6
          PIOA_REGS->PIO_MSKR = 0x70U;
          PIOA_REGS->PIO_CFGR = 0x0U;

          /* Start TC0 to trigger periodic PWM duty update */
          NVIC_DisableIRQ(TC0_CH0_IRQn);
          NVIC_ClearPendingIRQ(TC0_CH0_IRQn);
          NVIC_SetPriority(TC0_CH0_IRQn, 1U);
          NVIC_EnableIRQ(TC0_CH0_IRQn);
          TC0_CH0_CompareCallbackRegister(MCAPP_ControlLoopISR, (uintptr_t)dummyforMisra);
          TC0_REGS->TC_CHANNEL[0].TC_EMR |= TC_EMR_TRIGSRCB(TC_EMR_TRIGSRCB_PWMx_Val);
          TC0_REGS->TC_CHANNEL[0].TC_CCR = (TC_CCR_CLKEN_Msk);
          TC0_CH0_CompareStart();

          /* Start TC1 for current measurement. Use the Burst option to increase
           the counter only when LX7720 SNS signal are at level logic one */
          NVIC_DisableIRQ(TC1_CH0_IRQn);
          NVIC_ClearPendingIRQ(TC1_CH0_IRQn);
          NVIC_SetPriority(TC1_CH0_IRQn, 0U);
          TC0_CH1_TimerCallbackRegister(MCAPP_CurrentSNSCountISR, (uintptr_t)dummyforMisra);
          NVIC_EnableIRQ(TC1_CH0_IRQn);
          TC3_REGS->TC_CHANNEL[0].TC_CMR |= TC_CMR_BURST_XC0;
          TC3_REGS->TC_CHANNEL[1].TC_CMR |= TC_CMR_BURST_XC1;

          gMCAPPData.switchStartCount = 0xFF;
          gMCAPPData.mcDirection = MC_APP_DIRECTION_FORWARD;
          gCtrlParam.direction = 1;
          gMCAPPData.mcState = MC_APP_STATE_WAIT_START;
          break;
          
      case MC_APP_STATE_WAIT_START:
          MCAPP_SwitchStartDebounce(MC_APP_STATE_START);  
          MCAPP_SwitchDirectionDebounce();
          break;

      case MC_APP_STATE_START:
          LED0_Set();
          MCAPP_LedDirectionUpdate(true);
          MCAPP_MotorStart();
          motor_activity_count = 0U;
          LED3_Clear();

          gMCAPPData.mcState = MC_APP_STATE_RUNNING;  
          break;

      case MC_APP_STATE_RUNNING:
          if(MCAPP_SlowLoopTimeIsFinished())
          {
            MCAPP_SlowControlLoop();
            motor_activity_count++;
          }

          if (motor_activity_count > ( 5U * MOTOR_ACTIVITY_SLOW_LOOP_COUNT_60_SEC) ) // 5 minutes
          {
              gMCAPPData.mcState = MC_APP_STATE_STOP_DECREASE;
          }
          else if (motor_activity_count > ( 4U * MOTOR_ACTIVITY_SLOW_LOOP_COUNT_60_SEC) ) // > 4 minutes
          {
              LED3_Set();
          }
          else{
              /* Dummy branch for MISRAC compliance*/
          }

          MCAPP_SwitchStartDebounce(MC_APP_STATE_STOP_DECREASE);
          
          // Check buttons
          MCAPP_SwitchDecrDebounce();
          MCAPP_SwitchIncrDebounce();
          MCAPP_SwitchResetDebounce();
         break;

        case MC_APP_STATE_STOP_DECREASE:

            if (speed_elec_rad_per_sec*(float)gCtrlParam.direction <= 110.0f)
            {
                gMCAPPData.mcState = MC_APP_STATE_STOP;

            }
            else
            {
                if(MCAPP_SlowLoopTimeIsFinished())
                {
                    MCAPP_SlowControlLoop();
                  
                    if((speed_elec_rad_per_sec*(float)gCtrlParam.direction) <= (motor_speed_target_elec_rad_per_sec + 10.0f))
                    {
                       MCAPP_SpeedDecrease();
                    }
                }
            }
        break;

      case MC_APP_STATE_STOP:
        LED0_Clear();
        MCAPP_LedDirectionUpdate(false);
        LED3_Clear();
        MCAPP_MotorStop();
        gMCAPPData.mcState = MC_APP_STATE_WAIT_START;
        break;

      default:
          /* Undefined state: Should never come here*/
          break;
  }
}
