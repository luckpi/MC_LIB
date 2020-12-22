#ifndef _FDWEAK_H
#define _FDWEAK_H
#include "IQmath.h"
#include "common.h"
#include "MotorConfig.h"
#include "smc.h"
/******************************** Field Weakening *****************************/
/* Field Weakening constant for constant torque range 
   Flux reference value */
#define IDREF_BASESPEED                NORM_CURRENT(0.0)
#define NORM_CURRENT(current_real)     Q15(current_real / MAX_MOTOR_CURRENT)
/*-------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening of the surface mounted permanent magnets
   PMSMs the mechanical damage of the rotor and the
   demagnetization of the permanent magnets is possible if
   cautions measures are not taken or the motor’s producer
   specifications are not respected.
  -------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening regime implementation, if the FOC is lost
   at high speed above the nominal value, the possibility of
   damaging the inverter is eminent. The reason is that the
   BEMF will have a greater value than the one that would be
   obtained for the nominal speed exceeding the DC bus voltage
   value and though the inverter’s power semiconductors and DC
   link capacitors would have to support it. Since the tuning
   proposed implies iterative coefficient corrections until
   the optimum functioning is achieved, the protection of the
   inverter with corresponding circuitry should be assured in
   case of stalling at high speeds.                            */

/* speed index is increase */
#define SPEED_INDEX_CONST 10

#define FWONSPEED FW_NOMINAL_SPEED_RPM *NOPOLESPAIRS

/* the following values indicate the d-current variation with speed 
 please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)     /* up to 2800 RPM */
#define	IDREF_SPEED1	NORM_CURRENT(-0.7)  /* ~2950 RPM */
#define	IDREF_SPEED2	NORM_CURRENT(-0.9)  /* ~3110 RPM */
#define	IDREF_SPEED3	NORM_CURRENT(-1.0)  /* ~3270 RPM */
#define	IDREF_SPEED4	NORM_CURRENT(-1.4)  /* ~3430 RPM */
#define	IDREF_SPEED5	NORM_CURRENT(-1.7)  /* ~3600 RPM */
#define	IDREF_SPEED6	NORM_CURRENT(-2.0)  /* ~3750 RPM */
#define	IDREF_SPEED7	NORM_CURRENT(-2.1)  /* ~3910 RPM */
#define	IDREF_SPEED8	NORM_CURRENT(-2.2)  /* ~4070 RPM */
#define	IDREF_SPEED9	NORM_CURRENT(-2.25) /* ~4230 RPM */
#define	IDREF_SPEED10	NORM_CURRENT(-2.3)  /* ~4380 RPM */
#define	IDREF_SPEED11	NORM_CURRENT(-2.35) /* ~4550 RPM */
#define	IDREF_SPEED12	NORM_CURRENT(-2.4)  /* ~4700 RPM */
#define	IDREF_SPEED13	NORM_CURRENT(-2.45) /* ~4860 RPM */
#define	IDREF_SPEED14	NORM_CURRENT(-2.5)  /* ~5020 RPM */
#define	IDREF_SPEED15	NORM_CURRENT(-2.5)  /* ~5180 RPM */
#define	IDREF_SPEED16	NORM_CURRENT(-2.5)  /* ~5340 RPM */
#define	IDREF_SPEED17	NORM_CURRENT(-2.5)  /* ~5500 RPM */

// This pre-processor condition will generate an error if maximum speed is out of
// range on Q15 when calculating Omega.
// #if (FW_NOMINAL_SPEED_RPM < NOMINAL_SPEED_RPM)
// 	#error FIELDWEAKSPEEDRPM must be greater than NOMINALSPEEDINRPM for field weakening.
// 	#error if application does not require Field Weakening, set FIELDWEAKSPEEDRPM value
// 	#error equal to NOMINALSPEEDINRPM
// #elif (((FW_NOMINAL_SPEED_RPM*NOPOLESPAIRS*2)/(60*SPEEDLOOPFREQ)) >= 1)
// 		#error FIELDWEAKSPEEDRPM will generate an Omega value greater than 1 which is the
// 		#error maximum in Q15 format. Reduce FIELDWEAKSPEEDRPM value, or increase speed
// 		#error control loop frequency, SPEEDLOOPFREQ
// #endif
/******************** End of Field Weakening Params ***************************/

//------------------  C API for FdWeak routine ---------------------

typedef struct
{
   int16_t qK1; // < Nominal speed value
   int16_t qIdRef;
   int16_t qFwOnSpeed;
   int16_t qFwActiv;
   int16_t qIndex;
   int16_t qFWPercentage;
   int16_t qInterpolPortion;
   int16_t qFwCurve[18]; // Curve for magnetizing current
} tFdWeakParm;
extern tFdWeakParm FdWeakParm;
extern int16_t FieldWeakening(int16_t qMotorSpeed);
extern void FWInit(void);

#endif
