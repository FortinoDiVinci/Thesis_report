/*
 * youbot_motor_joint3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "youbot_motor_joint3".
 *
 * Model version              : 1.16
 * Simulink Coder version : 8.13 (R2017b) 24-Jul-2017
 * C source code generated on : Thu Oct  3 09:51:49 2019
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_youbot_motor_joint3_h_
#define RTW_HEADER_youbot_motor_joint3_h_
#include <math.h>
#include <float.h>
#include <string.h>
#include <stddef.h>
#ifndef youbot_motor_joint3_COMMON_INCLUDES_
# define youbot_motor_joint3_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* youbot_motor_joint3_COMMON_INCLUDES_ */

#include "youbot_motor_joint3_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
# define rtmGetFinalTime(rtm)          ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
# define rtmGetRTWLogInfo(rtm)         ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
# define rtmGetTFinal(rtm)             ((rtm)->Timing.tFinal)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T Gain;                         /* '<S5>/Gain' */
  real_T PulseGenerator1;              /* '<Root>/Pulse Generator1' */
  real_T x0;                           /* '<S4>/x0' */
  real_T Envdamping;                   /* '<S4>/Env. damping' */
  real_T Envinertia;                   /* '<S4>/Env. inertia' */
  real_T PulseGenerator;               /* '<Root>/Pulse Generator' */
  real_T Integrator;                   /* '<S5>/Integrator' */
  real_T v_ref;                        /* '<Root>/Manual Switch' */
  real_T ManualSwitch2;                /* '<Root>/Manual Switch2' */
  real_T ManualSwitch;                 /* '<S5>/Manual Switch' */
  real_T Sum;                          /* '<S3>/Sum' */
  real_T Constant[3];                  /* '<Root>/Constant' */
  real_T TrigonometricFunction1;       /* '<S8>/Trigonometric Function1' */
  real_T TrigonometricFunction;        /* '<S8>/Trigonometric Function' */
  real_T Constant1;                    /* '<S8>/Constant1' */
  real_T TrigonometricFunction2;       /* '<S9>/Trigonometric Function2' */
  real_T TrigonometricFunction3;       /* '<S9>/Trigonometric Function3' */
  real_T IntegralGain;                 /* '<S1>/Integral Gain' */
  real_T IntegralGain_p;               /* '<S3>/Integral Gain' */
  real_T Sum2;                         /* '<S5>/Sum2' */
  real_T Sum3;                         /* '<S5>/Sum3' */
  real_T IntegralGain_d;               /* '<S7>/Integral Gain' */
} B_youbot_motor_joint3_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S10>/UD' */
  struct {
    void *LoggedData[3];
  } Scope_PWORK;                       /* '<Root>/Scope' */

  struct {
    void *LoggedData[3];
  } Scope1_PWORK;                      /* '<Root>/Scope1' */

  struct {
    void *LoggedData[3];
  } Scope2_PWORK;                      /* '<Root>/Scope2' */

  struct {
    void *LoggedData[3];
  } Scope3_PWORK;                      /* '<Root>/Scope3' */

  struct {
    void *LoggedData[2];
  } Scope4_PWORK;                      /* '<Root>/Scope4' */

  struct {
    void *LoggedData[2];
  } Scope5_PWORK;                      /* '<Root>/Scope5' */

  int32_T clockTickCounter;            /* '<Root>/Pulse Generator1' */
  int32_T clockTickCounter_b;          /* '<Root>/Pulse Generator' */
} DW_youbot_motor_joint3_T;

/* Continuous states (auto storage) */
typedef struct {
  real_T Motormech_CSTATE;             /* '<S5>/Motor mech' */
  real_T integrator_CSTATE;            /* '<S4>/integrator' */
  real_T Integrator_CSTATE;            /* '<S1>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S5>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S7>/Integrator' */
  real_T Motorelec_CSTATE;             /* '<S5>/Motor elec' */
  real_T Integrator_CSTATE_c;          /* '<S3>/Integrator' */
} X_youbot_motor_joint3_T;

/* State derivatives (auto storage) */
typedef struct {
  real_T Motormech_CSTATE;             /* '<S5>/Motor mech' */
  real_T integrator_CSTATE;            /* '<S4>/integrator' */
  real_T Integrator_CSTATE;            /* '<S1>/Integrator' */
  real_T Integrator_CSTATE_p;          /* '<S5>/Integrator' */
  real_T Integrator_CSTATE_d;          /* '<S7>/Integrator' */
  real_T Motorelec_CSTATE;             /* '<S5>/Motor elec' */
  real_T Integrator_CSTATE_c;          /* '<S3>/Integrator' */
} XDot_youbot_motor_joint3_T;

/* State disabled  */
typedef struct {
  boolean_T Motormech_CSTATE;          /* '<S5>/Motor mech' */
  boolean_T integrator_CSTATE;         /* '<S4>/integrator' */
  boolean_T Integrator_CSTATE;         /* '<S1>/Integrator' */
  boolean_T Integrator_CSTATE_p;       /* '<S5>/Integrator' */
  boolean_T Integrator_CSTATE_d;       /* '<S7>/Integrator' */
  boolean_T Motorelec_CSTATE;          /* '<S5>/Motor elec' */
  boolean_T Integrator_CSTATE_c;       /* '<S3>/Integrator' */
} XDis_youbot_motor_joint3_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Parameters (auto storage) */
struct P_youbot_motor_joint3_T_ {
  real_T B_env;                        /* Variable: B_env
                                        * Referenced by: '<S4>/Env. damping'
                                        */
  real_T I_env;                        /* Variable: I_env
                                        * Referenced by: '<S4>/Env. inertia'
                                        */
  real_T K;                            /* Variable: K
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  real_T K_cur;                        /* Variable: K_cur
                                        * Referenced by: '<S3>/Proportional Gain'
                                        */
  real_T K_env;                        /* Variable: K_env
                                        * Referenced by: '<S4>/Env. stiffness'
                                        */
  real_T K_pos;                        /* Variable: K_pos
                                        * Referenced by: '<S6>/Proportional Gain'
                                        */
  real_T Ke;                           /* Variable: Ke
                                        * Referenced by: '<S5>/Speed constant'
                                        */
  real_T Ki;                           /* Variable: Ki
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
  real_T Ki_cur;                       /* Variable: Ki_cur
                                        * Referenced by: '<S3>/Integral Gain'
                                        */
  real_T Kt;                           /* Variable: Kt
                                        * Referenced by: '<S5>/Gain1'
                                        */
  real_T Kv;                           /* Variable: Kv
                                        * Referenced by: '<S7>/Proportional Gain'
                                        */
  real_T Kvi;                          /* Variable: Kvi
                                        * Referenced by: '<S7>/Integral Gain'
                                        */
  real_T N;                            /* Variable: N
                                        * Referenced by:
                                        *   '<S5>/Gain'
                                        *   '<S5>/Gain2'
                                        */
  real_T weight[3];                    /* Variable: weight
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T DiscreteDerivative_ICPrevScaled;/* Mask Parameter: DiscreteDerivative_ICPrevScaled
                                          * Referenced by: '<S10>/UD'
                                          */
  real_T Saturation_UpperSat;          /* Expression: 0.5
                                        * Referenced by: '<S5>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -0.5
                                        * Referenced by: '<S5>/Saturation'
                                        */
  real_T Motormech_A;                  /* Computed Parameter: Motormech_A
                                        * Referenced by: '<S5>/Motor mech'
                                        */
  real_T Motormech_C;                  /* Computed Parameter: Motormech_C
                                        * Referenced by: '<S5>/Motor mech'
                                        */
  real_T PulseGenerator1_Amp;          /* Expression: 1
                                        * Referenced by: '<Root>/Pulse Generator1'
                                        */
  real_T PulseGenerator1_Period;       /* Computed Parameter: PulseGenerator1_Period
                                        * Referenced by: '<Root>/Pulse Generator1'
                                        */
  real_T PulseGenerator1_Duty;         /* Computed Parameter: PulseGenerator1_Duty
                                        * Referenced by: '<Root>/Pulse Generator1'
                                        */
  real_T PulseGenerator1_PhaseDelay;   /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator1'
                                        */
  real_T x0_Value;                     /* Expression: 0
                                        * Referenced by: '<S4>/x0'
                                        */
  real_T integrator_A;                 /* Computed Parameter: integrator_A
                                        * Referenced by: '<S4>/integrator'
                                        */
  real_T integrator_C;                 /* Computed Parameter: integrator_C
                                        * Referenced by: '<S4>/integrator'
                                        */
  real_T TSamp_WtEt;                   /* Computed Parameter: TSamp_WtEt
                                        * Referenced by: '<S10>/TSamp'
                                        */
  real_T Integrator_IC;                /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T PulseGenerator_Amp;           /* Expression: 90/180*pi
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_Period;        /* Computed Parameter: PulseGenerator_Period
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_Duty;          /* Computed Parameter: PulseGenerator_Duty
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T PulseGenerator_PhaseDelay;    /* Expression: 0
                                        * Referenced by: '<Root>/Pulse Generator'
                                        */
  real_T Integrator_IC_l;              /* Expression: 0
                                        * Referenced by: '<S5>/Integrator'
                                        */
  real_T Integrator_IC_i;              /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S7>/Integrator'
                                        */
  real_T Motorelec_A;                  /* Computed Parameter: Motorelec_A
                                        * Referenced by: '<S5>/Motor elec'
                                        */
  real_T Motorelec_C;                  /* Computed Parameter: Motorelec_C
                                        * Referenced by: '<S5>/Motor elec'
                                        */
  real_T Integrator_IC_k;              /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S3>/Integrator'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S8>/Constant1'
                                        */
  uint8_T ManualSwitch_CurrentSetting; /* Computed Parameter: ManualSwitch_CurrentSetting
                                        * Referenced by: '<Root>/Manual Switch'
                                        */
  uint8_T ManualSwitch2_CurrentSetting;/* Computed Parameter: ManualSwitch2_CurrentSetting
                                        * Referenced by: '<Root>/Manual Switch2'
                                        */
  uint8_T ManualSwitch_CurrentSetting_a;/* Computed Parameter: ManualSwitch_CurrentSetting_a
                                         * Referenced by: '<S5>/Manual Switch'
                                         */
  uint8_T ManualSwitch1_CurrentSetting;/* Computed Parameter: ManualSwitch1_CurrentSetting
                                        * Referenced by: '<Root>/Manual Switch1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_youbot_motor_joint3_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_youbot_motor_joint3_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[7];
  real_T odeF[3][7];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (auto storage) */
extern P_youbot_motor_joint3_T youbot_motor_joint3_P;

/* Block signals (auto storage) */
extern B_youbot_motor_joint3_T youbot_motor_joint3_B;

/* Continuous states (auto storage) */
extern X_youbot_motor_joint3_T youbot_motor_joint3_X;

/* Block states (auto storage) */
extern DW_youbot_motor_joint3_T youbot_motor_joint3_DW;

/* Model entry point functions */
extern void youbot_motor_joint3_initialize(void);
extern void youbot_motor_joint3_step(void);
extern void youbot_motor_joint3_terminate(void);

/* Real-time Model object */
extern RT_MODEL_youbot_motor_joint3_T *const youbot_motor_joint3_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'youbot_motor_joint3'
 * '<S1>'   : 'youbot_motor_joint3/Admittance ctrl'
 * '<S2>'   : 'youbot_motor_joint3/Cross Product'
 * '<S3>'   : 'youbot_motor_joint3/Current ctrl'
 * '<S4>'   : 'youbot_motor_joint3/Environnment '
 * '<S5>'   : 'youbot_motor_joint3/Motor'
 * '<S6>'   : 'youbot_motor_joint3/Position ctrl'
 * '<S7>'   : 'youbot_motor_joint3/Speed ctrl'
 * '<S8>'   : 'youbot_motor_joint3/theta projection'
 * '<S9>'   : 'youbot_motor_joint3/theta projection1'
 * '<S10>'  : 'youbot_motor_joint3/Environnment /Discrete Derivative'
 */
#endif                                 /* RTW_HEADER_youbot_motor_joint3_h_ */
