/*
 * youbot_motor_joint3.c
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

#include "youbot_motor_joint3.h"
#include "youbot_motor_joint3_private.h"

/* Block signals (auto storage) */
B_youbot_motor_joint3_T youbot_motor_joint3_B;

/* Continuous states */
X_youbot_motor_joint3_T youbot_motor_joint3_X;

/* Block states (auto storage) */
DW_youbot_motor_joint3_T youbot_motor_joint3_DW;

/* Real-time model */
RT_MODEL_youbot_motor_joint3_T youbot_motor_joint3_M_;
RT_MODEL_youbot_motor_joint3_T *const youbot_motor_joint3_M =
  &youbot_motor_joint3_M_;

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 7;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  youbot_motor_joint3_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  youbot_motor_joint3_step();
  youbot_motor_joint3_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  youbot_motor_joint3_step();
  youbot_motor_joint3_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void youbot_motor_joint3_step(void)
{
  /* local block i/o variables */
  real_T rtb_TSamp;
  real_T rtb_Motormech;
  real_T rtb_ManualSwitch1;
  real_T rtb_Sum_d;
  real_T rtb_Motorelec;
  real_T rtb_Add3_idx_0;
  real_T rtb_Add3_idx_2;
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    /* set solver stop time */
    if (!(youbot_motor_joint3_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&youbot_motor_joint3_M->solverInfo,
                            ((youbot_motor_joint3_M->Timing.clockTickH0 + 1) *
        youbot_motor_joint3_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&youbot_motor_joint3_M->solverInfo,
                            ((youbot_motor_joint3_M->Timing.clockTick0 + 1) *
        youbot_motor_joint3_M->Timing.stepSize0 +
        youbot_motor_joint3_M->Timing.clockTickH0 *
        youbot_motor_joint3_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(youbot_motor_joint3_M)) {
    youbot_motor_joint3_M->Timing.t[0] = rtsiGetT
      (&youbot_motor_joint3_M->solverInfo);
  }

  /* TransferFcn: '<S5>/Motor mech' */
  rtb_Motormech = youbot_motor_joint3_P.Motormech_C *
    youbot_motor_joint3_X.Motormech_CSTATE;

  /* Gain: '<S5>/Gain' */
  youbot_motor_joint3_B.Gain = 1.0 / youbot_motor_joint3_P.N * rtb_Motormech;
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    /* DiscretePulseGenerator: '<Root>/Pulse Generator1' */
    youbot_motor_joint3_B.PulseGenerator1 =
      (youbot_motor_joint3_DW.clockTickCounter <
       youbot_motor_joint3_P.PulseGenerator1_Duty) &&
      (youbot_motor_joint3_DW.clockTickCounter >= 0) ?
      youbot_motor_joint3_P.PulseGenerator1_Amp : 0.0;
    if (youbot_motor_joint3_DW.clockTickCounter >=
        youbot_motor_joint3_P.PulseGenerator1_Period - 1.0) {
      youbot_motor_joint3_DW.clockTickCounter = 0;
    } else {
      youbot_motor_joint3_DW.clockTickCounter++;
    }

    /* End of DiscretePulseGenerator: '<Root>/Pulse Generator1' */

    /* Constant: '<S4>/x0' */
    youbot_motor_joint3_B.x0 = youbot_motor_joint3_P.x0_Value;

    /* Gain: '<S4>/Env. damping' */
    youbot_motor_joint3_B.Envdamping = youbot_motor_joint3_P.B_env * 0.0;

    /* SampleTimeMath: '<S10>/TSamp'
     *
     * About '<S10>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp = 0.0 * youbot_motor_joint3_P.TSamp_WtEt;

    /* Gain: '<S4>/Env. inertia' incorporates:
     *  Sum: '<S10>/Diff'
     *  UnitDelay: '<S10>/UD'
     */
    youbot_motor_joint3_B.Envinertia = (rtb_TSamp -
      youbot_motor_joint3_DW.UD_DSTATE) * youbot_motor_joint3_P.I_env;

    /* DiscretePulseGenerator: '<Root>/Pulse Generator' */
    youbot_motor_joint3_B.PulseGenerator =
      (youbot_motor_joint3_DW.clockTickCounter_b <
       youbot_motor_joint3_P.PulseGenerator_Duty) &&
      (youbot_motor_joint3_DW.clockTickCounter_b >= 0) ?
      youbot_motor_joint3_P.PulseGenerator_Amp : 0.0;
    if (youbot_motor_joint3_DW.clockTickCounter_b >=
        youbot_motor_joint3_P.PulseGenerator_Period - 1.0) {
      youbot_motor_joint3_DW.clockTickCounter_b = 0;
    } else {
      youbot_motor_joint3_DW.clockTickCounter_b++;
    }

    /* End of DiscretePulseGenerator: '<Root>/Pulse Generator' */
  }

  /* Sum: '<Root>/Sum2' incorporates:
   *  Gain: '<S4>/Env. stiffness'
   *  Sum: '<S4>/Sum3'
   *  Sum: '<S4>/Sum5'
   *  TransferFcn: '<S4>/integrator'
   */
  rtb_ManualSwitch1 = ((youbot_motor_joint3_P.integrator_C *
                        youbot_motor_joint3_X.integrator_CSTATE -
                        youbot_motor_joint3_B.x0) * youbot_motor_joint3_P.K_env
                       + youbot_motor_joint3_B.Envdamping) +
    youbot_motor_joint3_B.Envinertia;

  /* Integrator: '<S5>/Integrator' */
  youbot_motor_joint3_B.Integrator = youbot_motor_joint3_X.Integrator_CSTATE_p;

  /* ManualSwitch: '<Root>/Manual Switch' incorporates:
   *  Gain: '<S1>/Proportional Gain'
   *  Gain: '<S6>/Proportional Gain'
   *  Integrator: '<S1>/Integrator'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S1>/Sum'
   */
  if (youbot_motor_joint3_P.ManualSwitch_CurrentSetting == 1) {
    youbot_motor_joint3_B.v_ref = youbot_motor_joint3_P.K * rtb_ManualSwitch1 +
      youbot_motor_joint3_X.Integrator_CSTATE;
  } else {
    youbot_motor_joint3_B.v_ref = (youbot_motor_joint3_B.PulseGenerator -
      youbot_motor_joint3_B.Integrator) * youbot_motor_joint3_P.K_pos;
  }

  /* End of ManualSwitch: '<Root>/Manual Switch' */

  /* Sum: '<Root>/Sum' */
  rtb_Sum_d = youbot_motor_joint3_B.v_ref - youbot_motor_joint3_B.Gain;

  /* ManualSwitch: '<Root>/Manual Switch2' incorporates:
   *  Gain: '<S7>/Proportional Gain'
   *  Integrator: '<S7>/Integrator'
   *  Sum: '<S7>/Sum'
   */
  if (youbot_motor_joint3_P.ManualSwitch2_CurrentSetting == 1) {
    youbot_motor_joint3_B.ManualSwitch2 = youbot_motor_joint3_B.PulseGenerator1;
  } else {
    youbot_motor_joint3_B.ManualSwitch2 = youbot_motor_joint3_P.Kv * rtb_Sum_d +
      youbot_motor_joint3_X.Integrator_CSTATE_d;
  }

  /* End of ManualSwitch: '<Root>/Manual Switch2' */
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
  }

  /* TransferFcn: '<S5>/Motor elec' */
  rtb_Motorelec = youbot_motor_joint3_P.Motorelec_C *
    youbot_motor_joint3_X.Motorelec_CSTATE;

  /* ManualSwitch: '<S5>/Manual Switch' incorporates:
   *  Saturate: '<S5>/Saturation'
   */
  if (youbot_motor_joint3_P.ManualSwitch_CurrentSetting_a == 1) {
    youbot_motor_joint3_B.ManualSwitch = rtb_Motorelec;
  } else if (rtb_Motorelec > youbot_motor_joint3_P.Saturation_UpperSat) {
    /* Saturate: '<S5>/Saturation' */
    youbot_motor_joint3_B.ManualSwitch =
      youbot_motor_joint3_P.Saturation_UpperSat;
  } else if (rtb_Motorelec < youbot_motor_joint3_P.Saturation_LowerSat) {
    /* Saturate: '<S5>/Saturation' */
    youbot_motor_joint3_B.ManualSwitch =
      youbot_motor_joint3_P.Saturation_LowerSat;
  } else {
    /* Saturate: '<S5>/Saturation' */
    youbot_motor_joint3_B.ManualSwitch = rtb_Motorelec;
  }

  /* End of ManualSwitch: '<S5>/Manual Switch' */

  /* Sum: '<Root>/Sum1' */
  rtb_Motorelec = youbot_motor_joint3_B.ManualSwitch2 -
    youbot_motor_joint3_B.ManualSwitch;

  /* Sum: '<S3>/Sum' incorporates:
   *  Gain: '<S3>/Proportional Gain'
   *  Integrator: '<S3>/Integrator'
   */
  youbot_motor_joint3_B.Sum = youbot_motor_joint3_P.K_cur * rtb_Motorelec +
    youbot_motor_joint3_X.Integrator_CSTATE_c;
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    /* Constant: '<Root>/Constant' */
    youbot_motor_joint3_B.Constant[0] = youbot_motor_joint3_P.weight[0];
    youbot_motor_joint3_B.Constant[1] = youbot_motor_joint3_P.weight[1];
    youbot_motor_joint3_B.Constant[2] = youbot_motor_joint3_P.weight[2];

    /* Constant: '<S8>/Constant1' */
    youbot_motor_joint3_B.Constant1 = youbot_motor_joint3_P.Constant1_Value;
  }

  /* Trigonometry: '<S8>/Trigonometric Function1' */
  youbot_motor_joint3_B.TrigonometricFunction1 = cos
    (youbot_motor_joint3_B.Integrator);

  /* Trigonometry: '<S8>/Trigonometric Function' */
  youbot_motor_joint3_B.TrigonometricFunction = sin
    (youbot_motor_joint3_B.Integrator);

  /* Sum: '<S2>/Add3' incorporates:
   *  Product: '<S2>/Element product'
   */
  rtb_Add3_idx_0 = youbot_motor_joint3_B.Constant[1] *
    youbot_motor_joint3_B.TrigonometricFunction1 -
    youbot_motor_joint3_B.Constant[2] * youbot_motor_joint3_B.Constant1;
  rtb_Add3_idx_2 = youbot_motor_joint3_B.Constant[0] *
    youbot_motor_joint3_B.Constant1 - youbot_motor_joint3_B.Constant[1] *
    youbot_motor_joint3_B.TrigonometricFunction;

  /* Trigonometry: '<S9>/Trigonometric Function2' */
  if (rtb_Add3_idx_0 > 1.0) {
    rtb_Add3_idx_0 = 1.0;
  } else {
    if (rtb_Add3_idx_0 < -1.0) {
      rtb_Add3_idx_0 = -1.0;
    }
  }

  youbot_motor_joint3_B.TrigonometricFunction2 = asin(rtb_Add3_idx_0);

  /* End of Trigonometry: '<S9>/Trigonometric Function2' */

  /* Trigonometry: '<S9>/Trigonometric Function3' */
  if (rtb_Add3_idx_2 > 1.0) {
    rtb_Add3_idx_2 = 1.0;
  } else {
    if (rtb_Add3_idx_2 < -1.0) {
      rtb_Add3_idx_2 = -1.0;
    }
  }

  youbot_motor_joint3_B.TrigonometricFunction3 = acos(rtb_Add3_idx_2);

  /* End of Trigonometry: '<S9>/Trigonometric Function3' */
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
  }

  /* Gain: '<S1>/Integral Gain' */
  youbot_motor_joint3_B.IntegralGain = youbot_motor_joint3_P.Ki *
    rtb_ManualSwitch1;

  /* Gain: '<S3>/Integral Gain' */
  youbot_motor_joint3_B.IntegralGain_p = youbot_motor_joint3_P.Ki_cur *
    rtb_Motorelec;

  /* ManualSwitch: '<Root>/Manual Switch1' */
  if (youbot_motor_joint3_P.ManualSwitch1_CurrentSetting == 1) {
    rtb_ManualSwitch1 = youbot_motor_joint3_B.TrigonometricFunction2;
  }

  /* End of ManualSwitch: '<Root>/Manual Switch1' */

  /* Sum: '<S5>/Sum2' incorporates:
   *  Gain: '<S5>/Gain1'
   *  Gain: '<S5>/Gain2'
   */
  youbot_motor_joint3_B.Sum2 = youbot_motor_joint3_P.Kt *
    youbot_motor_joint3_B.ManualSwitch - 1.0 / youbot_motor_joint3_P.N *
    rtb_ManualSwitch1;

  /* Sum: '<S5>/Sum3' incorporates:
   *  Gain: '<S5>/Speed constant'
   */
  youbot_motor_joint3_B.Sum3 = youbot_motor_joint3_B.Sum -
    youbot_motor_joint3_P.Ke * rtb_Motormech;

  /* Gain: '<S7>/Integral Gain' */
  youbot_motor_joint3_B.IntegralGain_d = youbot_motor_joint3_P.Kvi * rtb_Sum_d;
  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(youbot_motor_joint3_M->rtwLogInfo,
                        (youbot_motor_joint3_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
      /* Update for UnitDelay: '<S10>/UD' */
      youbot_motor_joint3_DW.UD_DSTATE = rtb_TSamp;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(youbot_motor_joint3_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(youbot_motor_joint3_M)!=-1) &&
          !((rtmGetTFinal(youbot_motor_joint3_M)-
             (((youbot_motor_joint3_M->Timing.clockTick1+
                youbot_motor_joint3_M->Timing.clockTickH1* 4294967296.0)) *
              5.0E-5)) > (((youbot_motor_joint3_M->Timing.clockTick1+
                            youbot_motor_joint3_M->Timing.clockTickH1*
                            4294967296.0)) * 5.0E-5) * (DBL_EPSILON))) {
        rtmSetErrorStatus(youbot_motor_joint3_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&youbot_motor_joint3_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++youbot_motor_joint3_M->Timing.clockTick0)) {
      ++youbot_motor_joint3_M->Timing.clockTickH0;
    }

    youbot_motor_joint3_M->Timing.t[0] = rtsiGetSolverStopTime
      (&youbot_motor_joint3_M->solverInfo);

    {
      /* Update absolute timer for sample time: [5.0E-5s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 5.0E-5, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      youbot_motor_joint3_M->Timing.clockTick1++;
      if (!youbot_motor_joint3_M->Timing.clockTick1) {
        youbot_motor_joint3_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void youbot_motor_joint3_derivatives(void)
{
  XDot_youbot_motor_joint3_T *_rtXdot;
  _rtXdot = ((XDot_youbot_motor_joint3_T *) youbot_motor_joint3_M->derivs);

  /* Derivatives for TransferFcn: '<S5>/Motor mech' */
  _rtXdot->Motormech_CSTATE = 0.0;
  _rtXdot->Motormech_CSTATE += youbot_motor_joint3_P.Motormech_A *
    youbot_motor_joint3_X.Motormech_CSTATE;
  _rtXdot->Motormech_CSTATE += youbot_motor_joint3_B.Sum2;

  /* Derivatives for TransferFcn: '<S4>/integrator' */
  _rtXdot->integrator_CSTATE = 0.0;
  _rtXdot->integrator_CSTATE += youbot_motor_joint3_P.integrator_A *
    youbot_motor_joint3_X.integrator_CSTATE;

  /* Derivatives for Integrator: '<S1>/Integrator' */
  _rtXdot->Integrator_CSTATE = youbot_motor_joint3_B.IntegralGain;

  /* Derivatives for Integrator: '<S5>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = youbot_motor_joint3_B.Gain;

  /* Derivatives for Integrator: '<S7>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = youbot_motor_joint3_B.IntegralGain_d;

  /* Derivatives for TransferFcn: '<S5>/Motor elec' */
  _rtXdot->Motorelec_CSTATE = 0.0;
  _rtXdot->Motorelec_CSTATE += youbot_motor_joint3_P.Motorelec_A *
    youbot_motor_joint3_X.Motorelec_CSTATE;
  _rtXdot->Motorelec_CSTATE += youbot_motor_joint3_B.Sum3;

  /* Derivatives for Integrator: '<S3>/Integrator' */
  _rtXdot->Integrator_CSTATE_c = youbot_motor_joint3_B.IntegralGain_p;
}

/* Model initialize function */
void youbot_motor_joint3_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)youbot_motor_joint3_M, 0,
                sizeof(RT_MODEL_youbot_motor_joint3_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&youbot_motor_joint3_M->solverInfo,
                          &youbot_motor_joint3_M->Timing.simTimeStep);
    rtsiSetTPtr(&youbot_motor_joint3_M->solverInfo, &rtmGetTPtr
                (youbot_motor_joint3_M));
    rtsiSetStepSizePtr(&youbot_motor_joint3_M->solverInfo,
                       &youbot_motor_joint3_M->Timing.stepSize0);
    rtsiSetdXPtr(&youbot_motor_joint3_M->solverInfo,
                 &youbot_motor_joint3_M->derivs);
    rtsiSetContStatesPtr(&youbot_motor_joint3_M->solverInfo, (real_T **)
                         &youbot_motor_joint3_M->contStates);
    rtsiSetNumContStatesPtr(&youbot_motor_joint3_M->solverInfo,
      &youbot_motor_joint3_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&youbot_motor_joint3_M->solverInfo,
      &youbot_motor_joint3_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&youbot_motor_joint3_M->solverInfo,
      &youbot_motor_joint3_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&youbot_motor_joint3_M->solverInfo,
      &youbot_motor_joint3_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&youbot_motor_joint3_M->solverInfo,
                          (&rtmGetErrorStatus(youbot_motor_joint3_M)));
    rtsiSetRTModelPtr(&youbot_motor_joint3_M->solverInfo, youbot_motor_joint3_M);
  }

  rtsiSetSimTimeStep(&youbot_motor_joint3_M->solverInfo, MAJOR_TIME_STEP);
  youbot_motor_joint3_M->intgData.y = youbot_motor_joint3_M->odeY;
  youbot_motor_joint3_M->intgData.f[0] = youbot_motor_joint3_M->odeF[0];
  youbot_motor_joint3_M->intgData.f[1] = youbot_motor_joint3_M->odeF[1];
  youbot_motor_joint3_M->intgData.f[2] = youbot_motor_joint3_M->odeF[2];
  youbot_motor_joint3_M->contStates = ((X_youbot_motor_joint3_T *)
    &youbot_motor_joint3_X);
  rtsiSetSolverData(&youbot_motor_joint3_M->solverInfo, (void *)
                    &youbot_motor_joint3_M->intgData);
  rtsiSetSolverName(&youbot_motor_joint3_M->solverInfo,"ode3");
  rtmSetTPtr(youbot_motor_joint3_M, &youbot_motor_joint3_M->Timing.tArray[0]);
  rtmSetTFinal(youbot_motor_joint3_M, 10.0);
  youbot_motor_joint3_M->Timing.stepSize0 = 5.0E-5;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = NULL;
    youbot_motor_joint3_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(youbot_motor_joint3_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(youbot_motor_joint3_M->rtwLogInfo, (NULL));
    rtliSetLogT(youbot_motor_joint3_M->rtwLogInfo, "tout");
    rtliSetLogX(youbot_motor_joint3_M->rtwLogInfo, "");
    rtliSetLogXFinal(youbot_motor_joint3_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(youbot_motor_joint3_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(youbot_motor_joint3_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(youbot_motor_joint3_M->rtwLogInfo, 0);
    rtliSetLogDecimation(youbot_motor_joint3_M->rtwLogInfo, 1);
    rtliSetLogY(youbot_motor_joint3_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(youbot_motor_joint3_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(youbot_motor_joint3_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &youbot_motor_joint3_B), 0,
                sizeof(B_youbot_motor_joint3_T));

  /* states (continuous) */
  {
    (void) memset((void *)&youbot_motor_joint3_X, 0,
                  sizeof(X_youbot_motor_joint3_T));
  }

  /* states (dwork) */
  (void) memset((void *)&youbot_motor_joint3_DW, 0,
                sizeof(DW_youbot_motor_joint3_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(youbot_motor_joint3_M->rtwLogInfo, 0.0,
    rtmGetTFinal(youbot_motor_joint3_M), youbot_motor_joint3_M->Timing.stepSize0,
    (&rtmGetErrorStatus(youbot_motor_joint3_M)));

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator1' */
  youbot_motor_joint3_DW.clockTickCounter = 0;

  /* Start for DiscretePulseGenerator: '<Root>/Pulse Generator' */
  youbot_motor_joint3_DW.clockTickCounter_b = 0;

  /* InitializeConditions for TransferFcn: '<S5>/Motor mech' */
  youbot_motor_joint3_X.Motormech_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S4>/integrator' */
  youbot_motor_joint3_X.integrator_CSTATE = 0.0;

  /* InitializeConditions for UnitDelay: '<S10>/UD' */
  youbot_motor_joint3_DW.UD_DSTATE =
    youbot_motor_joint3_P.DiscreteDerivative_ICPrevScaled;

  /* InitializeConditions for Integrator: '<S1>/Integrator' */
  youbot_motor_joint3_X.Integrator_CSTATE = youbot_motor_joint3_P.Integrator_IC;

  /* InitializeConditions for Integrator: '<S5>/Integrator' */
  youbot_motor_joint3_X.Integrator_CSTATE_p =
    youbot_motor_joint3_P.Integrator_IC_l;

  /* InitializeConditions for Integrator: '<S7>/Integrator' */
  youbot_motor_joint3_X.Integrator_CSTATE_d =
    youbot_motor_joint3_P.Integrator_IC_i;

  /* InitializeConditions for TransferFcn: '<S5>/Motor elec' */
  youbot_motor_joint3_X.Motorelec_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S3>/Integrator' */
  youbot_motor_joint3_X.Integrator_CSTATE_c =
    youbot_motor_joint3_P.Integrator_IC_k;
}

/* Model terminate function */
void youbot_motor_joint3_terminate(void)
{
  /* (no terminate code required) */
}
