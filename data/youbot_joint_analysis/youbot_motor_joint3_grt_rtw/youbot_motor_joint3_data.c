/*
 * youbot_motor_joint3_data.c
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

/* Block parameters (auto storage) */
P_youbot_motor_joint3_T youbot_motor_joint3_P = {
  /* Variable: B_env
   * Referenced by: '<S4>/Env. damping'
   */
  1.8,

  /* Variable: I_env
   * Referenced by: '<S4>/Env. inertia'
   */
  0.1,

  /* Variable: K
   * Referenced by: '<S1>/Proportional Gain'
   */
  10.0,

  /* Variable: K_cur
   * Referenced by: '<S3>/Proportional Gain'
   */
  11.71875,

  /* Variable: K_env
   * Referenced by: '<S4>/Env. stiffness'
   */
  25.0,

  /* Variable: K_pos
   * Referenced by: '<S6>/Proportional Gain'
   */
  3.90625,

  /* Variable: Ke
   * Referenced by: '<S5>/Speed constant'
   */
  0.0335,

  /* Variable: Ki
   * Referenced by: '<S1>/Integral Gain'
   */
  1.0,

  /* Variable: Ki_cur
   * Referenced by: '<S3>/Integral Gain'
   */
  0.011444091796875,

  /* Variable: Kt
   * Referenced by: '<S5>/Gain1'
   */
  0.0335,

  /* Variable: Kv
   * Referenced by: '<S7>/Proportional Gain'
   */
  1.171875,

  /* Variable: Kvi
   * Referenced by: '<S7>/Integral Gain'
   */
  0.091552734375,

  /* Variable: N
   * Referenced by:
   *   '<S5>/Gain'
   *   '<S5>/Gain2'
   */
  100.0,

  /* Variable: weight
   * Referenced by: '<Root>/Constant'
   */
  { 0.0, 0.0, 26.192700000000002 },

  /* Mask Parameter: DiscreteDerivative_ICPrevScaled
   * Referenced by: '<S10>/UD'
   */
  0.0,

  /* Expression: 0.5
   * Referenced by: '<S5>/Saturation'
   */
  0.5,

  /* Expression: -0.5
   * Referenced by: '<S5>/Saturation'
   */
  -0.5,

  /* Computed Parameter: Motormech_A
   * Referenced by: '<S5>/Motor mech'
   */
  -0.0011384938277096146,

  /* Computed Parameter: Motormech_C
   * Referenced by: '<S5>/Motor mech'
   */
  49409.75257726344,

  /* Expression: 1
   * Referenced by: '<Root>/Pulse Generator1'
   */
  1.0,

  /* Computed Parameter: PulseGenerator1_Period
   * Referenced by: '<Root>/Pulse Generator1'
   */
  20000.0,

  /* Computed Parameter: PulseGenerator1_Duty
   * Referenced by: '<Root>/Pulse Generator1'
   */
  10000.0,

  /* Expression: 0
   * Referenced by: '<Root>/Pulse Generator1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S4>/x0'
   */
  0.0,

  /* Computed Parameter: integrator_A
   * Referenced by: '<S4>/integrator'
   */
  -0.0,

  /* Computed Parameter: integrator_C
   * Referenced by: '<S4>/integrator'
   */
  1.0,

  /* Computed Parameter: TSamp_WtEt
   * Referenced by: '<S10>/TSamp'
   */
  20000.0,

  /* Expression: InitialConditionForIntegrator
   * Referenced by: '<S1>/Integrator'
   */
  0.0,

  /* Expression: 90/180*pi
   * Referenced by: '<Root>/Pulse Generator'
   */
  1.5707963267948966,

  /* Computed Parameter: PulseGenerator_Period
   * Referenced by: '<Root>/Pulse Generator'
   */
  200000.0,

  /* Computed Parameter: PulseGenerator_Duty
   * Referenced by: '<Root>/Pulse Generator'
   */
  100000.0,

  /* Expression: 0
   * Referenced by: '<Root>/Pulse Generator'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S5>/Integrator'
   */
  0.0,

  /* Expression: InitialConditionForIntegrator
   * Referenced by: '<S7>/Integrator'
   */
  0.0,

  /* Computed Parameter: Motorelec_A
   * Referenced by: '<S5>/Motor elec'
   */
  -1706.8062827225131,

  /* Computed Parameter: Motorelec_C
   * Referenced by: '<S5>/Motor elec'
   */
  1745.2006980802792,

  /* Expression: InitialConditionForIntegrator
   * Referenced by: '<S3>/Integrator'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S8>/Constant1'
   */
  0.0,

  /* Computed Parameter: ManualSwitch_CurrentSetting
   * Referenced by: '<Root>/Manual Switch'
   */
  0U,

  /* Computed Parameter: ManualSwitch2_CurrentSetting
   * Referenced by: '<Root>/Manual Switch2'
   */
  0U,

  /* Computed Parameter: ManualSwitch_CurrentSetting_a
   * Referenced by: '<S5>/Manual Switch'
   */
  1U,

  /* Computed Parameter: ManualSwitch1_CurrentSetting
   * Referenced by: '<Root>/Manual Switch1'
   */
  1U
};
