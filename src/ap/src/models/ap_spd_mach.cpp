#include "ap_spd_mach.h"
#include "ap_spd_mach_private.h"

void ap_spd_machModelClass::step()
{
  real_T rtb_Sum_i;
  real_T rtb_Gain;
  real_T rtb_Sum4;
  real_T rtb_Sum3_f;
  real_T rtb_Gain1_a;
  rtb_Gain = ap_spd_mach_P.DiscreteDerivativeVariableTs_Gain * ap_spd_mach_U.V_IAS_kts;
  rtb_Sum4 = (rtb_Gain - ap_spd_mach_DWork.Delay_DSTATE) / ap_spd_mach_U.Ts * ap_spd_mach_P.Gain3_Gain +
    ap_spd_mach_U.V_IAS_kts;
  rtb_Sum_i = ap_spd_mach_U.Ts * ap_spd_mach_P.LagFilter1_C1;
  rtb_Sum3_f = rtb_Sum_i + ap_spd_mach_P.Constant_Value_d;
  ap_spd_mach_DWork.Delay1_DSTATE = 1.0 / rtb_Sum3_f * (ap_spd_mach_P.Constant_Value_d - rtb_Sum_i) *
    ap_spd_mach_DWork.Delay1_DSTATE + (rtb_Sum4 + ap_spd_mach_DWork.Delay_DSTATE_f) * (rtb_Sum_i / rtb_Sum3_f);
  rtb_Sum_i = ap_spd_mach_P.Gain2_Gain * ap_spd_mach_U.Theta_deg;
  rtb_Sum3_f = ap_spd_mach_DWork.Delay1_DSTATE - ap_spd_mach_U.V_c_kts;
  if (rtb_Sum3_f > ap_spd_mach_P.Saturation_UpperSat) {
    rtb_Sum3_f = ap_spd_mach_P.Saturation_UpperSat;
  } else {
    if (rtb_Sum3_f < ap_spd_mach_P.Saturation_LowerSat) {
      rtb_Sum3_f = ap_spd_mach_P.Saturation_LowerSat;
    }
  }

  rtb_Sum3_f = ap_spd_mach_P.Gain1_Gain_a * rtb_Sum3_f + rtb_Sum_i;
  if (rtb_Sum3_f > ap_spd_mach_P.Constant_Value) {
    rtb_Sum3_f = ap_spd_mach_P.Constant_Value;
  } else {
    rtb_Gain1_a = ap_spd_mach_P.Gain1_Gain * ap_spd_mach_P.Constant_Value;
    if (rtb_Sum3_f < rtb_Gain1_a) {
      rtb_Sum3_f = rtb_Gain1_a;
    }
  }

  ap_spd_mach_Y.Theta_FD = rtb_Sum3_f;
  if (!ap_spd_mach_U.enabled) {
    ap_spd_mach_DWork.icLoad = 1U;
  }

  if (ap_spd_mach_DWork.icLoad != 0) {
    ap_spd_mach_DWork.Delay_DSTATE_fp = rtb_Sum_i;
  }

  rtb_Sum3_f -= ap_spd_mach_DWork.Delay_DSTATE_fp;
  rtb_Sum_i = ap_spd_mach_P.Constant2_Value * ap_spd_mach_U.Ts;
  if (rtb_Sum3_f >= rtb_Sum_i) {
    rtb_Sum3_f = rtb_Sum_i;
  }

  rtb_Sum_i = ap_spd_mach_P.Gain1_Gain_h * ap_spd_mach_P.Constant2_Value * ap_spd_mach_U.Ts;
  if (rtb_Sum3_f > rtb_Sum_i) {
    rtb_Sum_i = rtb_Sum3_f;
  }

  ap_spd_mach_DWork.Delay_DSTATE_fp += rtb_Sum_i;
  rtb_Sum3_f = ap_spd_mach_U.Ts * ap_spd_mach_P.LagFilter_C1;
  rtb_Sum_i = rtb_Sum3_f + ap_spd_mach_P.Constant_Value_k;
  ap_spd_mach_Y.Theta_AP = 1.0 / rtb_Sum_i * (ap_spd_mach_P.Constant_Value_k - rtb_Sum3_f) * ap_spd_mach_Y.Theta_AP +
    (ap_spd_mach_DWork.Delay_DSTATE_fp + ap_spd_mach_DWork.Delay_DSTATE_a) * (rtb_Sum3_f / rtb_Sum_i);
  ap_spd_mach_DWork.Delay_DSTATE = rtb_Gain;
  ap_spd_mach_DWork.Delay_DSTATE_f = rtb_Sum4;
  ap_spd_mach_DWork.icLoad = 0U;
  ap_spd_mach_DWork.Delay_DSTATE_a = ap_spd_mach_DWork.Delay_DSTATE_fp;
}

void ap_spd_machModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_spd_mach_DWork), 0,
                     sizeof(D_Work_ap_spd_mach_T));
  (void)std::memset(&ap_spd_mach_U, 0, sizeof(ExternalInputs_ap_spd_mach_T));
  (void) std::memset(static_cast<void *>(&ap_spd_mach_Y), 0,
                     sizeof(ExternalOutputs_ap_spd_mach_T));
  ap_spd_mach_DWork.Delay_DSTATE = ap_spd_mach_P.DiscreteDerivativeVariableTs_InitialCondition;
  ap_spd_mach_DWork.Delay_DSTATE_f = ap_spd_mach_P.Delay_InitialCondition;
  ap_spd_mach_DWork.Delay1_DSTATE = ap_spd_mach_P.Delay1_InitialCondition;
  ap_spd_mach_DWork.icLoad = 1U;
  ap_spd_mach_DWork.Delay_DSTATE_a = ap_spd_mach_P.Delay_InitialCondition_g;
  ap_spd_mach_Y.Theta_AP = ap_spd_mach_P.Delay1_InitialCondition_p;
}

void ap_spd_machModelClass::terminate()
{
}

ap_spd_machModelClass::ap_spd_machModelClass()
{
}

ap_spd_machModelClass::~ap_spd_machModelClass()
{
}
