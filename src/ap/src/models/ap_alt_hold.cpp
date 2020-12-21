#include "ap_alt_hold.h"
#include "ap_alt_hold_private.h"

void ap_alt_holdModelClass::step()
{
  real_T rtb_ftmintoms;
  real_T rtb_Gain2;
  real_T rtb_Sum3;
  rtb_Gain2 = ap_alt_hold_P.Gain2_Gain * ap_alt_hold_U.Theta_deg;
  rtb_Sum3 = ap_alt_hold_P.kntoms_Gain * ap_alt_hold_U.V_TAS_kts;
  rtb_ftmintoms = (ap_alt_hold_U.H_c_ft - ap_alt_hold_U.H_ft) * ap_alt_hold_P.Gain_Gain;
  if (rtb_ftmintoms > ap_alt_hold_P.Saturation_UpperSat) {
    rtb_ftmintoms = ap_alt_hold_P.Saturation_UpperSat;
  } else {
    if (rtb_ftmintoms < ap_alt_hold_P.Saturation_LowerSat) {
      rtb_ftmintoms = ap_alt_hold_P.Saturation_LowerSat;
    }
  }

  rtb_ftmintoms = (rtb_ftmintoms - std::sin((rtb_Gain2 - std::cos(ap_alt_hold_P.Gain1_Gain_m * ap_alt_hold_U.Phi_deg) *
    ap_alt_hold_U.alpha_deg) * ap_alt_hold_P.Gain1_Gain_i) * rtb_Sum3 * ap_alt_hold_P.msftmin_Gain) *
    ap_alt_hold_P.ftmintoms_Gain;
  if (ap_alt_hold_DWork.IC_FirstOutputTime) {
    ap_alt_hold_DWork.IC_FirstOutputTime = false;
    rtb_Sum3 = ap_alt_hold_P.IC_Value;
  }

  rtb_ftmintoms /= rtb_Sum3;
  if (rtb_ftmintoms > 1.0) {
    rtb_ftmintoms = 1.0;
  } else {
    if (rtb_ftmintoms < -1.0) {
      rtb_ftmintoms = -1.0;
    }
  }

  rtb_Sum3 = ap_alt_hold_P.Gain_Gain_l * std::asin(rtb_ftmintoms) + rtb_Gain2;
  if (rtb_Sum3 > ap_alt_hold_P.Constant1_Value) {
    rtb_Sum3 = ap_alt_hold_P.Constant1_Value;
  } else {
    rtb_ftmintoms = ap_alt_hold_P.Gain1_Gain * ap_alt_hold_P.Constant1_Value;
    if (rtb_Sum3 < rtb_ftmintoms) {
      rtb_Sum3 = rtb_ftmintoms;
    }
  }

  ap_alt_hold_Y.Theta_FD = rtb_Sum3;
  if (!ap_alt_hold_U.enabled) {
    ap_alt_hold_DWork.icLoad = 1U;
  }

  if (ap_alt_hold_DWork.icLoad != 0) {
    ap_alt_hold_DWork.Delay_DSTATE = rtb_Gain2;
  }

  rtb_Sum3 -= ap_alt_hold_DWork.Delay_DSTATE;
  rtb_Gain2 = ap_alt_hold_P.Constant2_Value * ap_alt_hold_U.Ts;
  if (rtb_Sum3 >= rtb_Gain2) {
    rtb_Sum3 = rtb_Gain2;
  }

  rtb_Gain2 = ap_alt_hold_P.Gain1_Gain_o * ap_alt_hold_P.Constant2_Value * ap_alt_hold_U.Ts;
  if (rtb_Sum3 > rtb_Gain2) {
    rtb_Gain2 = rtb_Sum3;
  }

  ap_alt_hold_DWork.Delay_DSTATE += rtb_Gain2;
  rtb_Sum3 = ap_alt_hold_U.Ts * ap_alt_hold_P.LagFilter_C1;
  rtb_Gain2 = rtb_Sum3 + ap_alt_hold_P.Constant_Value;
  ap_alt_hold_Y.Theta_AP = 1.0 / rtb_Gain2 * (ap_alt_hold_P.Constant_Value - rtb_Sum3) * ap_alt_hold_Y.Theta_AP +
    (ap_alt_hold_DWork.Delay_DSTATE + ap_alt_hold_DWork.Delay_DSTATE_j) * (rtb_Sum3 / rtb_Gain2);
  ap_alt_hold_DWork.icLoad = 0U;
  ap_alt_hold_DWork.Delay_DSTATE_j = ap_alt_hold_DWork.Delay_DSTATE;
}

void ap_alt_holdModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_alt_hold_DWork), 0,
                     sizeof(D_Work_ap_alt_hold_T));
  (void)std::memset(&ap_alt_hold_U, 0, sizeof(ExternalInputs_ap_alt_hold_T));
  (void) std::memset(static_cast<void *>(&ap_alt_hold_Y), 0,
                     sizeof(ExternalOutputs_ap_alt_hold_T));
  ap_alt_hold_DWork.IC_FirstOutputTime = true;
  ap_alt_hold_DWork.icLoad = 1U;
  ap_alt_hold_DWork.Delay_DSTATE_j = ap_alt_hold_P.Delay_InitialCondition;
  ap_alt_hold_Y.Theta_AP = ap_alt_hold_P.Delay1_InitialCondition;
}

void ap_alt_holdModelClass::terminate()
{
}

ap_alt_holdModelClass::ap_alt_holdModelClass()
{
}

ap_alt_holdModelClass::~ap_alt_holdModelClass()
{
}
