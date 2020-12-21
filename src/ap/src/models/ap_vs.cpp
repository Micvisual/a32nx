#include "ap_vs.h"
#include "ap_vs_private.h"

void ap_vsModelClass::step()
{
  real_T rtb_ftmintoms;
  real_T rtb_Gain2;
  real_T rtb_Sum3;
  rtb_Gain2 = ap_vs_P.Gain2_Gain * ap_vs_U.Theta_deg;
  rtb_Sum3 = ap_vs_P.kntoms_Gain * ap_vs_U.V_TAS_kts;
  rtb_ftmintoms = (ap_vs_U.H_c_ft - std::sin((rtb_Gain2 - std::cos(ap_vs_P.Gain1_Gain_m * ap_vs_U.Phi_deg) *
    ap_vs_U.alpha_deg) * ap_vs_P.Gain1_Gain_i) * rtb_Sum3 * ap_vs_P.msftmin_Gain) * ap_vs_P.ftmintoms_Gain;
  if (ap_vs_DWork.IC_FirstOutputTime) {
    ap_vs_DWork.IC_FirstOutputTime = false;
    rtb_Sum3 = ap_vs_P.IC_Value;
  }

  rtb_Sum3 = rtb_ftmintoms / rtb_Sum3;
  if (rtb_Sum3 > 1.0) {
    rtb_Sum3 = 1.0;
  } else {
    if (rtb_Sum3 < -1.0) {
      rtb_Sum3 = -1.0;
    }
  }

  rtb_Sum3 = ap_vs_P.Gain_Gain * std::asin(rtb_Sum3) + rtb_Gain2;
  if (rtb_Sum3 > ap_vs_P.Constant1_Value) {
    rtb_Sum3 = ap_vs_P.Constant1_Value;
  } else {
    rtb_ftmintoms = ap_vs_P.Gain1_Gain * ap_vs_P.Constant1_Value;
    if (rtb_Sum3 < rtb_ftmintoms) {
      rtb_Sum3 = rtb_ftmintoms;
    }
  }

  ap_vs_Y.Theta_FD = rtb_Sum3;
  if (!ap_vs_U.enabled) {
    ap_vs_DWork.icLoad = 1U;
  }

  if (ap_vs_DWork.icLoad != 0) {
    ap_vs_DWork.Delay_DSTATE = rtb_Gain2;
  }

  rtb_Sum3 -= ap_vs_DWork.Delay_DSTATE;
  rtb_Gain2 = ap_vs_P.Constant2_Value * ap_vs_U.Ts;
  if (rtb_Sum3 >= rtb_Gain2) {
    rtb_Sum3 = rtb_Gain2;
  }

  rtb_Gain2 = ap_vs_P.Gain1_Gain_o * ap_vs_P.Constant2_Value * ap_vs_U.Ts;
  if (rtb_Sum3 > rtb_Gain2) {
    rtb_Gain2 = rtb_Sum3;
  }

  ap_vs_DWork.Delay_DSTATE += rtb_Gain2;
  rtb_Gain2 = ap_vs_U.Ts * ap_vs_P.LagFilter_C1;
  rtb_Sum3 = rtb_Gain2 + ap_vs_P.Constant_Value;
  ap_vs_Y.Theta_AP = 1.0 / rtb_Sum3 * (ap_vs_P.Constant_Value - rtb_Gain2) * ap_vs_Y.Theta_AP +
    (ap_vs_DWork.Delay_DSTATE + ap_vs_DWork.Delay_DSTATE_j) * (rtb_Gain2 / rtb_Sum3);
  ap_vs_DWork.icLoad = 0U;
  ap_vs_DWork.Delay_DSTATE_j = ap_vs_DWork.Delay_DSTATE;
}

void ap_vsModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_vs_DWork), 0,
                     sizeof(D_Work_ap_vs_T));
  (void)std::memset(&ap_vs_U, 0, sizeof(ExternalInputs_ap_vs_T));
  (void) std::memset(static_cast<void *>(&ap_vs_Y), 0,
                     sizeof(ExternalOutputs_ap_vs_T));
  ap_vs_DWork.IC_FirstOutputTime = true;
  ap_vs_DWork.icLoad = 1U;
  ap_vs_DWork.Delay_DSTATE_j = ap_vs_P.Delay_InitialCondition;
  ap_vs_Y.Theta_AP = ap_vs_P.Delay1_InitialCondition;
}

void ap_vsModelClass::terminate()
{
}

ap_vsModelClass::ap_vsModelClass()
{
}

ap_vsModelClass::~ap_vsModelClass()
{
}
