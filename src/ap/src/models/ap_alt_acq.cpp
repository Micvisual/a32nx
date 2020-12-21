#include "ap_alt_acq.h"
#include "ap_alt_acq_private.h"

void ap_alt_acqModelClass::step()
{
  real_T rtb_ftmintoms;
  real_T rtb_Gain2;
  real_T rtb_Sum3;
  rtb_Gain2 = ap_alt_acq_P.Gain2_Gain * ap_alt_acq_U.Theta_deg;
  rtb_Sum3 = ap_alt_acq_P.kntoms_Gain * ap_alt_acq_U.V_TAS_kts;
  rtb_ftmintoms = ((ap_alt_acq_U.H_c_ft - ap_alt_acq_U.H_ft) * ap_alt_acq_P.Gain_Gain - std::sin((rtb_Gain2 - std::cos
    (ap_alt_acq_P.Gain1_Gain_m * ap_alt_acq_U.Phi_deg) * ap_alt_acq_U.alpha_deg) * ap_alt_acq_P.Gain1_Gain_i) * rtb_Sum3
                   * ap_alt_acq_P.msftmin_Gain) * ap_alt_acq_P.ftmintoms_Gain;
  if (ap_alt_acq_DWork.IC_FirstOutputTime) {
    ap_alt_acq_DWork.IC_FirstOutputTime = false;
    rtb_Sum3 = ap_alt_acq_P.IC_Value;
  }

  rtb_Sum3 = rtb_ftmintoms / rtb_Sum3;
  if (rtb_Sum3 > 1.0) {
    rtb_Sum3 = 1.0;
  } else {
    if (rtb_Sum3 < -1.0) {
      rtb_Sum3 = -1.0;
    }
  }

  rtb_Sum3 = ap_alt_acq_P.Gain_Gain_l * std::asin(rtb_Sum3) + rtb_Gain2;
  if (rtb_Sum3 > ap_alt_acq_P.Constant1_Value) {
    rtb_Sum3 = ap_alt_acq_P.Constant1_Value;
  } else {
    rtb_ftmintoms = ap_alt_acq_P.Gain1_Gain * ap_alt_acq_P.Constant1_Value;
    if (rtb_Sum3 < rtb_ftmintoms) {
      rtb_Sum3 = rtb_ftmintoms;
    }
  }

  ap_alt_acq_Y.Theta_FD = rtb_Sum3;
  if (!ap_alt_acq_U.enabled) {
    ap_alt_acq_DWork.icLoad = 1U;
  }

  if (ap_alt_acq_DWork.icLoad != 0) {
    ap_alt_acq_DWork.Delay_DSTATE = rtb_Gain2;
  }

  rtb_Sum3 -= ap_alt_acq_DWork.Delay_DSTATE;
  rtb_Gain2 = ap_alt_acq_P.Constant2_Value * ap_alt_acq_U.Ts;
  if (rtb_Sum3 >= rtb_Gain2) {
    rtb_Sum3 = rtb_Gain2;
  }

  rtb_Gain2 = ap_alt_acq_P.Gain1_Gain_o * ap_alt_acq_P.Constant2_Value * ap_alt_acq_U.Ts;
  if (rtb_Sum3 > rtb_Gain2) {
    rtb_Gain2 = rtb_Sum3;
  }

  ap_alt_acq_DWork.Delay_DSTATE += rtb_Gain2;
  rtb_Gain2 = ap_alt_acq_U.Ts * ap_alt_acq_P.LagFilter_C1;
  rtb_Sum3 = rtb_Gain2 + ap_alt_acq_P.Constant_Value;
  ap_alt_acq_Y.Theta_AP = 1.0 / rtb_Sum3 * (ap_alt_acq_P.Constant_Value - rtb_Gain2) * ap_alt_acq_Y.Theta_AP +
    (ap_alt_acq_DWork.Delay_DSTATE + ap_alt_acq_DWork.Delay_DSTATE_j) * (rtb_Gain2 / rtb_Sum3);
  ap_alt_acq_DWork.icLoad = 0U;
  ap_alt_acq_DWork.Delay_DSTATE_j = ap_alt_acq_DWork.Delay_DSTATE;
}

void ap_alt_acqModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_alt_acq_DWork), 0,
                     sizeof(D_Work_ap_alt_acq_T));
  (void)std::memset(&ap_alt_acq_U, 0, sizeof(ExternalInputs_ap_alt_acq_T));
  (void) std::memset(static_cast<void *>(&ap_alt_acq_Y), 0,
                     sizeof(ExternalOutputs_ap_alt_acq_T));
  ap_alt_acq_DWork.IC_FirstOutputTime = true;
  ap_alt_acq_DWork.icLoad = 1U;
  ap_alt_acq_DWork.Delay_DSTATE_j = ap_alt_acq_P.Delay_InitialCondition;
  ap_alt_acq_Y.Theta_AP = ap_alt_acq_P.Delay1_InitialCondition;
}

void ap_alt_acqModelClass::terminate()
{
}

ap_alt_acqModelClass::ap_alt_acqModelClass()
{
}

ap_alt_acqModelClass::~ap_alt_acqModelClass()
{
}
