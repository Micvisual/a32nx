#include "ap_hdg_track.h"
#include "ap_hdg_track_private.h"

const uint8_T ap_hdg_track_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T ap_hdg_track_IN_any = 1U;
const uint8_T ap_hdg_track_IN_left = 2U;
const uint8_T ap_hdg_track_IN_right = 3U;
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[], uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

real_T rt_modd(real_T u0, real_T u1)
{
  real_T y;
  boolean_T yEq;
  real_T q;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else {
    y = std::fmod(u0, u1);
    yEq = (y == 0.0);
    if ((!yEq) && (u1 > std::floor(u1))) {
      q = std::abs(u0 / u1);
      yEq = (std::abs(q - std::floor(q + 0.5)) <= DBL_EPSILON * q);
    }

    if (yEq) {
      y = 0.0;
    } else {
      if ((u0 < 0.0) != (u1 < 0.0)) {
        y += u1;
      }
    }
  }

  return y;
}

void ap_hdg_trackModelClass::step()
{
  real_T rtb_Mod1;
  real_T rtb_Gain;
  real_T rtb_ROLLLIM1;
  real_T tmp;
  real_T tmp_0;
  rtb_ROLLLIM1 = look1_binlxpw(ap_hdg_track_U.V_TAS_kts, ap_hdg_track_P.ROLLLIM1_bp01Data,
    ap_hdg_track_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1 = rt_modd((ap_hdg_track_U.Psi_c_deg - (ap_hdg_track_U.Psi_deg + ap_hdg_track_P.Constant3_Value)) +
                     ap_hdg_track_P.Constant3_Value, ap_hdg_track_P.Constant3_Value);
  rtb_Gain = rt_modd(ap_hdg_track_P.Constant3_Value - rtb_Mod1, ap_hdg_track_P.Constant3_Value) *
    ap_hdg_track_P.Gain_Gain;
  if (ap_hdg_track_DWork.is_active_c3_ap_hdg_track == 0U) {
    ap_hdg_track_DWork.is_active_c3_ap_hdg_track = 1U;
    ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_any;
    if (std::abs(rtb_Gain) < std::abs(rtb_Mod1)) {
      rtb_Mod1 = rtb_Gain;
    }
  } else {
    switch (ap_hdg_track_DWork.is_c3_ap_hdg_track) {
     case ap_hdg_track_IN_any:
      tmp = std::abs(rtb_Mod1);
      tmp_0 = std::abs(rtb_Gain);
      if ((ap_hdg_track_P.Constant4_Value == 0.0) && (tmp < tmp_0) && (tmp > 10.0)) {
        ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_right;
      } else if ((ap_hdg_track_P.Constant4_Value == 0.0) && (tmp_0 < tmp) && (tmp_0 > 10.0)) {
        ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_left;
        rtb_Mod1 = rtb_Gain;
      } else {
        if (tmp_0 < tmp) {
          rtb_Mod1 = rtb_Gain;
        }
      }
      break;

     case ap_hdg_track_IN_left:
      tmp = std::abs(rtb_Gain);
      tmp_0 = std::abs(rtb_Mod1);
      if ((ap_hdg_track_P.Constant4_Value != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_any;
        if (tmp < tmp_0) {
          rtb_Mod1 = rtb_Gain;
        }
      } else {
        rtb_Mod1 = rtb_Gain;
      }
      break;

     default:
      tmp = std::abs(rtb_Gain);
      tmp_0 = std::abs(rtb_Mod1);
      if ((ap_hdg_track_P.Constant4_Value != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_any;
        if (tmp < tmp_0) {
          rtb_Mod1 = rtb_Gain;
        }
      }
      break;
    }
  }

  rtb_Mod1 *= ap_hdg_track_P.Gain_Gain_k;
  if (rtb_Mod1 > rtb_ROLLLIM1) {
    rtb_Mod1 = rtb_ROLLLIM1;
  } else {
    rtb_ROLLLIM1 *= ap_hdg_track_P.Gain1_Gain;
    if (rtb_Mod1 < rtb_ROLLLIM1) {
      rtb_Mod1 = rtb_ROLLLIM1;
    }
  }

  ap_hdg_track_Y.Phi_FD = rtb_Mod1;
  if (!ap_hdg_track_U.enabled) {
    ap_hdg_track_DWork.icLoad = 1U;
  }

  if (ap_hdg_track_DWork.icLoad != 0) {
    ap_hdg_track_DWork.Delay_DSTATE = ap_hdg_track_P.Gain2_Gain * ap_hdg_track_U.Phi_deg;
  }

  rtb_Mod1 -= ap_hdg_track_DWork.Delay_DSTATE;
  rtb_ROLLLIM1 = ap_hdg_track_P.Constant1_Value * ap_hdg_track_U.Ts;
  if (rtb_Mod1 >= rtb_ROLLLIM1) {
    rtb_Mod1 = rtb_ROLLLIM1;
  }

  rtb_ROLLLIM1 = ap_hdg_track_P.Gain1_Gain_d * ap_hdg_track_P.Constant1_Value * ap_hdg_track_U.Ts;
  if (rtb_Mod1 > rtb_ROLLLIM1) {
    rtb_ROLLLIM1 = rtb_Mod1;
  }

  ap_hdg_track_DWork.Delay_DSTATE += rtb_ROLLLIM1;
  rtb_ROLLLIM1 = ap_hdg_track_U.Ts * ap_hdg_track_P.LagFilter_C1;
  rtb_Mod1 = rtb_ROLLLIM1 + ap_hdg_track_P.Constant_Value;
  ap_hdg_track_Y.Phi_AP = 1.0 / rtb_Mod1 * (ap_hdg_track_P.Constant_Value - rtb_ROLLLIM1) * ap_hdg_track_Y.Phi_AP +
    (ap_hdg_track_DWork.Delay_DSTATE + ap_hdg_track_DWork.Delay_DSTATE_a) * (rtb_ROLLLIM1 / rtb_Mod1);
  ap_hdg_track_DWork.icLoad = 0U;
  ap_hdg_track_DWork.Delay_DSTATE_a = ap_hdg_track_DWork.Delay_DSTATE;
}

void ap_hdg_trackModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_hdg_track_DWork), 0,
                     sizeof(D_Work_ap_hdg_track_T));
  (void)std::memset(&ap_hdg_track_U, 0, sizeof(ExternalInputs_ap_hdg_track_T));
  (void) std::memset(static_cast<void *>(&ap_hdg_track_Y), 0,
                     sizeof(ExternalOutputs_ap_hdg_track_T));
  ap_hdg_track_DWork.icLoad = 1U;
  ap_hdg_track_DWork.Delay_DSTATE_a = ap_hdg_track_P.Delay_InitialCondition;
  ap_hdg_track_Y.Phi_AP = ap_hdg_track_P.Delay1_InitialCondition;
  ap_hdg_track_DWork.is_active_c3_ap_hdg_track = 0U;
  ap_hdg_track_DWork.is_c3_ap_hdg_track = ap_hdg_track_IN_NO_ACTIVE_CHILD;
}

void ap_hdg_trackModelClass::terminate()
{
}

ap_hdg_trackModelClass::ap_hdg_trackModelClass()
{
}

ap_hdg_trackModelClass::~ap_hdg_trackModelClass()
{
}
