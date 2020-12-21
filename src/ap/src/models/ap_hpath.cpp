#include "ap_hpath.h"
#include "ap_hpath_private.h"

const uint8_T ap_hpath_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T ap_hpath_IN_any = 1U;
const uint8_T ap_hpath_IN_left = 2U;
const uint8_T ap_hpath_IN_right = 3U;
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

void ap_hpathModelClass::step()
{
  real_T rtb_Mod1;
  real_T rtb_Gain;
  real_T rtb_ROLLLIM1;
  real_T tmp;
  real_T tmp_0;
  rtb_ROLLLIM1 = look1_binlxpw(ap_hpath_U.V_TAS_kts, ap_hpath_P.ROLLLIM1_bp01Data, ap_hpath_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1 = ap_hpath_P.Gain_Gain * ap_hpath_U.XTK_nm;
  if (rtb_Mod1 > ap_hpath_P.Saturation_UpperSat) {
    rtb_Mod1 = ap_hpath_P.Saturation_UpperSat;
  } else {
    if (rtb_Mod1 < ap_hpath_P.Saturation_LowerSat) {
      rtb_Mod1 = ap_hpath_P.Saturation_LowerSat;
    }
  }

  rtb_Mod1 = rt_modd((rt_modd((rtb_Mod1 + ap_hpath_U.TAE_deg) * ap_hpath_P.Gain1_Gain_a + (ap_hpath_U.Psi_track_deg +
    ap_hpath_P.Constant_Value), ap_hpath_P.Constant_Value) - (ap_hpath_U.Psi_track_deg + ap_hpath_P.Constant3_Value)) +
                     ap_hpath_P.Constant3_Value, ap_hpath_P.Constant3_Value);
  rtb_Gain = rt_modd(ap_hpath_P.Constant3_Value - rtb_Mod1, ap_hpath_P.Constant3_Value) * ap_hpath_P.Gain_Gain_p;
  if (ap_hpath_DWork.is_active_c3_ap_hpath == 0U) {
    ap_hpath_DWork.is_active_c3_ap_hpath = 1U;
    ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_any;
    if (std::abs(rtb_Gain) < std::abs(rtb_Mod1)) {
      rtb_Mod1 = rtb_Gain;
    }
  } else {
    switch (ap_hpath_DWork.is_c3_ap_hpath) {
     case ap_hpath_IN_any:
      tmp = std::abs(rtb_Mod1);
      tmp_0 = std::abs(rtb_Gain);
      if ((ap_hpath_P.Constant4_Value == 0.0) && (tmp < tmp_0) && (tmp > 10.0)) {
        ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_right;
      } else if ((ap_hpath_P.Constant4_Value == 0.0) && (tmp_0 < tmp) && (tmp_0 > 10.0)) {
        ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_left;
        rtb_Mod1 = rtb_Gain;
      } else {
        if (tmp_0 < tmp) {
          rtb_Mod1 = rtb_Gain;
        }
      }
      break;

     case ap_hpath_IN_left:
      tmp = std::abs(rtb_Gain);
      tmp_0 = std::abs(rtb_Mod1);
      if ((ap_hpath_P.Constant4_Value != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_any;
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
      if ((ap_hpath_P.Constant4_Value != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_any;
        if (tmp < tmp_0) {
          rtb_Mod1 = rtb_Gain;
        }
      }
      break;
    }
  }

  rtb_Mod1 *= ap_hpath_P.Gain_Gain_b;
  if (rtb_Mod1 > rtb_ROLLLIM1) {
    rtb_Mod1 = rtb_ROLLLIM1;
  } else {
    rtb_ROLLLIM1 *= ap_hpath_P.Gain1_Gain;
    if (rtb_Mod1 < rtb_ROLLLIM1) {
      rtb_Mod1 = rtb_ROLLLIM1;
    }
  }

  ap_hpath_Y.Phi_FD = rtb_Mod1;
  if (!ap_hpath_U.enabled) {
    ap_hpath_DWork.icLoad = 1U;
  }

  if (ap_hpath_DWork.icLoad != 0) {
    ap_hpath_DWork.Delay_DSTATE = ap_hpath_P.Gain2_Gain * ap_hpath_U.Phi_deg;
  }

  rtb_Mod1 -= ap_hpath_DWork.Delay_DSTATE;
  rtb_ROLLLIM1 = ap_hpath_P.Constant1_Value * ap_hpath_U.Ts;
  if (rtb_Mod1 >= rtb_ROLLLIM1) {
    rtb_Mod1 = rtb_ROLLLIM1;
  }

  rtb_ROLLLIM1 = ap_hpath_P.Gain1_Gain_n * ap_hpath_P.Constant1_Value * ap_hpath_U.Ts;
  if (rtb_Mod1 > rtb_ROLLLIM1) {
    rtb_ROLLLIM1 = rtb_Mod1;
  }

  ap_hpath_DWork.Delay_DSTATE += rtb_ROLLLIM1;
  rtb_ROLLLIM1 = ap_hpath_U.Ts * ap_hpath_P.LagFilter_C1;
  rtb_Mod1 = rtb_ROLLLIM1 + ap_hpath_P.Constant_Value_h;
  ap_hpath_Y.Phi_AP = 1.0 / rtb_Mod1 * (ap_hpath_P.Constant_Value_h - rtb_ROLLLIM1) * ap_hpath_Y.Phi_AP +
    (ap_hpath_DWork.Delay_DSTATE + ap_hpath_DWork.Delay_DSTATE_f) * (rtb_ROLLLIM1 / rtb_Mod1);
  ap_hpath_DWork.icLoad = 0U;
  ap_hpath_DWork.Delay_DSTATE_f = ap_hpath_DWork.Delay_DSTATE;
}

void ap_hpathModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&ap_hpath_DWork), 0,
                     sizeof(D_Work_ap_hpath_T));
  (void)std::memset(&ap_hpath_U, 0, sizeof(ExternalInputs_ap_hpath_T));
  (void) std::memset(static_cast<void *>(&ap_hpath_Y), 0,
                     sizeof(ExternalOutputs_ap_hpath_T));
  ap_hpath_DWork.icLoad = 1U;
  ap_hpath_DWork.Delay_DSTATE_f = ap_hpath_P.Delay_InitialCondition;
  ap_hpath_Y.Phi_AP = ap_hpath_P.Delay1_InitialCondition;
  ap_hpath_DWork.is_active_c3_ap_hpath = 0U;
  ap_hpath_DWork.is_c3_ap_hpath = ap_hpath_IN_NO_ACTIVE_CHILD;
}

void ap_hpathModelClass::terminate()
{
}

ap_hpathModelClass::ap_hpathModelClass()
{
}

ap_hpathModelClass::~ap_hpathModelClass()
{
}
