#include "Autopilot.h"
#include "Autopilot_private.h"

const uint8_T Autopilot_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T Autopilot_IN_any = 1U;
const uint8_T Autopilot_IN_left = 2U;
const uint8_T Autopilot_IN_right = 3U;
const uint8_T Autopilot_IN_NO_ACTIVE_CHILD_a = 0U;
const uint8_T Autopilot_IN_OFF = 1U;
const uint8_T Autopilot_IN_ON = 2U;
const uint8_T Autopilot_IN_MANAGED = 1U;
const uint8_T Autopilot_IN_SELECTED = 2U;
const uint8_T Autopilot_IN_ALT_ACQ = 1U;
const uint8_T Autopilot_IN_ALT_HOLD = 2U;
const uint8_T Autopilot_IN_SPD_MACH = 3U;
const uint8_T Autopilot_IN_VS = 4U;
const uint8_T Autopilot_IN_ALT = 1U;
const uint8_T Autopilot_IN_OP = 2U;
const uint8_T Autopilot_IN_OP_LD = 3U;
const uint8_T Autopilot_IN_OP_SMALL = 4U;
const ap_output Autopilot_rtZap_output = {
  {
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    0.0,

    {
      0.0,
      0.0
    },

    {
      0.0,
      0.0
    }
  }
} ;

const ap_input Autopilot_rtZap_input = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

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

void AutopilotModelClass::Autopilot_Chart_Init(rtDW_Chart_Autopilot_T *localDW)
{
  localDW->is_active_c5_Autopilot = 0U;
  localDW->is_c5_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD;
}

void AutopilotModelClass::Autopilot_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T *rty_out,
  rtDW_Chart_Autopilot_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  if (localDW->is_active_c5_Autopilot == 0U) {
    localDW->is_active_c5_Autopilot = 1U;
    localDW->is_c5_Autopilot = Autopilot_IN_any;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c5_Autopilot) {
     case Autopilot_IN_any:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp > 10.0)) {
        localDW->is_c5_Autopilot = Autopilot_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 > 10.0)) {
        localDW->is_c5_Autopilot = Autopilot_IN_left;
        *rty_out = rtu_left;
      } else if (tmp_0 < tmp) {
        *rty_out = rtu_left;
      } else {
        *rty_out = rtu_right;
      }
      break;

     case Autopilot_IN_left:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if ((rtu_use_short_path != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c5_Autopilot = Autopilot_IN_any;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_left;
      }
      break;

     default:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if ((rtu_use_short_path != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c5_Autopilot = Autopilot_IN_any;
        if (tmp < tmp_0) {
          *rty_out = rtu_left;
        } else {
          *rty_out = rtu_right;
        }
      } else {
        *rty_out = rtu_right;
      }
      break;
    }
  }
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

void AutopilotModelClass::step()
{
  real_T rtb_Gain_hc;
  real_T rtb_Gain;
  real_T rtb_Mod1;
  real_T rtb_Mod1_g;
  int32_T rtb_ACTIVE_LAW_o;
  int32_T rtb_out_e;
  real_T rtb_kntoms;
  real_T rtb_IC;
  real_T rtb_kntoms_l;
  real_T rtb_IC_b;
  real_T rtb_IC_m;
  real_T rtb_kntoms_f;
  real_T rtb_IC_f;
  int32_T rtb_ACTIVE_LAW;
  real_T rtb_out_c;
  int32_T rtb_mode;
  real_T rtb_Sum;
  if (Autopilot_DWork.is_active_c3_Autopilot == 0U) {
    Autopilot_DWork.is_active_c3_Autopilot = 1U;
    Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
    rtb_out_e = 0;
  } else if (Autopilot_DWork.is_c3_Autopilot == Autopilot_IN_OFF) {
    if (Autopilot_U.in.input.trigger_ap_master == 1.0) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_ON;
      rtb_out_e = 1;
    } else {
      rtb_out_e = 0;
    }
  } else {
    if ((Autopilot_U.in.input.trigger_ap_master == 1.0) || (Autopilot_U.in.input.trigger_ap_off == 1.0)) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
      rtb_out_e = 0;
    } else {
      rtb_out_e = 1;
    }
  }

  if (Autopilot_DWork.is_active_c1_Autopilot == 0U) {
    Autopilot_DWork.is_active_c1_Autopilot = 1U;
    Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
    rtb_ACTIVE_LAW = 1;
  } else {
    switch (Autopilot_DWork.is_c1_Autopilot) {
     case Autopilot_IN_ALT_ACQ:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= 20.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_ACTIVE_LAW = 1;
      } else {
        rtb_ACTIVE_LAW = 2;
      }
      break;

     case Autopilot_IN_ALT_HOLD:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) > 250.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_ACTIVE_LAW = 3;
      } else {
        rtb_ACTIVE_LAW = 1;
      }
      break;

     case Autopilot_IN_SPD_MACH:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_ACTIVE_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_vs_mode == 1.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_VS;
        rtb_ACTIVE_LAW = 4;
      } else {
        rtb_ACTIVE_LAW = 3;
      }
      break;

     default:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_ACTIVE_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_alt_mode != 0.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_ACTIVE_LAW = 3;
      } else {
        rtb_ACTIVE_LAW = 4;
      }
      break;
    }
  }

  rtb_Gain_hc = Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft;
  rtb_Gain = Autopilot_P.Gain_Gain_jt * rtb_Gain_hc;
  rtb_Sum = Autopilot_U.in.data.ap_V_c_kn - Autopilot_U.in.data.V_ias_kn;
  if (Autopilot_DWork.is_active_c4_Autopilot == 0U) {
    Autopilot_DWork.is_active_c4_Autopilot = 1U;
    Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
    rtb_mode = 0;
  } else {
    switch (Autopilot_DWork.is_c4_Autopilot) {
     case Autopilot_IN_ALT:
      rtb_Sum = std::abs(rtb_Gain_hc);
      if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else if ((rtb_Sum > 250.0) && (rtb_Sum <= 1200.0) && (rtb_Sum > std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0))
      {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_SMALL;
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Mod1 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Mod1 = 1.0;
        } else {
          rtb_Mod1 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Mod1 * 1000.0;
      } else {
        rtb_mode = 0;
      }
      break;

     case Autopilot_IN_OP:
      if (rtb_Gain_hc < 0.0) {
        rtb_Mod1 = -1.0;
      } else if (rtb_Gain_hc > 0.0) {
        rtb_Mod1 = 1.0;
      } else {
        rtb_Mod1 = rtb_Gain_hc;
      }

      if (rtb_Sum < 0.0) {
        rtb_Mod1_g = -1.0;
      } else if (rtb_Sum > 0.0) {
        rtb_Mod1_g = 1.0;
      } else {
        rtb_Mod1_g = rtb_Sum;
      }

      if ((rtb_Mod1 == rtb_Mod1_g) && (std::abs(rtb_Sum) >= 20.0)) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_LD;
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Mod1 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Mod1 = 1.0;
        } else {
          rtb_Mod1 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Mod1 * 1000.0;
      } else if (std::abs(rtb_Gain_hc) <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
        rtb_mode = 0;
      } else {
        rtb_mode = 1;
      }
      break;

     case Autopilot_IN_OP_LD:
      if (std::abs(rtb_Sum) < 5.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else {
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Mod1 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Mod1 = 1.0;
        } else {
          rtb_Mod1 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Mod1 * 1000.0;
      }
      break;

     default:
      rtb_Sum = std::abs(rtb_Gain_hc);
      if (rtb_Sum <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
        rtb_mode = 0;
      } else if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else {
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Mod1 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Mod1 = 1.0;
        } else {
          rtb_Mod1 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Mod1 * 1000.0;
      }
      break;
    }
  }

  if (Autopilot_DWork.is_active_c2_Autopilot == 0U) {
    Autopilot_DWork.is_active_c2_Autopilot = 1U;
    Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
    rtb_ACTIVE_LAW_o = 1;
  } else if (Autopilot_DWork.is_c2_Autopilot == Autopilot_IN_MANAGED) {
    if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
      rtb_ACTIVE_LAW_o = 1;
    } else {
      rtb_ACTIVE_LAW_o = 3;
    }
  } else {
    if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_MANAGED;
      rtb_ACTIVE_LAW_o = 3;
    } else {
      rtb_ACTIVE_LAW_o = 1;
    }
  }

  rtb_Mod1 = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value)) + Autopilot_P.Constant3_Value, Autopilot_P.Constant3_Value);
  Autopilot_Chart(rtb_Mod1, Autopilot_P.Gain_Gain_h * rt_modd(Autopilot_P.Constant3_Value - rtb_Mod1,
    Autopilot_P.Constant3_Value), Autopilot_P.Constant_Value, &rtb_Sum, &Autopilot_DWork.sf_Chart_b);
  rtb_Mod1_g = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_e)) + Autopilot_P.Constant3_Value_e, Autopilot_P.Constant3_Value_e);
  Autopilot_Chart(rtb_Mod1_g, Autopilot_P.Gain_Gain_l * rt_modd(Autopilot_P.Constant3_Value_e - rtb_Mod1_g,
    Autopilot_P.Constant3_Value_e), Autopilot_P.Constant_Value_k, &rtb_Mod1, &Autopilot_DWork.sf_Chart_f);
  rtb_out_c = Autopilot_P.Gain_Gain_aa * Autopilot_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_out_c > Autopilot_P.Saturation_UpperSat_k) {
    rtb_out_c = Autopilot_P.Saturation_UpperSat_k;
  } else {
    if (rtb_out_c < Autopilot_P.Saturation_LowerSat_g) {
      rtb_out_c = Autopilot_P.Saturation_LowerSat_g;
    }
  }

  rtb_Mod1_g = rt_modd((rt_modd((rtb_out_c + Autopilot_U.in.data.flight_guidance_tae_deg) * Autopilot_P.Gain1_Gain_p +
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_j), Autopilot_P.Constant_Value_j) -
                        (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_a)) +
                       Autopilot_P.Constant3_Value_a, Autopilot_P.Constant3_Value_a);
  Autopilot_Chart(rtb_Mod1_g, Autopilot_P.Gain_Gain_bp * rt_modd(Autopilot_P.Constant3_Value_a - rtb_Mod1_g,
    Autopilot_P.Constant3_Value_a), Autopilot_P.Constant_Value_g, &rtb_out_c, &Autopilot_DWork.sf_Chart_o);
  switch (rtb_ACTIVE_LAW_o) {
   case 1:
    rtb_Sum *= Autopilot_P.Gain_Gain_by;
    break;

   case 2:
    rtb_Sum = Autopilot_P.Gain_Gain_b * rtb_Mod1;
    break;

   default:
    rtb_Sum = Autopilot_P.Gain_Gain * rtb_out_c;
    break;
  }

  rtb_Mod1 = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data, Autopilot_P.ROLLLIM1_tableData,
    4U);
  if (rtb_Sum > rtb_Mod1) {
    rtb_Sum = rtb_Mod1;
  } else {
    rtb_Mod1 *= Autopilot_P.Gain1_Gain;
    if (rtb_Sum < rtb_Mod1) {
      rtb_Sum = rtb_Mod1;
    }
  }

  rtb_Mod1 = Autopilot_P.Gain_Gain_byk * Autopilot_U.in.data.Phi_deg;
  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad = 1U;
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE = rtb_Mod1;
  }

  rtb_out_c = rtb_Sum - Autopilot_DWork.Delay_DSTATE;
  rtb_kntoms = Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_out_c < rtb_kntoms) {
    rtb_kntoms = rtb_out_c;
  }

  rtb_out_c = Autopilot_P.Gain1_Gain_ih * Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_kntoms > rtb_out_c) {
    rtb_out_c = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE += rtb_out_c;
  rtb_Mod1_g = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1;
  rtb_out_c = rtb_Mod1_g + Autopilot_P.Constant_Value_m;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_out_c * (Autopilot_P.Constant_Value_m - rtb_Mod1_g) *
    Autopilot_DWork.Delay1_DSTATE + (Autopilot_DWork.Delay_DSTATE + Autopilot_DWork.Delay_DSTATE_n) * (rtb_Mod1_g /
    rtb_out_c);
  rtb_out_c = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_k;
  rtb_kntoms = Autopilot_P.RateLimiterVariableTs_up * Autopilot_U.in.time.dt;
  if (rtb_out_c < rtb_kntoms) {
    rtb_kntoms = rtb_out_c;
  }

  rtb_out_c = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo;
  if (rtb_kntoms > rtb_out_c) {
    rtb_out_c = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_k += rtb_out_c;
  if (Autopilot_DWork.Delay_DSTATE_k > Autopilot_P.Saturation_UpperSat_c) {
    rtb_Mod1_g = Autopilot_P.Saturation_UpperSat_c;
  } else if (Autopilot_DWork.Delay_DSTATE_k < Autopilot_P.Saturation_LowerSat_fa) {
    rtb_Mod1_g = Autopilot_P.Saturation_LowerSat_fa;
  } else {
    rtb_Mod1_g = Autopilot_DWork.Delay_DSTATE_k;
  }

  Autopilot_Y.out.output.autopilot.Phi_c_deg = (Autopilot_P.Constant_Value_d - rtb_Mod1_g) * rtb_Mod1 +
    Autopilot_DWork.Delay1_DSTATE * rtb_Mod1_g;
  rtb_kntoms = Autopilot_P.kntoms_Gain * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime) {
    Autopilot_DWork.IC_FirstOutputTime = false;
    rtb_IC = Autopilot_P.IC_Value;
  } else {
    rtb_IC = rtb_kntoms;
  }

  rtb_kntoms_l = Autopilot_P.kntoms_Gain_e * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_h) {
    Autopilot_DWork.IC_FirstOutputTime_h = false;
    rtb_IC_b = Autopilot_P.IC_Value_p;
  } else {
    rtb_IC_b = rtb_kntoms_l;
  }

  rtb_Mod1 = Autopilot_P.DiscreteDerivativeVariableTs_Gain * Autopilot_U.in.data.V_ias_kn;
  rtb_Mod1_g = (rtb_Mod1 - Autopilot_DWork.Delay_DSTATE_nk) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain +
    Autopilot_U.in.data.V_ias_kn;
  rtb_out_c = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_IC_m = rtb_out_c + Autopilot_P.Constant_Value_i;
  Autopilot_DWork.Delay1_DSTATE_b = 1.0 / rtb_IC_m * (Autopilot_P.Constant_Value_i - rtb_out_c) *
    Autopilot_DWork.Delay1_DSTATE_b + (rtb_Mod1_g + Autopilot_DWork.Delay_DSTATE_h) * (rtb_out_c / rtb_IC_m);
  rtb_out_c = Autopilot_P.kntoms_Gain_a * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_p) {
    Autopilot_DWork.IC_FirstOutputTime_p = false;
    rtb_IC_m = Autopilot_P.IC_Value_n;
  } else {
    rtb_IC_m = rtb_out_c;
  }

  rtb_kntoms_f = Autopilot_P.kntoms_Gain_p * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_k) {
    Autopilot_DWork.IC_FirstOutputTime_k = false;
    rtb_IC_f = Autopilot_P.IC_Value_pe;
  } else {
    rtb_IC_f = rtb_kntoms_f;
  }

  switch (rtb_ACTIVE_LAW) {
   case 1:
    rtb_out_c = rtb_Gain_hc * Autopilot_P.Gain_Gain_ml;
    if (rtb_out_c > Autopilot_P.Saturation_UpperSat_i) {
      rtb_out_c = Autopilot_P.Saturation_UpperSat_i;
    } else {
      if (rtb_out_c < Autopilot_P.Saturation_LowerSat_f) {
        rtb_out_c = Autopilot_P.Saturation_LowerSat_f;
      }
    }

    rtb_out_c = (rtb_out_c - std::sin((Autopilot_P.Gain2_Gain_kp * Autopilot_U.in.data.Theta_deg - std::cos
      (Autopilot_P.Gain1_Gain_a * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_i) * rtb_kntoms * Autopilot_P.msftmin_Gain_e) * Autopilot_P.ftmintoms_Gain_j / rtb_IC;
    if (rtb_out_c > 1.0) {
      rtb_out_c = 1.0;
    } else {
      if (rtb_out_c < -1.0) {
        rtb_out_c = -1.0;
      }
    }

    rtb_IC_m = Autopilot_P.Gain_Gain_n * std::asin(rtb_out_c);
    break;

   case 2:
    rtb_out_c = (rtb_Gain_hc * Autopilot_P.Gain_Gain_m - std::sin((Autopilot_P.Gain2_Gain_k1 *
      Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_kd * Autopilot_U.in.data.Phi_deg) *
      Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_o) * rtb_kntoms_l * Autopilot_P.msftmin_Gain_d) *
      Autopilot_P.ftmintoms_Gain_m / rtb_IC_b;
    if (rtb_out_c > 1.0) {
      rtb_out_c = 1.0;
    } else {
      if (rtb_out_c < -1.0) {
        rtb_out_c = -1.0;
      }
    }

    rtb_IC_m = Autopilot_P.Gain_Gain_j * std::asin(rtb_out_c);
    break;

   case 3:
    if (rtb_mode > Autopilot_P.Switch_Threshold) {
      rtb_out_c = Autopilot_DWork.Delay1_DSTATE_b - Autopilot_U.in.data.ap_V_c_kn;
      if (rtb_out_c > Autopilot_P.Saturation_UpperSat) {
        rtb_out_c = Autopilot_P.Saturation_UpperSat;
      } else {
        if (rtb_out_c < Autopilot_P.Saturation_LowerSat) {
          rtb_out_c = Autopilot_P.Saturation_LowerSat;
        }
      }

      rtb_IC_m = Autopilot_P.Gain1_Gain_g * rtb_out_c;
    } else {
      rtb_out_c = (rtb_Gain - std::sin((Autopilot_P.Gain2_Gain_k * Autopilot_U.in.data.Theta_deg - std::cos
        (Autopilot_P.Gain1_Gain_c * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
        Autopilot_P.Gain1_Gain_h) * rtb_out_c * Autopilot_P.msftmin_Gain_p) * Autopilot_P.ftmintoms_Gain_c / rtb_IC_m;
      if (rtb_out_c > 1.0) {
        rtb_out_c = 1.0;
      } else {
        if (rtb_out_c < -1.0) {
          rtb_out_c = -1.0;
        }
      }

      rtb_IC_m = Autopilot_P.Gain_Gain_o * std::asin(rtb_out_c);
    }
    break;

   default:
    rtb_out_c = (Autopilot_U.in.data.ap_H_dot_c_ft_min - std::sin((Autopilot_P.Gain2_Gain *
      Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_b * Autopilot_U.in.data.Phi_deg) *
      Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_k) * rtb_kntoms_f * Autopilot_P.msftmin_Gain) *
      Autopilot_P.ftmintoms_Gain / rtb_IC_f;
    if (rtb_out_c > 1.0) {
      rtb_out_c = 1.0;
    } else {
      if (rtb_out_c < -1.0) {
        rtb_out_c = -1.0;
      }
    }

    rtb_IC_m = Autopilot_P.Gain_Gain_a * std::asin(rtb_out_c);
    break;
  }

  rtb_Gain_hc = Autopilot_P.Gain_Gain_nw * Autopilot_U.in.data.Theta_deg;
  rtb_IC_m += rtb_Gain_hc;
  if (rtb_IC_m > Autopilot_P.Constant1_Value) {
    rtb_IC_m = Autopilot_P.Constant1_Value;
  } else {
    rtb_Gain = Autopilot_P.Gain1_Gain_cl * Autopilot_P.Constant1_Value;
    if (rtb_IC_m < rtb_Gain) {
      rtb_IC_m = rtb_Gain;
    }
  }

  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad_i = 1U;
  }

  if (Autopilot_DWork.icLoad_i != 0) {
    Autopilot_DWork.Delay_DSTATE_n4 = rtb_Gain_hc;
  }

  rtb_out_c = rtb_IC_m - Autopilot_DWork.Delay_DSTATE_n4;
  rtb_kntoms = Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_out_c < rtb_kntoms) {
    rtb_kntoms = rtb_out_c;
  }

  rtb_out_c = Autopilot_P.Gain1_Gain_d * Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_kntoms > rtb_out_c) {
    rtb_out_c = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_n4 += rtb_out_c;
  rtb_Gain = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_p;
  rtb_out_c = rtb_Gain + Autopilot_P.Constant_Value_ig;
  Autopilot_DWork.Delay1_DSTATE_k = 1.0 / rtb_out_c * (Autopilot_P.Constant_Value_ig - rtb_Gain) *
    Autopilot_DWork.Delay1_DSTATE_k + (Autopilot_DWork.Delay_DSTATE_n4 + Autopilot_DWork.Delay_DSTATE_l) * (rtb_Gain /
    rtb_out_c);
  rtb_out_c = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_f;
  rtb_kntoms = Autopilot_P.RateLimiterVariableTs_up_k * Autopilot_U.in.time.dt;
  if (rtb_out_c < rtb_kntoms) {
    rtb_kntoms = rtb_out_c;
  }

  rtb_out_c = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo_h;
  if (rtb_kntoms > rtb_out_c) {
    rtb_out_c = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_f += rtb_out_c;
  if (Autopilot_DWork.Delay_DSTATE_f > Autopilot_P.Saturation_UpperSat_a) {
    rtb_Gain = Autopilot_P.Saturation_UpperSat_a;
  } else if (Autopilot_DWork.Delay_DSTATE_f < Autopilot_P.Saturation_LowerSat_j) {
    rtb_Gain = Autopilot_P.Saturation_LowerSat_j;
  } else {
    rtb_Gain = Autopilot_DWork.Delay_DSTATE_f;
  }

  Autopilot_Y.out.output.autopilot.Theta_c_deg = (Autopilot_P.Constant_Value_p - rtb_Gain) * rtb_Gain_hc +
    Autopilot_DWork.Delay1_DSTATE_k * rtb_Gain;
  Autopilot_Y.out.time = Autopilot_U.in.time;
  Autopilot_Y.out.input = Autopilot_U.in.input;
  Autopilot_Y.out.data = Autopilot_U.in.data;
  Autopilot_Y.out.output.ap_on = rtb_out_e;
  Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_IC_m;
  Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_Sum;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_n = Autopilot_DWork.Delay_DSTATE;
  Autopilot_DWork.Delay_DSTATE_nk = rtb_Mod1;
  Autopilot_DWork.Delay_DSTATE_h = rtb_Mod1_g;
  Autopilot_DWork.icLoad_i = 0U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_DWork.Delay_DSTATE_n4;
}

void AutopilotModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&Autopilot_DWork), 0,
                     sizeof(D_Work_Autopilot_T));
  Autopilot_U.in = Autopilot_rtZap_input;
  Autopilot_Y.out = Autopilot_rtZap_output;
  Autopilot_DWork.IC_FirstOutputTime = true;
  Autopilot_DWork.IC_FirstOutputTime_h = true;
  Autopilot_DWork.IC_FirstOutputTime_p = true;
  Autopilot_DWork.IC_FirstOutputTime_k = true;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_n = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.RateLimiterVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_nk = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.Delay_InitialCondition_c;
  Autopilot_DWork.Delay1_DSTATE_b = Autopilot_P.Delay1_InitialCondition_e;
  Autopilot_DWork.icLoad_i = 1U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_P.Delay_InitialCondition_o;
  Autopilot_DWork.Delay1_DSTATE_k = Autopilot_P.Delay1_InitialCondition_n;
  Autopilot_DWork.Delay_DSTATE_f = Autopilot_P.RateLimiterVariableTs_InitialCondition_i;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c1_Autopilot = 0U;
  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c4_Autopilot = 0U;
  Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c2_Autopilot = 0U;
  Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_b);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_f);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_o);
}

void AutopilotModelClass::terminate()
{
}

AutopilotModelClass::AutopilotModelClass()
{
}

AutopilotModelClass::~AutopilotModelClass()
{
}
