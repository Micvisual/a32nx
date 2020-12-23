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
  real_T rtb_autopilot_m;
  real_T rtb_flight_director_g;
  real_T rtb_flight_director;
  real_T rtb_Gain;
  real_T rtb_out_e;
  real_T rtb_ROLLLIM1;
  boolean_T rtb_H_PATH;
  real_T rtb_out_h;
  int32_T rtb_ACTIVE_LAW_l;
  real_T rtb_IC;
  real_T rtb_IC_j;
  real_T rtb_Gain1_m;
  real_T rtb_IC_c;
  int32_T rtb_ACTIVE_LAW;
  boolean_T rtb_ALT_HOLD;
  boolean_T rtb_ALT_ACQ;
  boolean_T rtb_SPD_MACH;
  boolean_T rtb_VS;
  int32_T rtb_mode;
  real_T rtb_Sum;
  real_T rtb_Sum3_g;
  real_T rtb_Gain1_kh;
  if (Autopilot_DWork.is_active_c3_Autopilot == 0U) {
    Autopilot_DWork.is_active_c3_Autopilot = 1U;
    Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
    Autopilot_Y.out.output.ap_on = 0.0;
  } else if (Autopilot_DWork.is_c3_Autopilot == Autopilot_IN_OFF) {
    if (Autopilot_U.in.input.trigger_ap_master == 1.0) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_ON;
      Autopilot_Y.out.output.ap_on = 1.0;
    } else {
      Autopilot_Y.out.output.ap_on = 0.0;
    }
  } else {
    if ((Autopilot_U.in.input.trigger_ap_master == 1.0) || (Autopilot_U.in.input.trigger_ap_off == 1.0)) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
      Autopilot_Y.out.output.ap_on = 0.0;
    } else {
      Autopilot_Y.out.output.ap_on = 1.0;
    }
  }

  if (Autopilot_DWork.is_active_c1_Autopilot == 0U) {
    Autopilot_DWork.is_active_c1_Autopilot = 1U;
    Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
    rtb_ACTIVE_LAW = 1;
    rtb_ALT_HOLD = true;
    rtb_ALT_ACQ = true;
    rtb_SPD_MACH = true;
    rtb_VS = false;
  } else {
    switch (Autopilot_DWork.is_c1_Autopilot) {
     case Autopilot_IN_ALT_ACQ:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= 20.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_ACTIVE_LAW = 1;
        rtb_ALT_HOLD = true;
        rtb_ALT_ACQ = true;
        rtb_SPD_MACH = true;
        rtb_VS = false;
      } else {
        rtb_ACTIVE_LAW = 2;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = true;
        rtb_SPD_MACH = false;
        rtb_VS = false;
      }
      break;

     case Autopilot_IN_ALT_HOLD:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) > 250.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_ACTIVE_LAW = 3;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = false;
        rtb_SPD_MACH = true;
        rtb_VS = false;
      } else {
        rtb_ACTIVE_LAW = 1;
        rtb_ALT_HOLD = true;
        rtb_ALT_ACQ = true;
        rtb_SPD_MACH = true;
        rtb_VS = false;
      }
      break;

     case Autopilot_IN_SPD_MACH:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_ACTIVE_LAW = 2;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = true;
        rtb_SPD_MACH = false;
        rtb_VS = false;
      } else if (Autopilot_U.in.input.trigger_vs_mode == 1.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_VS;
        rtb_ACTIVE_LAW = 4;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = false;
        rtb_SPD_MACH = false;
        rtb_VS = true;
      } else {
        rtb_ACTIVE_LAW = 3;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = false;
        rtb_SPD_MACH = true;
        rtb_VS = false;
      }
      break;

     default:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_ACTIVE_LAW = 2;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = true;
        rtb_SPD_MACH = false;
        rtb_VS = false;
      } else if (Autopilot_U.in.input.trigger_alt_mode != 0.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_ACTIVE_LAW = 3;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = false;
        rtb_SPD_MACH = true;
        rtb_VS = false;
      } else {
        rtb_ACTIVE_LAW = 4;
        rtb_ALT_HOLD = false;
        rtb_ALT_ACQ = false;
        rtb_SPD_MACH = false;
        rtb_VS = true;
      }
      break;
    }
  }

  rtb_Gain1_kh = Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft;
  rtb_Gain = Autopilot_P.Gain_Gain_jt * rtb_Gain1_kh;
  rtb_Sum = Autopilot_U.in.data.ap_V_c_kn - Autopilot_U.in.data.V_ias_kn;
  if (Autopilot_DWork.is_active_c4_Autopilot == 0U) {
    Autopilot_DWork.is_active_c4_Autopilot = 1U;
    Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
    rtb_mode = 0;
  } else {
    switch (Autopilot_DWork.is_c4_Autopilot) {
     case Autopilot_IN_ALT:
      rtb_Sum = std::abs(rtb_Gain1_kh);
      if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else if ((rtb_Sum > 250.0) && (rtb_Sum <= 1200.0) && (rtb_Sum > std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0))
      {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_SMALL;
        rtb_mode = 0;
        if (rtb_Gain1_kh < 0.0) {
          rtb_ROLLLIM1 = -1.0;
        } else if (rtb_Gain1_kh > 0.0) {
          rtb_ROLLLIM1 = 1.0;
        } else {
          rtb_ROLLLIM1 = rtb_Gain1_kh;
        }

        rtb_Gain = rtb_ROLLLIM1 * 1000.0;
      } else {
        rtb_mode = 0;
      }
      break;

     case Autopilot_IN_OP:
      if (rtb_Gain1_kh < 0.0) {
        rtb_ROLLLIM1 = -1.0;
      } else if (rtb_Gain1_kh > 0.0) {
        rtb_ROLLLIM1 = 1.0;
      } else {
        rtb_ROLLLIM1 = rtb_Gain1_kh;
      }

      if (rtb_Sum < 0.0) {
        rtb_out_e = -1.0;
      } else if (rtb_Sum > 0.0) {
        rtb_out_e = 1.0;
      } else {
        rtb_out_e = rtb_Sum;
      }

      if ((rtb_ROLLLIM1 == rtb_out_e) && (std::abs(rtb_Sum) >= 20.0)) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_LD;
        rtb_mode = 0;
        if (rtb_Gain1_kh < 0.0) {
          rtb_ROLLLIM1 = -1.0;
        } else if (rtb_Gain1_kh > 0.0) {
          rtb_ROLLLIM1 = 1.0;
        } else {
          rtb_ROLLLIM1 = rtb_Gain1_kh;
        }

        rtb_Gain = rtb_ROLLLIM1 * 1000.0;
      } else if (std::abs(rtb_Gain1_kh) <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
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
        if (rtb_Gain1_kh < 0.0) {
          rtb_ROLLLIM1 = -1.0;
        } else if (rtb_Gain1_kh > 0.0) {
          rtb_ROLLLIM1 = 1.0;
        } else {
          rtb_ROLLLIM1 = rtb_Gain1_kh;
        }

        rtb_Gain = rtb_ROLLLIM1 * 1000.0;
      }
      break;

     default:
      rtb_Sum = std::abs(rtb_Gain1_kh);
      if (rtb_Sum <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
        rtb_mode = 0;
      } else if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else {
        rtb_mode = 0;
        if (rtb_Gain1_kh < 0.0) {
          rtb_ROLLLIM1 = -1.0;
        } else if (rtb_Gain1_kh > 0.0) {
          rtb_ROLLLIM1 = 1.0;
        } else {
          rtb_ROLLLIM1 = rtb_Gain1_kh;
        }

        rtb_Gain = rtb_ROLLLIM1 * 1000.0;
      }
      break;
    }
  }

  if (Autopilot_DWork.is_active_c2_Autopilot == 0U) {
    Autopilot_DWork.is_active_c2_Autopilot = 1U;
    Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
    rtb_ACTIVE_LAW_l = 1;
    rtb_H_PATH = false;
  } else if (Autopilot_DWork.is_c2_Autopilot == Autopilot_IN_MANAGED) {
    if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
      rtb_ACTIVE_LAW_l = 1;
      rtb_H_PATH = false;
    } else {
      rtb_ACTIVE_LAW_l = 3;
      rtb_H_PATH = true;
      Autopilot_DWork.icLoad = 1U;
    }
  } else {
    if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_MANAGED;
      rtb_ACTIVE_LAW_l = 3;
      rtb_H_PATH = true;
      Autopilot_DWork.icLoad = 1U;
    } else {
      rtb_ACTIVE_LAW_l = 1;
      rtb_H_PATH = false;
    }
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE = Autopilot_P.Gain2_Gain_j * Autopilot_U.in.data.Phi_deg;
  }

  rtb_ROLLLIM1 = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data,
    Autopilot_P.ROLLLIM1_tableData, 4U);
  rtb_Sum = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value)) + Autopilot_P.Constant3_Value, Autopilot_P.Constant3_Value);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_i * rt_modd(Autopilot_P.Constant3_Value - rtb_Sum,
    Autopilot_P.Constant3_Value), Autopilot_P.Constant4_Value, &rtb_out_e, &Autopilot_DWork.sf_Chart_h);
  rtb_Sum = Autopilot_P.Gain_Gain_e * rtb_out_e;
  if (rtb_Sum > rtb_ROLLLIM1) {
    rtb_Sum = rtb_ROLLLIM1;
  } else {
    rtb_ROLLLIM1 *= Autopilot_P.Gain1_Gain_a;
    if (rtb_Sum < rtb_ROLLLIM1) {
      rtb_Sum = rtb_ROLLLIM1;
    }
  }

  rtb_flight_director_g = rtb_Sum - Autopilot_DWork.Delay_DSTATE;
  rtb_autopilot_m = Autopilot_P.Constant1_Value_j * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_p * Autopilot_P.Constant1_Value_j * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_c;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_c - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE + (Autopilot_DWork.Delay_DSTATE + Autopilot_DWork.Delay_DSTATE_l) * (rtb_autopilot_m /
    rtb_flight_director_g);
  Autopilot_DWork.icLoad_a = 1U;
  if (Autopilot_DWork.icLoad_a != 0) {
    Autopilot_DWork.Delay_DSTATE_a = Autopilot_P.Gain2_Gain_c * Autopilot_U.in.data.Phi_deg;
  }

  rtb_out_e = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data_a,
    Autopilot_P.ROLLLIM1_tableData_j, 4U);
  rtb_ROLLLIM1 = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_m)) + Autopilot_P.Constant3_Value_m, Autopilot_P.Constant3_Value_m);
  Autopilot_Chart(rtb_ROLLLIM1, Autopilot_P.Gain_Gain_d3 * rt_modd(Autopilot_P.Constant3_Value_m - rtb_ROLLLIM1,
    Autopilot_P.Constant3_Value_m), Autopilot_P.Constant4_Value_k, &rtb_autopilot_m, &Autopilot_DWork.sf_Chart_m);
  rtb_ROLLLIM1 = Autopilot_P.Gain_Gain_en * rtb_autopilot_m;
  if (rtb_ROLLLIM1 > rtb_out_e) {
    rtb_ROLLLIM1 = rtb_out_e;
  } else {
    rtb_out_e *= Autopilot_P.Gain1_Gain_f;
    if (rtb_ROLLLIM1 < rtb_out_e) {
      rtb_ROLLLIM1 = rtb_out_e;
    }
  }

  rtb_flight_director_g = rtb_ROLLLIM1 - Autopilot_DWork.Delay_DSTATE_a;
  rtb_autopilot_m = Autopilot_P.Constant1_Value_h * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_nw * Autopilot_P.Constant1_Value_h * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_a += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_o;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_j;
  Autopilot_DWork.Delay1_DSTATE_b = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_j - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_b + (Autopilot_DWork.Delay_DSTATE_a + Autopilot_DWork.Delay_DSTATE_g) *
    (rtb_autopilot_m / rtb_flight_director_g);
  if (!rtb_H_PATH) {
    Autopilot_DWork.icLoad_k = 1U;
  }

  if (Autopilot_DWork.icLoad_k != 0) {
    Autopilot_DWork.Delay_DSTATE_p = Autopilot_P.Gain2_Gain_cw * Autopilot_U.in.data.Phi_deg;
  }

  rtb_autopilot_m = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data_j,
    Autopilot_P.ROLLLIM1_tableData_c, 4U);
  rtb_flight_director_g = Autopilot_P.Gain_Gain_a * Autopilot_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_flight_director_g > Autopilot_P.Saturation_UpperSat_k) {
    rtb_flight_director_g = Autopilot_P.Saturation_UpperSat_k;
  } else {
    if (rtb_flight_director_g < Autopilot_P.Saturation_LowerSat_g) {
      rtb_flight_director_g = Autopilot_P.Saturation_LowerSat_g;
    }
  }

  rtb_out_e = rt_modd((rt_modd((rtb_flight_director_g + Autopilot_U.in.data.flight_guidance_tae_deg) *
    Autopilot_P.Gain1_Gain_pa + (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_a),
    Autopilot_P.Constant_Value_a) - (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_j)) +
                      Autopilot_P.Constant3_Value_j, Autopilot_P.Constant3_Value_j);
  Autopilot_Chart(rtb_out_e, Autopilot_P.Gain_Gain_i0 * rt_modd(Autopilot_P.Constant3_Value_j - rtb_out_e,
    Autopilot_P.Constant3_Value_j), Autopilot_P.Constant4_Value_d, &rtb_out_h, &Autopilot_DWork.sf_Chart_i);
  rtb_out_e = Autopilot_P.Gain_Gain_l0 * rtb_out_h;
  if (rtb_out_e > rtb_autopilot_m) {
    rtb_out_e = rtb_autopilot_m;
  } else {
    rtb_autopilot_m *= Autopilot_P.Gain1_Gain;
    if (rtb_out_e < rtb_autopilot_m) {
      rtb_out_e = rtb_autopilot_m;
    }
  }

  rtb_flight_director_g = rtb_out_e - Autopilot_DWork.Delay_DSTATE_p;
  rtb_autopilot_m = Autopilot_P.Constant1_Value_m * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_ee * Autopilot_P.Constant1_Value_m * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_p += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_e;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_b;
  Autopilot_DWork.Delay1_DSTATE_g = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_b - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_g + (Autopilot_DWork.Delay_DSTATE_p + Autopilot_DWork.Delay_DSTATE_d) *
    (rtb_autopilot_m / rtb_flight_director_g);
  switch (rtb_ACTIVE_LAW_l) {
   case 1:
    Autopilot_Y.out.output.autopilot.Phi_c_deg = Autopilot_DWork.Delay1_DSTATE;
    break;

   case 2:
    Autopilot_Y.out.output.autopilot.Phi_c_deg = Autopilot_DWork.Delay1_DSTATE_b;
    break;

   default:
    Autopilot_Y.out.output.autopilot.Phi_c_deg = Autopilot_DWork.Delay1_DSTATE_g;
    break;
  }

  rtb_autopilot_m = Autopilot_P.Gain2_Gain_k * Autopilot_U.in.data.Theta_deg;
  rtb_out_h = Autopilot_P.kntoms_Gain * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime) {
    Autopilot_DWork.IC_FirstOutputTime = false;
    rtb_IC = Autopilot_P.IC_Value;
  } else {
    rtb_IC = rtb_out_h;
  }

  if (rtb_ALT_HOLD) {
    rtb_flight_director_g = rtb_Gain1_kh * Autopilot_P.Gain_Gain_m;
    if (rtb_flight_director_g > Autopilot_P.Saturation_UpperSat) {
      rtb_flight_director_g = Autopilot_P.Saturation_UpperSat;
    } else {
      if (rtb_flight_director_g < Autopilot_P.Saturation_LowerSat) {
        rtb_flight_director_g = Autopilot_P.Saturation_LowerSat;
      }
    }

    rtb_flight_director_g = (rtb_flight_director_g - std::sin((rtb_autopilot_m - std::cos(Autopilot_P.Gain1_Gain_f3 *
      Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_fr) * rtb_out_h *
      Autopilot_P.msftmin_Gain_f) * Autopilot_P.ftmintoms_Gain_b / rtb_IC;
    if (rtb_flight_director_g > 1.0) {
      rtb_flight_director_g = 1.0;
    } else {
      if (rtb_flight_director_g < -1.0) {
        rtb_flight_director_g = -1.0;
      }
    }

    rtb_Sum3_g = Autopilot_P.Gain_Gain_d * std::asin(rtb_flight_director_g) + rtb_autopilot_m;
    if (rtb_Sum3_g > Autopilot_P.Constant1_Value_g) {
      rtb_Sum3_g = Autopilot_P.Constant1_Value_g;
    } else {
      rtb_out_h = Autopilot_P.Gain1_Gain_d * Autopilot_P.Constant1_Value_g;
      if (rtb_Sum3_g < rtb_out_h) {
        rtb_Sum3_g = rtb_out_h;
      }
    }
  } else {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  rtb_out_h = Autopilot_P.Gain2_Gain_k1 * Autopilot_U.in.data.Theta_deg;
  rtb_IC = Autopilot_P.kntoms_Gain_e * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_p) {
    Autopilot_DWork.IC_FirstOutputTime_p = false;
    rtb_IC_j = Autopilot_P.IC_Value_b;
  } else {
    rtb_IC_j = rtb_IC;
  }

  if (rtb_ALT_ACQ) {
    rtb_flight_director_g = (rtb_Gain1_kh * Autopilot_P.Gain_Gain - std::sin((rtb_out_h - std::cos
      (Autopilot_P.Gain1_Gain_e * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_h) * rtb_IC * Autopilot_P.msftmin_Gain) * Autopilot_P.ftmintoms_Gain / rtb_IC_j;
    if (rtb_flight_director_g > 1.0) {
      rtb_flight_director_g = 1.0;
    } else {
      if (rtb_flight_director_g < -1.0) {
        rtb_flight_director_g = -1.0;
      }
    }

    rtb_IC_j = Autopilot_P.Gain_Gain_j * std::asin(rtb_flight_director_g) + rtb_out_h;
    if (rtb_IC_j > Autopilot_P.Constant1_Value) {
      rtb_IC_j = Autopilot_P.Constant1_Value;
    } else {
      rtb_Gain1_kh = Autopilot_P.Gain1_Gain_n * Autopilot_P.Constant1_Value;
      if (rtb_IC_j < rtb_Gain1_kh) {
        rtb_IC_j = rtb_Gain1_kh;
      }
    }
  } else {
    rtb_IC_j = rtb_out_h;
  }

  rtb_Gain1_m = Autopilot_P.Gain1_Gain_gk * Autopilot_U.in.data.Theta_deg;
  rtb_Gain1_kh = Autopilot_P.DiscreteDerivativeVariableTs_Gain * Autopilot_U.in.data.V_ias_kn;
  rtb_IC = (rtb_Gain1_kh - Autopilot_DWork.Delay_DSTATE_i) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain +
    Autopilot_U.in.data.V_ias_kn;
  rtb_flight_director_g = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_flight_director = rtb_flight_director_g + Autopilot_P.Constant_Value_l;
  Autopilot_DWork.Delay1_DSTATE_e = 1.0 / rtb_flight_director * (Autopilot_P.Constant_Value_l - rtb_flight_director_g) *
    Autopilot_DWork.Delay1_DSTATE_e + (rtb_IC + Autopilot_DWork.Delay_DSTATE_n) * (rtb_flight_director_g /
    rtb_flight_director);
  rtb_flight_director_g = Autopilot_P.kntoms_Gain_a * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_c) {
    Autopilot_DWork.IC_FirstOutputTime_c = false;
    rtb_flight_director = Autopilot_P.IC_Value_e;
  } else {
    rtb_flight_director = rtb_flight_director_g;
  }

  if (rtb_SPD_MACH) {
    if (rtb_mode > Autopilot_P.Switch_Threshold) {
      rtb_flight_director_g = Autopilot_DWork.Delay1_DSTATE_e - Autopilot_U.in.data.ap_V_c_kn;
      if (rtb_flight_director_g > Autopilot_P.Saturation_UpperSat_a) {
        rtb_flight_director_g = Autopilot_P.Saturation_UpperSat_a;
      } else {
        if (rtb_flight_director_g < Autopilot_P.Saturation_LowerSat_d) {
          rtb_flight_director_g = Autopilot_P.Saturation_LowerSat_d;
        }
      }

      rtb_Gain = Autopilot_P.Gain1_Gain_g * rtb_flight_director_g;
    } else {
      rtb_flight_director_g = (rtb_Gain - std::sin((Autopilot_P.Gain2_Gain * Autopilot_U.in.data.Theta_deg - std::cos
        (Autopilot_P.Gain1_Gain_ea * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
        Autopilot_P.Gain1_Gain_b) * rtb_flight_director_g * Autopilot_P.msftmin_Gain_p) * Autopilot_P.ftmintoms_Gain_c /
        rtb_flight_director;
      if (rtb_flight_director_g > 1.0) {
        rtb_flight_director_g = 1.0;
      } else {
        if (rtb_flight_director_g < -1.0) {
          rtb_flight_director_g = -1.0;
        }
      }

      rtb_Gain = Autopilot_P.Gain_Gain_l * std::asin(rtb_flight_director_g);
    }

    rtb_Gain += rtb_Gain1_m;
    if (rtb_Gain > Autopilot_P.Constant_Value) {
      rtb_Gain = Autopilot_P.Constant_Value;
    } else {
      rtb_flight_director_g = Autopilot_P.Gain1_Gain_nh * Autopilot_P.Constant_Value;
      if (rtb_Gain < rtb_flight_director_g) {
        rtb_Gain = rtb_flight_director_g;
      }
    }
  } else {
    rtb_Gain = rtb_Gain1_m;
  }

  rtb_flight_director = Autopilot_P.Gain2_Gain_k2 * Autopilot_U.in.data.Theta_deg;
  rtb_flight_director_g = Autopilot_P.kntoms_Gain_p * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_cl) {
    Autopilot_DWork.IC_FirstOutputTime_cl = false;
    rtb_IC_c = Autopilot_P.IC_Value_h;
  } else {
    rtb_IC_c = rtb_flight_director_g;
  }

  if (rtb_VS) {
    rtb_flight_director_g = (Autopilot_U.in.data.ap_H_dot_c_ft_min - std::sin((rtb_flight_director - std::cos
      (Autopilot_P.Gain1_Gain_o * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_i) * rtb_flight_director_g * Autopilot_P.msftmin_Gain_o) * Autopilot_P.ftmintoms_Gain_f /
      rtb_IC_c;
    if (rtb_flight_director_g > 1.0) {
      rtb_flight_director_g = 1.0;
    } else {
      if (rtb_flight_director_g < -1.0) {
        rtb_flight_director_g = -1.0;
      }
    }

    rtb_IC_c = Autopilot_P.Gain_Gain_ds * std::asin(rtb_flight_director_g) + rtb_flight_director;
    if (rtb_IC_c > Autopilot_P.Constant1_Value_e) {
      rtb_IC_c = Autopilot_P.Constant1_Value_e;
    } else {
      rtb_flight_director_g = Autopilot_P.Gain1_Gain_j * Autopilot_P.Constant1_Value_e;
      if (rtb_IC_c < rtb_flight_director_g) {
        rtb_IC_c = rtb_flight_director_g;
      }
    }
  } else {
    rtb_IC_c = rtb_flight_director;
  }

  switch (rtb_ACTIVE_LAW) {
   case 1:
    Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Sum3_g;
    break;

   case 2:
    Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_IC_j;
    break;

   case 3:
    Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Gain;
    break;

   default:
    Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_IC_c;
    break;
  }

  if (!rtb_ALT_HOLD) {
    Autopilot_DWork.icLoad_l = 1U;
  }

  if (Autopilot_DWork.icLoad_l != 0) {
    Autopilot_DWork.Delay_DSTATE_e = rtb_autopilot_m;
  }

  rtb_flight_director_g = rtb_Sum3_g - Autopilot_DWork.Delay_DSTATE_e;
  rtb_autopilot_m = Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_ab * Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_e += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_c;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_f;
  Autopilot_DWork.Delay1_DSTATE_n = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_f - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_n + (Autopilot_DWork.Delay_DSTATE_e + Autopilot_DWork.Delay_DSTATE_pw) *
    (rtb_autopilot_m / rtb_flight_director_g);
  if (!rtb_ALT_ACQ) {
    Autopilot_DWork.icLoad_aw = 1U;
  }

  if (Autopilot_DWork.icLoad_aw != 0) {
    Autopilot_DWork.Delay_DSTATE_py = rtb_out_h;
  }

  rtb_flight_director_g = rtb_IC_j - Autopilot_DWork.Delay_DSTATE_py;
  rtb_autopilot_m = Autopilot_P.Constant2_Value_k * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_e0 * Autopilot_P.Constant2_Value_k * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_py += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_k;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_p;
  Autopilot_DWork.Delay1_DSTATE_e1 = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_p - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_e1 + (Autopilot_DWork.Delay_DSTATE_py + Autopilot_DWork.Delay_DSTATE_b) *
    (rtb_autopilot_m / rtb_flight_director_g);
  if (!rtb_SPD_MACH) {
    Autopilot_DWork.icLoad_m = 1U;
  }

  if (Autopilot_DWork.icLoad_m != 0) {
    Autopilot_DWork.Delay_DSTATE_m = rtb_Gain1_m;
  }

  rtb_flight_director_g = rtb_Gain - Autopilot_DWork.Delay_DSTATE_m;
  rtb_autopilot_m = Autopilot_P.Constant2_Value_p * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_m * Autopilot_P.Constant2_Value_p * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_m += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_ku;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_d;
  Autopilot_DWork.Delay1_DSTATE_o = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_d - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_o + (Autopilot_DWork.Delay_DSTATE_m + Autopilot_DWork.Delay_DSTATE_k) *
    (rtb_autopilot_m / rtb_flight_director_g);
  if (!rtb_VS) {
    Autopilot_DWork.icLoad_kc = 1U;
  }

  if (Autopilot_DWork.icLoad_kc != 0) {
    Autopilot_DWork.Delay_DSTATE_ip = rtb_flight_director;
  }

  rtb_flight_director_g = rtb_IC_c - Autopilot_DWork.Delay_DSTATE_ip;
  rtb_autopilot_m = Autopilot_P.Constant2_Value_f * Autopilot_U.in.time.dt;
  if (rtb_flight_director_g < rtb_autopilot_m) {
    rtb_autopilot_m = rtb_flight_director_g;
  }

  rtb_Sum3_g = Autopilot_P.Gain1_Gain_a2 * Autopilot_P.Constant2_Value_f * Autopilot_U.in.time.dt;
  if (rtb_autopilot_m > rtb_Sum3_g) {
    rtb_Sum3_g = rtb_autopilot_m;
  }

  Autopilot_DWork.Delay_DSTATE_ip += rtb_Sum3_g;
  rtb_autopilot_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_l;
  rtb_flight_director_g = rtb_autopilot_m + Autopilot_P.Constant_Value_o;
  Autopilot_DWork.Delay1_DSTATE_l = 1.0 / rtb_flight_director_g * (Autopilot_P.Constant_Value_o - rtb_autopilot_m) *
    Autopilot_DWork.Delay1_DSTATE_l + (Autopilot_DWork.Delay_DSTATE_ip + Autopilot_DWork.Delay_DSTATE_o) *
    (rtb_autopilot_m / rtb_flight_director_g);
  switch (rtb_ACTIVE_LAW) {
   case 1:
    Autopilot_Y.out.output.autopilot.Theta_c_deg = Autopilot_DWork.Delay1_DSTATE_n;
    break;

   case 2:
    Autopilot_Y.out.output.autopilot.Theta_c_deg = Autopilot_DWork.Delay1_DSTATE_e1;
    break;

   case 3:
    Autopilot_Y.out.output.autopilot.Theta_c_deg = Autopilot_DWork.Delay1_DSTATE_o;
    break;

   default:
    Autopilot_Y.out.output.autopilot.Theta_c_deg = Autopilot_DWork.Delay1_DSTATE_l;
    break;
  }

  switch (rtb_ACTIVE_LAW_l) {
   case 1:
    Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_Sum;
    break;

   case 2:
    Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_ROLLLIM1;
    break;

   default:
    Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_out_e;
    break;
  }

  Autopilot_Y.out.time = Autopilot_U.in.time;
  Autopilot_Y.out.input = Autopilot_U.in.input;
  Autopilot_Y.out.data = Autopilot_U.in.data;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_DWork.Delay_DSTATE;
  Autopilot_DWork.icLoad_a = 0U;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_DWork.Delay_DSTATE_a;
  Autopilot_DWork.icLoad_k = 0U;
  Autopilot_DWork.Delay_DSTATE_d = Autopilot_DWork.Delay_DSTATE_p;
  Autopilot_DWork.Delay_DSTATE_i = rtb_Gain1_kh;
  Autopilot_DWork.Delay_DSTATE_n = rtb_IC;
  Autopilot_DWork.icLoad_l = 0U;
  Autopilot_DWork.Delay_DSTATE_pw = Autopilot_DWork.Delay_DSTATE_e;
  Autopilot_DWork.icLoad_aw = 0U;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_DWork.Delay_DSTATE_py;
  Autopilot_DWork.icLoad_m = 0U;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_DWork.Delay_DSTATE_m;
  Autopilot_DWork.icLoad_kc = 0U;
  Autopilot_DWork.Delay_DSTATE_o = Autopilot_DWork.Delay_DSTATE_ip;
}

void AutopilotModelClass::initialize()
{
  (void) std::memset(static_cast<void *>(&Autopilot_DWork), 0,
                     sizeof(D_Work_Autopilot_T));
  Autopilot_U.in = Autopilot_rtZap_input;
  Autopilot_Y.out = Autopilot_rtZap_output;
  Autopilot_DWork.IC_FirstOutputTime = true;
  Autopilot_DWork.IC_FirstOutputTime_p = true;
  Autopilot_DWork.IC_FirstOutputTime_c = true;
  Autopilot_DWork.IC_FirstOutputTime_cl = true;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.icLoad_a = 1U;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_P.Delay_InitialCondition_h;
  Autopilot_DWork.Delay1_DSTATE_b = Autopilot_P.Delay1_InitialCondition_a;
  Autopilot_DWork.icLoad_k = 1U;
  Autopilot_DWork.Delay_DSTATE_d = Autopilot_P.Delay_InitialCondition_m;
  Autopilot_DWork.Delay1_DSTATE_g = Autopilot_P.Delay1_InitialCondition_g;
  Autopilot_DWork.Delay_DSTATE_i = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_n = Autopilot_P.Delay_InitialCondition_f;
  Autopilot_DWork.Delay1_DSTATE_e = Autopilot_P.Delay1_InitialCondition_f;
  Autopilot_DWork.icLoad_l = 1U;
  Autopilot_DWork.Delay_DSTATE_pw = Autopilot_P.Delay_InitialCondition_g;
  Autopilot_DWork.Delay1_DSTATE_n = Autopilot_P.Delay1_InitialCondition_l;
  Autopilot_DWork.icLoad_aw = 1U;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_P.Delay_InitialCondition_gs;
  Autopilot_DWork.Delay1_DSTATE_e1 = Autopilot_P.Delay1_InitialCondition_o;
  Autopilot_DWork.icLoad_m = 1U;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.Delay_InitialCondition_i;
  Autopilot_DWork.Delay1_DSTATE_o = Autopilot_P.Delay1_InitialCondition_h;
  Autopilot_DWork.icLoad_kc = 1U;
  Autopilot_DWork.Delay_DSTATE_o = Autopilot_P.Delay_InitialCondition_o;
  Autopilot_DWork.Delay1_DSTATE_l = Autopilot_P.Delay1_InitialCondition_p;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c1_Autopilot = 0U;
  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c4_Autopilot = 0U;
  Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c2_Autopilot = 0U;
  Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_h);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_m);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_i);
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
