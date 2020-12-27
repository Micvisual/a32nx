#include "Autopilot.h"
#include "Autopilot_private.h"

const uint8_T Autopilot_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T Autopilot_IN_any = 1U;
const uint8_T Autopilot_IN_left = 2U;
const uint8_T Autopilot_IN_right = 3U;
const uint8_T Autopilot_IN_NO_ACTIVE_CHILD_a = 0U;
const uint8_T Autopilot_IN_OFF = 1U;
const uint8_T Autopilot_IN_ON = 2U;
const uint8_T Autopilot_IN_COND = 1U;
const uint8_T Autopilot_IN_LOC = 1U;
const uint8_T Autopilot_IN_LOC_CPT = 2U;
const uint8_T Autopilot_IN_LOC_TRK = 3U;
const uint8_T Autopilot_IN_MANAGED = 2U;
const uint8_T Autopilot_IN_SELECTED = 3U;
const uint8_T Autopilot_IN_ALT_ACQ = 1U;
const uint8_T Autopilot_IN_ALT_HOLD = 2U;
const uint8_T Autopilot_IN_GS = 3U;
const uint8_T Autopilot_IN_SPD_MACH = 4U;
const uint8_T Autopilot_IN_VS = 5U;
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
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c5_Autopilot = Autopilot_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
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
  real_T rtb_Tsxlo;
  real_T rtb_Gain_p;
  real_T rtb_Gain;
  real_T rtb_Sum3;
  real_T rtb_Mod1_m;
  int32_T rtb_out_e;
  real_T rtb_kntoms;
  real_T rtb_IC;
  real_T rtb_kntoms_l;
  real_T rtb_IC_e;
  real_T rtb_kntoms_b;
  real_T rtb_IC_g;
  real_T rtb_kntoms_e;
  real_T rtb_IC_h;
  int32_T rtb_LAW;
  real_T rtb_Sum_f;
  real_T rtb_out_il;
  real_T rtb_out;
  real_T rtb_out_m;
  real_T rtb_out_j;
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
    rtb_LAW = 1;
  } else {
    switch (Autopilot_DWork.is_c1_Autopilot) {
     case Autopilot_IN_ALT_ACQ:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= 20.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_LAW = 1;
      } else {
        rtb_LAW = 2;
      }
      break;

     case Autopilot_IN_ALT_HOLD:
      if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_GS;
        rtb_LAW = 5;
      } else if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) > 250.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_LAW = 3;
      } else {
        rtb_LAW = 1;
      }
      break;

     case Autopilot_IN_GS:
      if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_LAW = 1;
      } else {
        rtb_LAW = 5;
      }
      break;

     case Autopilot_IN_SPD_MACH:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_vs_mode == 1.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_VS;
        rtb_LAW = 4;
      } else {
        rtb_LAW = 3;
      }
      break;

     default:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_alt_mode != 0.0) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_LAW = 3;
      } else {
        rtb_LAW = 4;
      }
      break;
    }
  }

  rtb_Gain_p = Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft;
  rtb_Gain = Autopilot_P.Gain_Gain_jt * rtb_Gain_p;
  rtb_Sum = Autopilot_U.in.data.ap_V_c_kn - Autopilot_U.in.data.V_ias_kn;
  if (Autopilot_DWork.is_active_c4_Autopilot == 0U) {
    Autopilot_DWork.is_active_c4_Autopilot = 1U;
    Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
    rtb_mode = 0;
  } else {
    switch (Autopilot_DWork.is_c4_Autopilot) {
     case Autopilot_IN_ALT:
      rtb_Sum = std::abs(rtb_Gain_p);
      if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else if ((rtb_Sum > 250.0) && (rtb_Sum <= 1200.0) && (rtb_Sum > std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0))
      {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_SMALL;
        rtb_mode = 0;
        if (rtb_Gain_p < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_p > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_p;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      } else {
        rtb_mode = 0;
      }
      break;

     case Autopilot_IN_OP:
      if (rtb_Gain_p < 0.0) {
        rtb_Sum3 = -1.0;
      } else if (rtb_Gain_p > 0.0) {
        rtb_Sum3 = 1.0;
      } else {
        rtb_Sum3 = rtb_Gain_p;
      }

      if (rtb_Sum < 0.0) {
        rtb_Mod1_m = -1.0;
      } else if (rtb_Sum > 0.0) {
        rtb_Mod1_m = 1.0;
      } else {
        rtb_Mod1_m = rtb_Sum;
      }

      if ((rtb_Sum3 == rtb_Mod1_m) && (std::abs(rtb_Sum) >= 20.0)) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_LD;
        rtb_mode = 0;
        if (rtb_Gain_p < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_p > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_p;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      } else if (std::abs(rtb_Gain_p) <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
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
        if (rtb_Gain_p < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_p > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_p;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      }
      break;

     default:
      rtb_Sum = std::abs(rtb_Gain_p);
      if (rtb_Sum <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
        rtb_mode = 0;
      } else if (rtb_Sum > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else {
        rtb_mode = 0;
        if (rtb_Gain_p < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_p > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_p;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      }
      break;
    }
  }

  if (Autopilot_DWork.is_active_c2_Autopilot == 0U) {
    Autopilot_DWork.is_active_c2_Autopilot = 1U;
    Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
    Autopilot_B.LAW = 1.0;
  } else {
    switch (Autopilot_DWork.is_c2_Autopilot) {
     case Autopilot_IN_LOC:
      if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
        Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_a;
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
        Autopilot_B.LAW = 1.0;
      } else if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
        Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_a;
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_MANAGED;
        Autopilot_B.LAW = 3.0;
      } else {
        switch (Autopilot_DWork.is_LOC) {
         case Autopilot_IN_COND:
          if (Autopilot_U.in.time.simulation_time - Autopilot_DWork.loc_trk_time >= 10.0) {
            Autopilot_DWork.is_LOC = Autopilot_IN_LOC_TRK;
            Autopilot_B.LAW = 5.0;
          } else {
            if (std::abs(Autopilot_U.in.data.nav_radial_error_deg) >= 0.16) {
              Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
              Autopilot_B.LAW = 4.0;
            }
          }
          break;

         case Autopilot_IN_LOC_CPT:
          if (std::abs(Autopilot_U.in.data.nav_radial_error_deg) < 0.16) {
            Autopilot_DWork.loc_trk_time = Autopilot_U.in.time.simulation_time;
            Autopilot_DWork.is_LOC = Autopilot_IN_COND;
          } else {
            Autopilot_B.LAW = 4.0;
          }
          break;

         default:
          Autopilot_B.LAW = 5.0;
          break;
        }
      }
      break;

     case Autopilot_IN_MANAGED:
      if (Autopilot_U.in.input.trigger_loc == 1.0) {
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_LOC;
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
        Autopilot_B.LAW = 4.0;
      } else if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_SELECTED;
        Autopilot_B.LAW = 1.0;
      } else {
        Autopilot_B.LAW = 3.0;
      }
      break;

     default:
      if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_MANAGED;
        Autopilot_B.LAW = 3.0;
      } else if (Autopilot_U.in.input.trigger_loc == 1.0) {
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_LOC;
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
        Autopilot_B.LAW = 4.0;
      } else {
        Autopilot_B.LAW = 1.0;
      }
      break;
    }
  }

  rtb_Sum = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value)) + Autopilot_P.Constant3_Value, Autopilot_P.Constant3_Value);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_b * rt_modd(Autopilot_P.Constant3_Value - rtb_Sum,
    Autopilot_P.Constant3_Value), Autopilot_P.Constant_Value_e, &rtb_out_il, &Autopilot_DWork.sf_Chart_i);
  rtb_Sum = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_c)) + Autopilot_P.Constant3_Value_c, Autopilot_P.Constant3_Value_c);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_mr * rt_modd(Autopilot_P.Constant3_Value_c - rtb_Sum,
    Autopilot_P.Constant3_Value_c), Autopilot_P.Constant_Value_k, &rtb_out, &Autopilot_DWork.sf_Chart_h);
  rtb_Tsxlo = Autopilot_P.Gain_Gain_a * Autopilot_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat_k) {
    rtb_Tsxlo = Autopilot_P.Saturation_UpperSat_k;
  } else {
    if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat_g) {
      rtb_Tsxlo = Autopilot_P.Saturation_LowerSat_g;
    }
  }

  rtb_Sum = rt_modd((rt_modd((rtb_Tsxlo + Autopilot_U.in.data.flight_guidance_tae_deg) * Autopilot_P.Gain1_Gain_p +
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_m), Autopilot_P.Constant_Value_m) -
                     (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_cl)) +
                    Autopilot_P.Constant3_Value_cl, Autopilot_P.Constant3_Value_cl);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_f * rt_modd(Autopilot_P.Constant3_Value_cl - rtb_Sum,
    Autopilot_P.Constant3_Value_cl), Autopilot_P.Constant_Value_g, &rtb_out_m, &Autopilot_DWork.sf_Chart_b);
  rtb_Tsxlo = std::sin(Autopilot_P.Gain1_Gain_o * Autopilot_U.in.data.nav_radial_error_deg) *
    Autopilot_U.in.data.nav_dme_nmi * Autopilot_P.Gain2_Gain_p;
  if (rtb_Tsxlo > Autopilot_P.Saturation1_UpperSat) {
    rtb_Tsxlo = Autopilot_P.Saturation1_UpperSat;
  } else {
    if (rtb_Tsxlo < Autopilot_P.Saturation1_LowerSat) {
      rtb_Tsxlo = Autopilot_P.Saturation1_LowerSat;
    }
  }

  rtb_Sum = rt_modd((rt_modd((((Autopilot_U.in.data.nav_radial_error_deg + Autopilot_U.in.data.nav_loc_deg) -
    Autopilot_U.in.data.Psi_magnetic_track_deg) + rtb_Tsxlo) * Autopilot_P.Gain3_Gain +
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_o), Autopilot_P.Constant_Value_o) -
                     (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_k)) +
                    Autopilot_P.Constant3_Value_k, Autopilot_P.Constant3_Value_k);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_az * rt_modd(Autopilot_P.Constant3_Value_k - rtb_Sum,
    Autopilot_P.Constant3_Value_k), Autopilot_P.Constant_Value_b, &rtb_out_j, &Autopilot_DWork.sf_Chart_dz);
  rtb_Sum = Autopilot_P.DiscreteDerivativeVariableTs_Gain * Autopilot_U.in.data.nav_radial_error_deg;
  rtb_Sum3 = (rtb_Sum - Autopilot_DWork.Delay_DSTATE) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_o +
    Autopilot_U.in.data.nav_radial_error_deg;
  rtb_Mod1_m = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1;
  rtb_Tsxlo = rtb_Mod1_m + Autopilot_P.Constant_Value_by;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_by - rtb_Mod1_m) *
    Autopilot_DWork.Delay1_DSTATE + (rtb_Sum3 + Autopilot_DWork.Delay_DSTATE_g) * (rtb_Mod1_m / rtb_Tsxlo);
  rtb_Mod1_m = rt_modd((rt_modd(Autopilot_P.Gain4_Gain_f * Autopilot_DWork.Delay1_DSTATE +
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_h), Autopilot_P.Constant_Value_h) -
                        (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_cq)) +
                       Autopilot_P.Constant3_Value_cq, Autopilot_P.Constant3_Value_cq);
  Autopilot_Chart(rtb_Mod1_m, Autopilot_P.Gain_Gain_n * rt_modd(Autopilot_P.Constant3_Value_cq - rtb_Mod1_m,
    Autopilot_P.Constant3_Value_cq), Autopilot_P.Constant_Value_k5, &rtb_Tsxlo, &Autopilot_DWork.sf_Chart_k);
  switch (static_cast<int32_T>(Autopilot_B.LAW)) {
   case 1:
    rtb_Mod1_m = rtb_out_il * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1, Autopilot_P.ScheduledGain_Table, 3U);
    break;

   case 2:
    rtb_Mod1_m = rtb_out * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_n, Autopilot_P.ScheduledGain_Table_n, 3U);
    break;

   case 3:
    rtb_Mod1_m = rtb_out_m * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_nx, Autopilot_P.ScheduledGain_Table_g, 3U);
    break;

   case 4:
    rtb_Mod1_m = rtb_out_j * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_e, Autopilot_P.ScheduledGain_Table_c, 3U);
    break;

   default:
    rtb_Mod1_m = rtb_Tsxlo * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_g, Autopilot_P.ScheduledGain_Table_m, 3U);
    break;
  }

  rtb_Tsxlo = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data, Autopilot_P.ROLLLIM1_tableData,
    4U);
  if (rtb_Mod1_m > rtb_Tsxlo) {
    rtb_Mod1_m = rtb_Tsxlo;
  } else {
    rtb_Tsxlo *= Autopilot_P.Gain1_Gain;
    if (rtb_Mod1_m < rtb_Tsxlo) {
      rtb_Mod1_m = rtb_Tsxlo;
    }
  }

  rtb_out_il = Autopilot_P.Gain_Gain_by * Autopilot_U.in.data.Phi_deg;
  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad = 1U;
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE_f = rtb_out_il;
  }

  rtb_Tsxlo = rtb_Mod1_m - Autopilot_DWork.Delay_DSTATE_f;
  rtb_kntoms = Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms) {
    rtb_kntoms = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_P.Gain1_Gain_d * Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_kntoms > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_f += rtb_Tsxlo;
  rtb_out = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_o;
  rtb_Tsxlo = rtb_out + Autopilot_P.Constant_Value_d;
  Autopilot_DWork.Delay1_DSTATE_h = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_d - rtb_out) *
    Autopilot_DWork.Delay1_DSTATE_h + (Autopilot_DWork.Delay_DSTATE_f + Autopilot_DWork.Delay_DSTATE_h) * (rtb_out /
    rtb_Tsxlo);
  rtb_Tsxlo = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_k;
  rtb_kntoms = Autopilot_P.RateLimiterVariableTs_up * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms) {
    rtb_kntoms = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo;
  if (rtb_kntoms > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_k += rtb_Tsxlo;
  if (Autopilot_DWork.Delay_DSTATE_k > Autopilot_P.Saturation_UpperSat_d) {
    rtb_out = Autopilot_P.Saturation_UpperSat_d;
  } else if (Autopilot_DWork.Delay_DSTATE_k < Autopilot_P.Saturation_LowerSat_d) {
    rtb_out = Autopilot_P.Saturation_LowerSat_d;
  } else {
    rtb_out = Autopilot_DWork.Delay_DSTATE_k;
  }

  Autopilot_Y.out.output.autopilot.Phi_c_deg = (Autopilot_P.Constant_Value_dc - rtb_out) * rtb_out_il +
    Autopilot_DWork.Delay1_DSTATE_h * rtb_out;
  rtb_kntoms = Autopilot_P.kntoms_Gain * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime) {
    Autopilot_DWork.IC_FirstOutputTime = false;
    rtb_IC = Autopilot_P.IC_Value;
  } else {
    rtb_IC = rtb_kntoms;
  }

  rtb_kntoms_l = Autopilot_P.kntoms_Gain_e * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_j) {
    Autopilot_DWork.IC_FirstOutputTime_j = false;
    rtb_IC_e = Autopilot_P.IC_Value_l;
  } else {
    rtb_IC_e = rtb_kntoms_l;
  }

  rtb_out_il = Autopilot_P.DiscreteDerivativeVariableTs_Gain_e * Autopilot_U.in.data.V_ias_kn;
  rtb_out = (rtb_out_il - Autopilot_DWork.Delay_DSTATE_m) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_h +
    Autopilot_U.in.data.V_ias_kn;
  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_Sum_f = rtb_Tsxlo + Autopilot_P.Constant_Value_n;
  Autopilot_DWork.Delay1_DSTATE_e = 1.0 / rtb_Sum_f * (Autopilot_P.Constant_Value_n - rtb_Tsxlo) *
    Autopilot_DWork.Delay1_DSTATE_e + (rtb_out + Autopilot_DWork.Delay_DSTATE_a) * (rtb_Tsxlo / rtb_Sum_f);
  rtb_kntoms_b = Autopilot_P.kntoms_Gain_a * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_h) {
    Autopilot_DWork.IC_FirstOutputTime_h = false;
    rtb_IC_g = Autopilot_P.IC_Value_a;
  } else {
    rtb_IC_g = rtb_kntoms_b;
  }

  rtb_kntoms_e = Autopilot_P.kntoms_Gain_p * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_b) {
    Autopilot_DWork.IC_FirstOutputTime_b = false;
    rtb_IC_h = Autopilot_P.IC_Value_k;
  } else {
    rtb_IC_h = rtb_kntoms_e;
  }

  rtb_out_m = Autopilot_P.DiscreteDerivativeVariableTs_Gain_o * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_out_j = (rtb_out_m - Autopilot_DWork.Delay_DSTATE_at) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_a +
    Autopilot_P.Gain1_Gain_kq * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_Sum_f = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_m;
  rtb_Tsxlo = rtb_Sum_f + Autopilot_P.Constant_Value_i;
  Autopilot_DWork.Delay1_DSTATE_j = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_i - rtb_Sum_f) *
    Autopilot_DWork.Delay1_DSTATE_j + (rtb_out_j + Autopilot_DWork.Delay_DSTATE_gp) * (rtb_Sum_f / rtb_Tsxlo);
  switch (rtb_LAW) {
   case 1:
    rtb_Tsxlo = rtb_Gain_p * Autopilot_P.Gain_Gain_ml;
    if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat_i) {
      rtb_Tsxlo = Autopilot_P.Saturation_UpperSat_i;
    } else {
      if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat_f) {
        rtb_Tsxlo = Autopilot_P.Saturation_LowerSat_f;
      }
    }

    rtb_Tsxlo = (rtb_Tsxlo - std::sin((Autopilot_P.Gain2_Gain_kp * Autopilot_U.in.data.Theta_deg - std::cos
      (Autopilot_P.Gain1_Gain_i * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_k3) * rtb_kntoms * Autopilot_P.msftmin_Gain_n) * Autopilot_P.ftmintoms_Gain_m / rtb_IC;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_f = Autopilot_P.Gain_Gain_p * std::asin(rtb_Tsxlo);
    break;

   case 2:
    if (rtb_Gain_p < 0.0) {
      rtb_Gain = -1.0;
    } else if (rtb_Gain_p > 0.0) {
      rtb_Gain = 1.0;
    } else {
      rtb_Gain = rtb_Gain_p;
    }

    rtb_Tsxlo = ((Autopilot_P.Constant_Value * rtb_Gain + rtb_Gain_p) * Autopilot_P.Gain_Gain_m - std::sin
                 ((Autopilot_P.Gain2_Gain_k1 * Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_ca *
      Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_f) * rtb_kntoms_l *
                 Autopilot_P.msftmin_Gain_c) * Autopilot_P.ftmintoms_Gain_k / rtb_IC_e;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_f = Autopilot_P.Gain_Gain_g * std::asin(rtb_Tsxlo);
    break;

   case 3:
    if (rtb_mode > Autopilot_P.Switch_Threshold) {
      rtb_Tsxlo = Autopilot_DWork.Delay1_DSTATE_e - Autopilot_U.in.data.ap_V_c_kn;
      if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat) {
        rtb_Tsxlo = Autopilot_P.Saturation_UpperSat;
      } else {
        if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat) {
          rtb_Tsxlo = Autopilot_P.Saturation_LowerSat;
        }
      }

      rtb_Sum_f = Autopilot_P.Gain1_Gain_g * rtb_Tsxlo;
    } else {
      rtb_Tsxlo = (rtb_Gain - std::sin((Autopilot_P.Gain2_Gain_k * Autopilot_U.in.data.Theta_deg - std::cos
        (Autopilot_P.Gain1_Gain_h * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
        Autopilot_P.Gain1_Gain_cz) * rtb_kntoms_b * Autopilot_P.msftmin_Gain_h) * Autopilot_P.ftmintoms_Gain_j /
        rtb_IC_g;
      if (rtb_Tsxlo > 1.0) {
        rtb_Tsxlo = 1.0;
      } else {
        if (rtb_Tsxlo < -1.0) {
          rtb_Tsxlo = -1.0;
        }
      }

      rtb_Sum_f = Autopilot_P.Gain_Gain_j * std::asin(rtb_Tsxlo);
    }
    break;

   case 4:
    rtb_Tsxlo = (Autopilot_U.in.data.ap_H_dot_c_ft_min - std::sin((Autopilot_P.Gain2_Gain *
      Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_c * Autopilot_U.in.data.Phi_deg) *
      Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_k) * rtb_kntoms_e * Autopilot_P.msftmin_Gain) *
      Autopilot_P.ftmintoms_Gain / rtb_IC_h;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_f = Autopilot_P.Gain_Gain * std::asin(rtb_Tsxlo);
    break;

   default:
    rtb_Sum_f = Autopilot_P.Gain4_Gain * Autopilot_DWork.Delay1_DSTATE_j;
    break;
  }

  rtb_Gain_p = Autopilot_P.Gain_Gain_nw * Autopilot_U.in.data.Theta_deg;
  rtb_Sum_f += rtb_Gain_p;
  if (rtb_Sum_f > Autopilot_P.Constant1_Value) {
    rtb_Sum_f = Autopilot_P.Constant1_Value;
  } else {
    rtb_Gain = Autopilot_P.Gain1_Gain_im * Autopilot_P.Constant1_Value;
    if (rtb_Sum_f < rtb_Gain) {
      rtb_Sum_f = rtb_Gain;
    }
  }

  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad_b = 1U;
  }

  if (Autopilot_DWork.icLoad_b != 0) {
    Autopilot_DWork.Delay_DSTATE_d = rtb_Gain_p;
  }

  rtb_Tsxlo = rtb_Sum_f - Autopilot_DWork.Delay_DSTATE_d;
  rtb_kntoms = Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms) {
    rtb_kntoms = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_P.Gain1_Gain_m * Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_kntoms > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_d += rtb_Tsxlo;
  rtb_Gain = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_n;
  rtb_Tsxlo = rtb_Gain + Autopilot_P.Constant_Value_l;
  Autopilot_DWork.Delay1_DSTATE_m = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_l - rtb_Gain) *
    Autopilot_DWork.Delay1_DSTATE_m + (Autopilot_DWork.Delay_DSTATE_d + Autopilot_DWork.Delay_DSTATE_o) * (rtb_Gain /
    rtb_Tsxlo);
  rtb_Tsxlo = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_l;
  rtb_kntoms = Autopilot_P.RateLimiterVariableTs_up_k * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms) {
    rtb_kntoms = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo_h;
  if (rtb_kntoms > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms;
  }

  Autopilot_DWork.Delay_DSTATE_l += rtb_Tsxlo;
  if (Autopilot_DWork.Delay_DSTATE_l > Autopilot_P.Saturation_UpperSat_h) {
    rtb_Gain = Autopilot_P.Saturation_UpperSat_h;
  } else if (Autopilot_DWork.Delay_DSTATE_l < Autopilot_P.Saturation_LowerSat_fk) {
    rtb_Gain = Autopilot_P.Saturation_LowerSat_fk;
  } else {
    rtb_Gain = Autopilot_DWork.Delay_DSTATE_l;
  }

  Autopilot_Y.out.output.autopilot.Theta_c_deg = (Autopilot_P.Constant_Value_na - rtb_Gain) * rtb_Gain_p +
    Autopilot_DWork.Delay1_DSTATE_m * rtb_Gain;
  Autopilot_Y.out.time = Autopilot_U.in.time;
  Autopilot_Y.out.input = Autopilot_U.in.input;
  Autopilot_Y.out.data = Autopilot_U.in.data;
  Autopilot_Y.out.output.ap_on = rtb_out_e;
  Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_f;
  Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_Mod1_m;
  Autopilot_DWork.Delay_DSTATE = rtb_Sum;
  Autopilot_DWork.Delay_DSTATE_g = rtb_Sum3;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_h = Autopilot_DWork.Delay_DSTATE_f;
  Autopilot_DWork.Delay_DSTATE_m = rtb_out_il;
  Autopilot_DWork.Delay_DSTATE_a = rtb_out;
  Autopilot_DWork.Delay_DSTATE_at = rtb_out_m;
  Autopilot_DWork.Delay_DSTATE_gp = rtb_out_j;
  Autopilot_DWork.icLoad_b = 0U;
  Autopilot_DWork.Delay_DSTATE_o = Autopilot_DWork.Delay_DSTATE_d;
}

void AutopilotModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&Autopilot_B)), 0,
                     sizeof(BlockIO_Autopilot_T));
  (void) std::memset(static_cast<void *>(&Autopilot_DWork), 0,
                     sizeof(D_Work_Autopilot_T));
  Autopilot_U.in = Autopilot_rtZap_input;
  Autopilot_Y.out = Autopilot_rtZap_output;
  Autopilot_DWork.IC_FirstOutputTime = true;
  Autopilot_DWork.IC_FirstOutputTime_j = true;
  Autopilot_DWork.IC_FirstOutputTime_h = true;
  Autopilot_DWork.IC_FirstOutputTime_b = true;
  Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.Delay_InitialCondition_p;
  Autopilot_DWork.Delay1_DSTATE_h = Autopilot_P.Delay1_InitialCondition_k;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.RateLimiterVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_m = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_m;
  Autopilot_DWork.Delay_DSTATE_a = Autopilot_P.Delay_InitialCondition_i;
  Autopilot_DWork.Delay1_DSTATE_e = Autopilot_P.Delay1_InitialCondition_o;
  Autopilot_DWork.Delay_DSTATE_at = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_p;
  Autopilot_DWork.Delay_DSTATE_gp = Autopilot_P.Delay_InitialCondition_b;
  Autopilot_DWork.Delay1_DSTATE_j = Autopilot_P.Delay1_InitialCondition_l;
  Autopilot_DWork.icLoad_b = 1U;
  Autopilot_DWork.Delay_DSTATE_o = Autopilot_P.Delay_InitialCondition_m;
  Autopilot_DWork.Delay1_DSTATE_m = Autopilot_P.Delay1_InitialCondition_e;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_P.RateLimiterVariableTs_InitialCondition_i;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c1_Autopilot = 0U;
  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c4_Autopilot = 0U;
  Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c2_Autopilot = 0U;
  Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_i);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_h);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_b);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_dz);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_k);
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
