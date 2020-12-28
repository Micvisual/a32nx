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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

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
  real_T x[3];
  real_T rtb_Tsxlo;
  real_T rtb_Product_a;
  real_T rtb_Gain_hc;
  real_T rtb_Gain;
  real_T rtb_Sum3;
  real_T rtb_Mod1_n;
  int32_T rtb_out_e;
  real_T rtb_kntoms;
  real_T rtb_IC;
  real_T rtb_kntoms_l;
  real_T rtb_IC_h;
  real_T rtb_kntoms_b;
  real_T rtb_IC_fz;
  real_T rtb_kntoms_e;
  real_T rtb_IC_n;
  real_T rtb_Sum4_p;
  real_T rtb_Sum_ij;
  real_T rtb_out_i;
  real_T rtb_out_g5;
  real_T rtb_out;
  real_T rtb_out_a;
  real_T rtb_out_g;
  int32_T rtb_LAW;
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

  rtb_Gain_hc = Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft;
  rtb_Gain = Autopilot_P.Gain_Gain_j * rtb_Gain_hc;
  rtb_Sum = Autopilot_U.in.data.ap_V_c_kn - Autopilot_U.in.data.V_ias_kn;
  if (Autopilot_DWork.is_active_c4_Autopilot == 0U) {
    Autopilot_DWork.is_active_c4_Autopilot = 1U;
    Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
    rtb_mode = 0;
  } else {
    switch (Autopilot_DWork.is_c4_Autopilot) {
     case Autopilot_IN_ALT:
      rtb_Tsxlo = std::abs(rtb_Gain_hc);
      if (rtb_Tsxlo > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else if ((rtb_Tsxlo > 250.0) && (rtb_Tsxlo <= 1200.0) && (rtb_Tsxlo > std::abs(Autopilot_U.in.data.H_dot_ft_min)
                  / 8.0)) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_SMALL;
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      } else {
        rtb_mode = 0;
      }
      break;

     case Autopilot_IN_OP:
      if (rtb_Gain_hc < 0.0) {
        rtb_Sum3 = -1.0;
      } else if (rtb_Gain_hc > 0.0) {
        rtb_Sum3 = 1.0;
      } else {
        rtb_Sum3 = rtb_Gain_hc;
      }

      if (rtb_Sum < 0.0) {
        rtb_out_i = -1.0;
      } else if (rtb_Sum > 0.0) {
        rtb_out_i = 1.0;
      } else {
        rtb_out_i = rtb_Sum;
      }

      if ((rtb_Sum3 == rtb_out_i) && (std::abs(rtb_Sum) >= 20.0)) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP_LD;
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
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
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_hc;
        }

        rtb_Gain = rtb_Sum3 * 1000.0;
      }
      break;

     default:
      rtb_Tsxlo = std::abs(rtb_Gain_hc);
      if (rtb_Tsxlo <= std::abs(Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ALT;
        rtb_mode = 0;
      } else if (rtb_Tsxlo > 1200.0) {
        Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OP;
        rtb_mode = 1;
      } else {
        rtb_mode = 0;
        if (rtb_Gain_hc < 0.0) {
          rtb_Sum3 = -1.0;
        } else if (rtb_Gain_hc > 0.0) {
          rtb_Sum3 = 1.0;
        } else {
          rtb_Sum3 = rtb_Gain_hc;
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
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_m2 * rt_modd(Autopilot_P.Constant3_Value - rtb_Sum,
    Autopilot_P.Constant3_Value), Autopilot_P.Constant_Value_e, &rtb_out_g5, &Autopilot_DWork.sf_Chart_c);
  rtb_Sum = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_i)) + Autopilot_P.Constant3_Value_i, Autopilot_P.Constant3_Value_i);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_b * rt_modd(Autopilot_P.Constant3_Value_i - rtb_Sum,
    Autopilot_P.Constant3_Value_i), Autopilot_P.Constant_Value_k, &rtb_out, &Autopilot_DWork.sf_Chart_f);
  rtb_Tsxlo = Autopilot_P.Gain_Gain_a * Autopilot_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat_k) {
    rtb_Tsxlo = Autopilot_P.Saturation_UpperSat_k;
  } else {
    if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat_g) {
      rtb_Tsxlo = Autopilot_P.Saturation_LowerSat_g;
    }
  }

  rtb_Sum = rt_modd((rt_modd((Autopilot_P.Gain2_Gain_a * Autopilot_U.in.data.flight_guidance_tae_deg + rtb_Tsxlo) *
    Autopilot_P.Gain1_Gain_pa + (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_eg),
    Autopilot_P.Constant_Value_eg) - (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_b)) +
                    Autopilot_P.Constant3_Value_b, Autopilot_P.Constant3_Value_b);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_cz * rt_modd(Autopilot_P.Constant3_Value_b - rtb_Sum,
    Autopilot_P.Constant3_Value_b), Autopilot_P.Constant_Value_g, &rtb_out_a, &Autopilot_DWork.sf_Chart_e);
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
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_p), Autopilot_P.Constant_Value_p) -
                     (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_m)) +
                    Autopilot_P.Constant3_Value_m, Autopilot_P.Constant3_Value_m);
  Autopilot_Chart(rtb_Sum, Autopilot_P.Gain_Gain_h * rt_modd(Autopilot_P.Constant3_Value_m - rtb_Sum,
    Autopilot_P.Constant3_Value_m), Autopilot_P.Constant_Value_b, &rtb_out_g, &Autopilot_DWork.sf_Chart_i);
  rtb_Sum = Autopilot_P.DiscreteDerivativeVariableTs_Gain * Autopilot_U.in.data.nav_radial_error_deg;
  rtb_Sum3 = (rtb_Sum - Autopilot_DWork.Delay_DSTATE) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_o +
    Autopilot_U.in.data.nav_radial_error_deg;
  rtb_out_i = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1;
  rtb_Tsxlo = rtb_out_i + Autopilot_P.Constant_Value_i;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_i - rtb_out_i) *
    Autopilot_DWork.Delay1_DSTATE + (rtb_Sum3 + Autopilot_DWork.Delay_DSTATE_j) * (rtb_out_i / rtb_Tsxlo);
  rtb_Mod1_n = rt_modd((rt_modd(Autopilot_P.Gain4_Gain_f * Autopilot_DWork.Delay1_DSTATE +
    (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant_Value_n), Autopilot_P.Constant_Value_n) -
                        (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_m0)) +
                       Autopilot_P.Constant3_Value_m0, Autopilot_P.Constant3_Value_m0);
  Autopilot_Chart(rtb_Mod1_n, Autopilot_P.Gain_Gain_n * rt_modd(Autopilot_P.Constant3_Value_m0 - rtb_Mod1_n,
    Autopilot_P.Constant3_Value_m0), Autopilot_P.Constant_Value_k5, &rtb_out_i, &Autopilot_DWork.sf_Chart_g);
  if (Autopilot_P.ManualSwitch_CurrentSetting == 1) {
    rtb_Tsxlo = Autopilot_P.Constant_Value;
  } else {
    rtb_Tsxlo = Autopilot_B.LAW;
  }

  switch (static_cast<int32_T>(rtb_Tsxlo)) {
   case 1:
    rtb_out_i = rtb_out_g5 * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1, Autopilot_P.ScheduledGain_Table, 3U);
    break;

   case 2:
    rtb_out_i = rtb_out * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_e, Autopilot_P.ScheduledGain_Table_e, 3U);
    break;

   case 3:
    rtb_out_i = rtb_out_a * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_f, Autopilot_P.ScheduledGain_Table_o, 3U);
    break;

   case 4:
    rtb_out_i = rtb_out_g * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_f0, Autopilot_P.ScheduledGain_Table_i, 3U);
    break;

   default:
    rtb_out_i *= look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ScheduledGain_BreakpointsForDimension1_b,
      Autopilot_P.ScheduledGain_Table_n, 3U);
    break;
  }

  rtb_out_g5 = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data, Autopilot_P.ROLLLIM1_tableData,
    4U);
  if (rtb_out_i > rtb_out_g5) {
    rtb_out_i = rtb_out_g5;
  } else {
    rtb_out_g5 *= Autopilot_P.Gain1_Gain;
    if (rtb_out_i < rtb_out_g5) {
      rtb_out_i = rtb_out_g5;
    }
  }

  rtb_out_g5 = Autopilot_P.Gain_Gain_by * Autopilot_U.in.data.Phi_deg;
  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad = 1U;
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE_jv = rtb_out_g5;
  }

  rtb_Tsxlo = rtb_out_i - Autopilot_DWork.Delay_DSTATE_jv;
  rtb_kntoms_l = Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms_l) {
    rtb_kntoms_l = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_P.Gain1_Gain_ja * Autopilot_P.Constant2_Value * Autopilot_U.in.time.dt;
  if (rtb_kntoms_l > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms_l;
  }

  Autopilot_DWork.Delay_DSTATE_jv += rtb_Tsxlo;
  rtb_out = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_p;
  rtb_Tsxlo = rtb_out + Autopilot_P.Constant_Value_gn;
  Autopilot_DWork.Delay1_DSTATE_m = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_gn - rtb_out) *
    Autopilot_DWork.Delay1_DSTATE_m + (Autopilot_DWork.Delay_DSTATE_jv + Autopilot_DWork.Delay_DSTATE_b) * (rtb_out /
    rtb_Tsxlo);
  rtb_Tsxlo = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_jv5;
  rtb_kntoms_l = Autopilot_P.RateLimiterVariableTs_up * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms_l) {
    rtb_kntoms_l = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo;
  if (rtb_kntoms_l > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms_l;
  }

  Autopilot_DWork.Delay_DSTATE_jv5 += rtb_Tsxlo;
  if (Autopilot_DWork.Delay_DSTATE_jv5 > Autopilot_P.Saturation_UpperSat_c) {
    rtb_out = Autopilot_P.Saturation_UpperSat_c;
  } else if (Autopilot_DWork.Delay_DSTATE_jv5 < Autopilot_P.Saturation_LowerSat_fc) {
    rtb_out = Autopilot_P.Saturation_LowerSat_fc;
  } else {
    rtb_out = Autopilot_DWork.Delay_DSTATE_jv5;
  }

  Autopilot_Y.out.output.autopilot.Phi_c_deg = (Autopilot_P.Constant_Value_a - rtb_out) * rtb_out_g5 +
    Autopilot_DWork.Delay1_DSTATE_m * rtb_out;
  rtb_kntoms = Autopilot_P.kntoms_Gain * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime) {
    Autopilot_DWork.IC_FirstOutputTime = false;
    rtb_IC = Autopilot_P.IC_Value;
  } else {
    rtb_IC = rtb_kntoms;
  }

  rtb_kntoms_l = Autopilot_P.kntoms_Gain_e * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_m) {
    Autopilot_DWork.IC_FirstOutputTime_m = false;
    rtb_IC_h = Autopilot_P.IC_Value_g;
  } else {
    rtb_IC_h = rtb_kntoms_l;
  }

  rtb_out_g5 = Autopilot_P.DiscreteDerivativeVariableTs_Gain_e * Autopilot_U.in.data.V_ias_kn;
  rtb_out = (rtb_out_g5 - Autopilot_DWork.Delay_DSTATE_m) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_h +
    Autopilot_U.in.data.V_ias_kn;
  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_Sum_ij = rtb_Tsxlo + Autopilot_P.Constant_Value_pg;
  Autopilot_DWork.Delay1_DSTATE_b = 1.0 / rtb_Sum_ij * (Autopilot_P.Constant_Value_pg - rtb_Tsxlo) *
    Autopilot_DWork.Delay1_DSTATE_b + (rtb_out + Autopilot_DWork.Delay_DSTATE_k) * (rtb_Tsxlo / rtb_Sum_ij);
  rtb_kntoms_b = Autopilot_P.kntoms_Gain_a * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_d) {
    Autopilot_DWork.IC_FirstOutputTime_d = false;
    rtb_IC_fz = Autopilot_P.IC_Value_a;
  } else {
    rtb_IC_fz = rtb_kntoms_b;
  }

  rtb_kntoms_e = Autopilot_P.kntoms_Gain_p * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_i) {
    Autopilot_DWork.IC_FirstOutputTime_i = false;
    rtb_IC_n = Autopilot_P.IC_Value_ay;
  } else {
    rtb_IC_n = rtb_kntoms_e;
  }

  rtb_out_a = Autopilot_P.DiscreteDerivativeVariableTs_Gain_o * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_out_g = (rtb_out_a - Autopilot_DWork.Delay_DSTATE_g) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_a +
    Autopilot_P.Gain1_Gain_k * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_Sum_ij = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_m;
  rtb_Tsxlo = rtb_Sum_ij + Autopilot_P.Constant_Value_g0;
  Autopilot_DWork.Delay1_DSTATE_p = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_g0 - rtb_Sum_ij) *
    Autopilot_DWork.Delay1_DSTATE_p + (rtb_out_g + Autopilot_DWork.Delay_DSTATE_i) * (rtb_Sum_ij / rtb_Tsxlo);
  rtb_Mod1_n = Autopilot_P.DiscreteDerivativeVariableTs_Gain_ei * Autopilot_U.in.data.V_ias_kn;
  rtb_Sum4_p = (rtb_Mod1_n - Autopilot_DWork.Delay_DSTATE_iu) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_j +
    Autopilot_U.in.data.V_ias_kn;
  rtb_Sum_ij = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1_c;
  rtb_Tsxlo = rtb_Sum_ij + Autopilot_P.Constant_Value_h;
  Autopilot_DWork.Delay1_DSTATE_a = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_h - rtb_Sum_ij) *
    Autopilot_DWork.Delay1_DSTATE_a + (rtb_Sum4_p + Autopilot_DWork.Delay_DSTATE_bp) * (rtb_Sum_ij / rtb_Tsxlo);
  rtb_Sum_ij = Autopilot_P.kntoms_Gain_d * Autopilot_U.in.data.V_tas_kn;
  rtb_Product_a = std::sin((Autopilot_P.Gain2_Gain_f * Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_f
    * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_m3) * rtb_Sum_ij;
  if (Autopilot_DWork.IC_FirstOutputTime_b) {
    Autopilot_DWork.IC_FirstOutputTime_b = false;
    rtb_Sum_ij = Autopilot_P.IC_Value_l;
  }

  rtb_Tsxlo = Autopilot_DWork.Delay1_DSTATE_a - Autopilot_U.in.data.V_c_srs_kn;
  rtb_Sum_ij = (Autopilot_P.Constant_Value_iq - Autopilot_P.msftmin_Gain_n * rtb_Product_a) *
    Autopilot_P.ftmintoms_Gain_e / rtb_Sum_ij;
  x[0] = Autopilot_P.Constant1_Value_i - Autopilot_P.Gain1_Gain_ji * Autopilot_U.in.data.Theta_deg;
  if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat_a) {
    rtb_Tsxlo = Autopilot_P.Saturation_UpperSat_a;
  } else {
    if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat_e) {
      rtb_Tsxlo = Autopilot_P.Saturation_LowerSat_e;
    }
  }

  x[1] = Autopilot_P.Gain1_Gain_ae * rtb_Tsxlo;
  if (rtb_Sum_ij > 1.0) {
    rtb_Sum_ij = 1.0;
  } else {
    if (rtb_Sum_ij < -1.0) {
      rtb_Sum_ij = -1.0;
    }
  }

  x[2] = Autopilot_P.Gain_Gain_bg * std::asin(rtb_Sum_ij) * Autopilot_P.Gain_Gain_c2;
  if (Autopilot_P.ManualSwitch_CurrentSetting_h == 1) {
    rtb_Tsxlo = Autopilot_P.Constant_Value_o;
  } else {
    rtb_Tsxlo = rtb_LAW;
  }

  switch (static_cast<int32_T>(rtb_Tsxlo)) {
   case 1:
    rtb_Tsxlo = rtb_Gain_hc * Autopilot_P.Gain_Gain_ml;
    if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat_i) {
      rtb_Tsxlo = Autopilot_P.Saturation_UpperSat_i;
    } else {
      if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat_f) {
        rtb_Tsxlo = Autopilot_P.Saturation_LowerSat_f;
      }
    }

    rtb_Tsxlo = (rtb_Tsxlo - std::sin((Autopilot_P.Gain2_Gain_kp * Autopilot_U.in.data.Theta_deg - std::cos
      (Autopilot_P.Gain1_Gain_p * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_p2) * rtb_kntoms * Autopilot_P.msftmin_Gain_m) * Autopilot_P.ftmintoms_Gain_f / rtb_IC;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_ij = Autopilot_P.Gain_Gain_oj * std::asin(rtb_Tsxlo) * Autopilot_P.Gain_Gain_oy;
    break;

   case 2:
    if (rtb_Gain_hc < 0.0) {
      rtb_Gain = -1.0;
    } else if (rtb_Gain_hc > 0.0) {
      rtb_Gain = 1.0;
    } else {
      rtb_Gain = rtb_Gain_hc;
    }

    rtb_Tsxlo = ((Autopilot_P.Constant_Value_j * rtb_Gain + rtb_Gain_hc) * Autopilot_P.Gain_Gain_m - std::sin
                 ((Autopilot_P.Gain2_Gain_k1 * Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_a *
      Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_i) * rtb_kntoms_l *
                 Autopilot_P.msftmin_Gain_a) * Autopilot_P.ftmintoms_Gain_g / rtb_IC_h;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_ij = Autopilot_P.Gain_Gain_o0 * std::asin(rtb_Tsxlo) * Autopilot_P.Gain_Gain_c;
    break;

   case 3:
    if (rtb_mode > Autopilot_P.Switch_Threshold) {
      rtb_Tsxlo = Autopilot_DWork.Delay1_DSTATE_b - Autopilot_U.in.data.ap_V_c_kn;
      if (rtb_Tsxlo > Autopilot_P.Saturation_UpperSat) {
        rtb_Tsxlo = Autopilot_P.Saturation_UpperSat;
      } else {
        if (rtb_Tsxlo < Autopilot_P.Saturation_LowerSat) {
          rtb_Tsxlo = Autopilot_P.Saturation_LowerSat;
        }
      }

      rtb_Sum_ij = Autopilot_P.Gain1_Gain_g * rtb_Tsxlo;
    } else {
      rtb_Tsxlo = (rtb_Gain - std::sin((Autopilot_P.Gain2_Gain_kd * Autopilot_U.in.data.Theta_deg - std::cos
        (Autopilot_P.Gain1_Gain_j1 * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg) *
        Autopilot_P.Gain1_Gain_n) * rtb_kntoms_b * Autopilot_P.msftmin_Gain_c) * Autopilot_P.ftmintoms_Gain_j /
        rtb_IC_fz;
      if (rtb_Tsxlo > 1.0) {
        rtb_Tsxlo = 1.0;
      } else {
        if (rtb_Tsxlo < -1.0) {
          rtb_Tsxlo = -1.0;
        }
      }

      rtb_Sum_ij = Autopilot_P.Gain_Gain_f * std::asin(rtb_Tsxlo) * Autopilot_P.Gain_Gain_o1;
    }
    break;

   case 4:
    rtb_Tsxlo = (Autopilot_U.in.data.ap_H_dot_c_ft_min - std::sin((Autopilot_P.Gain2_Gain_k *
      Autopilot_U.in.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_c * Autopilot_U.in.data.Phi_deg) *
      Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_h) * rtb_kntoms_e * Autopilot_P.msftmin_Gain) *
      Autopilot_P.ftmintoms_Gain / rtb_IC_n;
    if (rtb_Tsxlo > 1.0) {
      rtb_Tsxlo = 1.0;
    } else {
      if (rtb_Tsxlo < -1.0) {
        rtb_Tsxlo = -1.0;
      }
    }

    rtb_Sum_ij = Autopilot_P.Gain_Gain_d * std::asin(rtb_Tsxlo) * Autopilot_P.Gain_Gain_o;
    break;

   case 5:
    rtb_Sum_ij = Autopilot_P.Gain4_Gain * Autopilot_DWork.Delay1_DSTATE_p;
    break;

   case 6:
    if (x[0] < x[1]) {
      if (x[1] < x[2]) {
        rtb_LAW = 1;
      } else if (x[0] < x[2]) {
        rtb_LAW = 2;
      } else {
        rtb_LAW = 0;
      }
    } else if (x[0] < x[2]) {
      rtb_LAW = 0;
    } else if (x[1] < x[2]) {
      rtb_LAW = 2;
    } else {
      rtb_LAW = 1;
    }

    rtb_Sum_ij = x[rtb_LAW];
    break;

   default:
    rtb_Sum_ij = (Autopilot_U.in.data.ap_FPA_c_deg - (Autopilot_P.Gain2_Gain * Autopilot_U.in.data.Theta_deg - std::cos
      (Autopilot_P.Gain1_Gain_j * Autopilot_U.in.data.Phi_deg) * Autopilot_U.in.data.alpha_deg)) * Autopilot_P.Gain_Gain;
    break;
  }

  rtb_Gain_hc = Autopilot_P.Gain_Gain_nw * Autopilot_U.in.data.Theta_deg;
  rtb_Sum_ij += rtb_Gain_hc;
  if (rtb_Sum_ij > Autopilot_P.Constant1_Value) {
    rtb_Sum_ij = Autopilot_P.Constant1_Value;
  } else {
    rtb_Gain = Autopilot_P.Gain1_Gain_m * Autopilot_P.Constant1_Value;
    if (rtb_Sum_ij < rtb_Gain) {
      rtb_Sum_ij = rtb_Gain;
    }
  }

  if (rtb_out_e == 0) {
    Autopilot_DWork.icLoad_k = 1U;
  }

  if (Autopilot_DWork.icLoad_k != 0) {
    Autopilot_DWork.Delay_DSTATE_a = rtb_Gain_hc;
  }

  rtb_Tsxlo = rtb_Sum_ij - Autopilot_DWork.Delay_DSTATE_a;
  rtb_kntoms_l = Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms_l) {
    rtb_kntoms_l = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_P.Gain1_Gain_fb * Autopilot_P.Constant2_Value_e * Autopilot_U.in.time.dt;
  if (rtb_kntoms_l > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms_l;
  }

  Autopilot_DWork.Delay_DSTATE_a += rtb_Tsxlo;
  rtb_Gain = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_e;
  rtb_Tsxlo = rtb_Gain + Autopilot_P.Constant_Value_eb;
  Autopilot_DWork.Delay1_DSTATE_l = 1.0 / rtb_Tsxlo * (Autopilot_P.Constant_Value_eb - rtb_Gain) *
    Autopilot_DWork.Delay1_DSTATE_l + (Autopilot_DWork.Delay_DSTATE_a + Autopilot_DWork.Delay_DSTATE_l) * (rtb_Gain /
    rtb_Tsxlo);
  rtb_Tsxlo = static_cast<real_T>(rtb_out_e) - Autopilot_DWork.Delay_DSTATE_c;
  rtb_kntoms_l = Autopilot_P.RateLimiterVariableTs_up_k * Autopilot_U.in.time.dt;
  if (rtb_Tsxlo < rtb_kntoms_l) {
    rtb_kntoms_l = rtb_Tsxlo;
  }

  rtb_Tsxlo = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo_h;
  if (rtb_kntoms_l > rtb_Tsxlo) {
    rtb_Tsxlo = rtb_kntoms_l;
  }

  Autopilot_DWork.Delay_DSTATE_c += rtb_Tsxlo;
  if (Autopilot_DWork.Delay_DSTATE_c > Autopilot_P.Saturation_UpperSat_b) {
    rtb_Gain = Autopilot_P.Saturation_UpperSat_b;
  } else if (Autopilot_DWork.Delay_DSTATE_c < Autopilot_P.Saturation_LowerSat_k) {
    rtb_Gain = Autopilot_P.Saturation_LowerSat_k;
  } else {
    rtb_Gain = Autopilot_DWork.Delay_DSTATE_c;
  }

  Autopilot_Y.out.output.autopilot.Theta_c_deg = (Autopilot_P.Constant_Value_o1 - rtb_Gain) * rtb_Gain_hc +
    Autopilot_DWork.Delay1_DSTATE_l * rtb_Gain;
  Autopilot_Y.out.time = Autopilot_U.in.time;
  Autopilot_Y.out.input = Autopilot_U.in.input;
  Autopilot_Y.out.data = Autopilot_U.in.data;
  Autopilot_Y.out.output.ap_on = rtb_out_e;
  Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_ij;
  Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_out_i;
  Autopilot_DWork.Delay_DSTATE = rtb_Sum;
  Autopilot_DWork.Delay_DSTATE_j = rtb_Sum3;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_DWork.Delay_DSTATE_jv;
  Autopilot_DWork.Delay_DSTATE_m = rtb_out_g5;
  Autopilot_DWork.Delay_DSTATE_k = rtb_out;
  Autopilot_DWork.Delay_DSTATE_g = rtb_out_a;
  Autopilot_DWork.Delay_DSTATE_i = rtb_out_g;
  Autopilot_DWork.Delay_DSTATE_iu = rtb_Mod1_n;
  Autopilot_DWork.Delay_DSTATE_bp = rtb_Sum4_p;
  Autopilot_DWork.icLoad_k = 0U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_DWork.Delay_DSTATE_a;
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
  Autopilot_DWork.IC_FirstOutputTime_m = true;
  Autopilot_DWork.IC_FirstOutputTime_d = true;
  Autopilot_DWork.IC_FirstOutputTime_i = true;
  Autopilot_DWork.IC_FirstOutputTime_b = true;
  Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_j = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_P.Delay_InitialCondition_o;
  Autopilot_DWork.Delay1_DSTATE_m = Autopilot_P.Delay1_InitialCondition_c;
  Autopilot_DWork.Delay_DSTATE_jv5 = Autopilot_P.RateLimiterVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_m = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_m;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.Delay_InitialCondition_l;
  Autopilot_DWork.Delay1_DSTATE_b = Autopilot_P.Delay1_InitialCondition_h;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_p;
  Autopilot_DWork.Delay_DSTATE_i = Autopilot_P.Delay_InitialCondition_d;
  Autopilot_DWork.Delay1_DSTATE_p = Autopilot_P.Delay1_InitialCondition_p;
  Autopilot_DWork.Delay_DSTATE_iu = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_n;
  Autopilot_DWork.Delay_DSTATE_bp = Autopilot_P.Delay_InitialCondition_i;
  Autopilot_DWork.Delay1_DSTATE_a = Autopilot_P.Delay1_InitialCondition_b;
  Autopilot_DWork.icLoad_k = 1U;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_P.Delay_InitialCondition_b;
  Autopilot_DWork.Delay1_DSTATE_l = Autopilot_P.Delay1_InitialCondition_l;
  Autopilot_DWork.Delay_DSTATE_c = Autopilot_P.RateLimiterVariableTs_InitialCondition_i;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c1_Autopilot = 0U;
  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c4_Autopilot = 0U;
  Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_DWork.is_active_c2_Autopilot = 0U;
  Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_a;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_c);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_f);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_e);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_i);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_g);
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
