#include "Autopilot.h"
#include "Autopilot_private.h"

const uint8_T Autopilot_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T Autopilot_IN_any = 1U;
const uint8_T Autopilot_IN_left = 2U;
const uint8_T Autopilot_IN_right = 3U;
const uint8_T Autopilot_IN_COND = 1U;
const uint8_T Autopilot_IN_LOC = 1U;
const uint8_T Autopilot_IN_LOC_CPT = 2U;
const uint8_T Autopilot_IN_LOC_TRK = 3U;
const uint8_T Autopilot_IN_MANAGED = 2U;
const uint8_T Autopilot_IN_NO_ACTIVE_CHILD_j = 0U;
const uint8_T Autopilot_IN_ROLLOUT = 4U;
const uint8_T Autopilot_IN_SELECTED = 3U;
const uint8_T Autopilot_IN_InAir = 1U;
const uint8_T Autopilot_IN_OnGround = 2U;
const uint8_T Autopilot_IN_OFF = 1U;
const uint8_T Autopilot_IN_ON = 2U;
const uint8_T Autopilot_IN_ALT_ACQ = 1U;
const uint8_T Autopilot_IN_ALT_HOLD = 2U;
const uint8_T Autopilot_IN_FLARE = 3U;
const uint8_T Autopilot_IN_GS = 4U;
const uint8_T Autopilot_IN_SPD_MACH = 5U;
const uint8_T Autopilot_IN_VS = 6U;
const ap_output Autopilot_rtZap_output = {
  {
    0.0,
    0.0
  },

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
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    }
  }
} ;

const ap_input Autopilot_rtZap_input = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

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
  localDW->is_active_c1_Autopilot = 0U;
  localDW->is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD;
}

void AutopilotModelClass::Autopilot_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T *rty_out,
  rtDW_Chart_Autopilot_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  if (localDW->is_active_c1_Autopilot == 0U) {
    localDW->is_active_c1_Autopilot = 1U;
    localDW->is_c1_Autopilot = Autopilot_IN_any;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c1_Autopilot) {
     case Autopilot_IN_any:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c1_Autopilot = Autopilot_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
        localDW->is_c1_Autopilot = Autopilot_IN_left;
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
        localDW->is_c1_Autopilot = Autopilot_IN_any;
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
        localDW->is_c1_Autopilot = Autopilot_IN_any;
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
  real_T result[3];
  real_T x[3];
  real_T rtb_Minup;
  real_T rtb_Sum_j;
  real_T rtb_Product1_o;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  int32_T rtb_out;
  real_T rtb_Switch;
  real_T rtb_Mod1;
  real_T rtb_Mod2;
  real_T rtb_ROLLLIM1;
  real_T rtb_kntoms;
  real_T rtb_IC;
  real_T rtb_kntoms_o;
  real_T rtb_IC_c;
  real_T rtb_kntoms_k;
  real_T rtb_IC_n;
  real_T rtb_kntoms_i;
  real_T rtb_IC_k;
  real_T rtb_Sum4_f;
  real_T rtb_Switch_mx;
  real_T rtb_Sum_p;
  real_T rtb_out_lg;
  real_T rtb_out_ii;
  real_T rtb_out_f;
  real_T rtb_out_p;
  real_T rtb_out_aa;
  int32_T rtb_LAW;
  real_T rtb_Sum1_l5;
  real_T tmp[9];
  rtb_GainTheta = Autopilot_P.GainTheta_Gain * Autopilot_U.in.data.Theta_deg;
  rtb_GainTheta1 = Autopilot_P.GainTheta1_Gain * Autopilot_U.in.data.Phi_deg;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta;
  rtb_Mod2 = 0.017453292519943295 * rtb_GainTheta1;
  rtb_Mod1 = std::tan(rtb_Saturation1);
  rtb_Saturation = std::sin(rtb_Mod2);
  rtb_Mod2 = std::cos(rtb_Mod2);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation * rtb_Mod1;
  tmp[6] = rtb_Mod2 * rtb_Mod1;
  tmp[1] = 0.0;
  tmp[4] = rtb_Mod2;
  tmp[7] = -rtb_Saturation;
  tmp[2] = 0.0;
  rtb_Minup = 1.0 / std::cos(rtb_Saturation1);
  tmp[5] = rtb_Minup * rtb_Saturation;
  tmp[8] = rtb_Minup * rtb_Mod2;
  rtb_Saturation = Autopilot_P.Gain_Gain_k * Autopilot_U.in.data.p_rad_s * Autopilot_P.Gainpk_Gain;
  rtb_Saturation1 = Autopilot_P.Gain_Gain_l * Autopilot_U.in.data.q_rad_s * Autopilot_P.Gainqk_Gain;
  rtb_Mod1 = Autopilot_P.Gain_Gain_aq * Autopilot_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_Mod1 + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
      tmp[rtb_on_ground] * rtb_Saturation);
  }

  rtb_Saturation = Autopilot_P.Gain_Gain_af * Autopilot_U.in.data.gear_strut_compression_1 -
    Autopilot_P.Constant1_Value_p;
  if (rtb_Saturation > Autopilot_P.Saturation_UpperSat_k) {
    rtb_Saturation = Autopilot_P.Saturation_UpperSat_k;
  } else {
    if (rtb_Saturation < Autopilot_P.Saturation_LowerSat_m) {
      rtb_Saturation = Autopilot_P.Saturation_LowerSat_m;
    }
  }

  rtb_Saturation1 = Autopilot_P.Gain1_Gain_k1 * Autopilot_U.in.data.gear_strut_compression_2 -
    Autopilot_P.Constant1_Value_p;
  if (rtb_Saturation1 > Autopilot_P.Saturation1_UpperSat) {
    rtb_Saturation1 = Autopilot_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < Autopilot_P.Saturation1_LowerSat) {
      rtb_Saturation1 = Autopilot_P.Saturation1_LowerSat;
    }
  }

  if (Autopilot_DWork.is_active_c5_Autopilot == 0U) {
    Autopilot_DWork.is_active_c5_Autopilot = 1U;
    Autopilot_DWork.is_c5_Autopilot = Autopilot_IN_OnGround;
    rtb_on_ground = 1;
  } else if (Autopilot_DWork.is_c5_Autopilot == Autopilot_IN_InAir) {
    if ((rtb_Saturation > 0.1) || (rtb_Saturation1 > 0.1)) {
      Autopilot_DWork.is_c5_Autopilot = Autopilot_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      Autopilot_DWork.is_c5_Autopilot = Autopilot_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  if (Autopilot_DWork.is_active_c3_Autopilot == 0U) {
    Autopilot_DWork.is_active_c3_Autopilot = 1U;
    Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
    rtb_out = 0;
  } else if (Autopilot_DWork.is_c3_Autopilot == Autopilot_IN_OFF) {
    if (Autopilot_U.in.input.trigger_ap_master == 1.0) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_ON;
      rtb_out = 1;
    } else {
      rtb_out = 0;
    }
  } else {
    if ((Autopilot_U.in.input.trigger_ap_master == 1.0) || (Autopilot_U.in.input.trigger_ap_off == 1.0)) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
      rtb_out = 0;
    } else {
      rtb_out = 1;
    }
  }

  rtb_Saturation = Autopilot_P.Gain2_Gain * Autopilot_U.in.data.zeta_pos;
  if (Autopilot_DWork.is_active_c6_Autopilot == 0U) {
    Autopilot_DWork.is_active_c6_Autopilot = 1U;
    Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_SELECTED;
    Autopilot_B.LAW = 1.0;
  } else {
    switch (Autopilot_DWork.is_c6_Autopilot) {
     case Autopilot_IN_LOC:
      if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
        Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_j;
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_SELECTED;
        Autopilot_B.LAW = 1.0;
      } else if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
        Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_j;
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_MANAGED;
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

         case Autopilot_IN_LOC_TRK:
          if (rtb_on_ground != 0) {
            Autopilot_DWork.is_LOC = Autopilot_IN_ROLLOUT;
            Autopilot_B.LAW = 6.0;
          } else {
            Autopilot_B.LAW = 5.0;
          }
          break;

         default:
          Autopilot_B.LAW = 6.0;
          break;
        }
      }
      break;

     case Autopilot_IN_MANAGED:
      if (Autopilot_U.in.input.trigger_loc == 1.0) {
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_LOC;
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
        Autopilot_B.LAW = 4.0;
      } else if (Autopilot_U.in.input.trigger_hdg_mode == 1.0) {
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_SELECTED;
        Autopilot_B.LAW = 1.0;
      } else {
        Autopilot_B.LAW = 3.0;
      }
      break;

     default:
      if (Autopilot_U.in.input.trigger_hdg_mode == 2.0) {
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_MANAGED;
        Autopilot_B.LAW = 3.0;
      } else if (Autopilot_U.in.input.trigger_loc == 1.0) {
        Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_LOC;
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
        Autopilot_B.LAW = 4.0;
      } else {
        Autopilot_B.LAW = 1.0;
      }
      break;
    }
  }

  if (Autopilot_P.ManualSwitch_CurrentSetting == 1) {
    rtb_Minup = Autopilot_P.Constant_Value;
  } else {
    rtb_Minup = Autopilot_U.in.mode.lateral_mode;
  }

  if (rtb_Minup > Autopilot_P.Switch_Threshold_o) {
    rtb_Switch = rtb_Minup;
  } else {
    rtb_Switch = Autopilot_B.LAW;
  }

  if (rtb_Switch != Autopilot_P.CompareToConstant_const) {
    Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  if (Autopilot_U.in.data.nav_dme_nmi > Autopilot_P.Saturation_UpperSat_o) {
    rtb_Minup = Autopilot_P.Saturation_UpperSat_o;
  } else if (Autopilot_U.in.data.nav_dme_nmi < Autopilot_P.Saturation_LowerSat_o) {
    rtb_Minup = Autopilot_P.Saturation_LowerSat_o;
  } else {
    rtb_Minup = Autopilot_U.in.data.nav_dme_nmi;
  }

  rtb_Minup = std::sin(Autopilot_P.Gain1_Gain_b * Autopilot_U.in.data.nav_radial_error_deg) * rtb_Minup *
    Autopilot_P.Gain2_Gain_g;
  if (rtb_Minup > Autopilot_P.Saturation1_UpperSat_g) {
    rtb_Minup = Autopilot_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_Minup < Autopilot_P.Saturation1_LowerSat_k) {
      rtb_Minup = Autopilot_P.Saturation1_LowerSat_k;
    }
  }

  Autopilot_DWork.Delay_DSTATE += Autopilot_P.Gain6_Gain * rtb_Minup * Autopilot_P.DiscreteTimeIntegratorVariableTs_Gain
    * Autopilot_U.in.time.dt;
  if (Autopilot_DWork.Delay_DSTATE > Autopilot_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (Autopilot_DWork.Delay_DSTATE < Autopilot_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  rtb_Saturation1 = Autopilot_U.in.data.nav_radial_error_deg + Autopilot_U.in.data.nav_loc_deg;
  rtb_Mod1 = rt_modd((Autopilot_U.in.data.Psi_magnetic_deg - (rt_modd(rt_modd(rtb_Saturation1,
    Autopilot_P.Constant3_Value_b) + Autopilot_P.Constant3_Value_b, Autopilot_P.Constant3_Value_b) +
    Autopilot_P.Constant3_Value_i)) + Autopilot_P.Constant3_Value_i, Autopilot_P.Constant3_Value_i);
  rtb_Mod2 = rt_modd(Autopilot_P.Constant3_Value_i - rtb_Mod1, Autopilot_P.Constant3_Value_i);
  if (rtb_Mod1 < rtb_Mod2) {
    rtb_Mod1 *= Autopilot_P.Gain1_Gain_o;
  } else {
    rtb_Mod1 = Autopilot_P.Gain_Gain_a1 * rtb_Mod2;
  }

  rtb_Mod1 = rt_modd((rt_modd(rt_modd(((rtb_Minup * look1_binlxpw(Autopilot_U.in.data.V_gnd_kn,
    Autopilot_P.ScheduledGain_BreakpointsForDimension1, Autopilot_P.ScheduledGain_Table, 2U) +
    Autopilot_DWork.Delay_DSTATE) + Autopilot_P.Gain1_Gain_f * rtb_Mod1) + Autopilot_U.in.data.Psi_magnetic_deg,
    Autopilot_P.Constant3_Value_c) + Autopilot_P.Constant3_Value_c, Autopilot_P.Constant3_Value_c) -
                      (Autopilot_U.in.data.Psi_magnetic_deg + Autopilot_P.Constant3_Value_g)) +
                     Autopilot_P.Constant3_Value_g, Autopilot_P.Constant3_Value_g);
  rtb_Minup = rt_modd((Autopilot_U.in.data.nav_loc_deg - (Autopilot_U.in.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value_o)) + Autopilot_P.Constant3_Value_o, Autopilot_P.Constant3_Value_o);
  Autopilot_Chart(rtb_Minup, Autopilot_P.Gain_Gain_cw * rt_modd(Autopilot_P.Constant3_Value_o - rtb_Minup,
    Autopilot_P.Constant3_Value_o), Autopilot_P.Constant2_Value_l, &rtb_Mod2, &Autopilot_DWork.sf_Chart_p);
  Autopilot_Chart(rtb_Mod1, Autopilot_P.Gain_Gain_mt * rt_modd(Autopilot_P.Constant3_Value_g - rtb_Mod1,
    Autopilot_P.Constant3_Value_g), Autopilot_P.Constant1_Value_e, &rtb_Minup, &Autopilot_DWork.sf_Chart_n);
  switch (static_cast<int32_T>(rtb_Switch)) {
   case 1:
    rtb_Mod2 = Autopilot_P.beta_Value_e;
    break;

   case 2:
    rtb_Mod2 = Autopilot_P.beta_Value_b;
    break;

   case 3:
    rtb_Mod2 = Autopilot_P.beta_Value_m;
    break;

   case 4:
    rtb_Mod2 = Autopilot_P.beta_Value;
    break;

   case 5:
    if (Autopilot_U.in.data.H_radio_ft <= Autopilot_P.CompareToConstant_const_d) {
      rtb_Mod2 *= Autopilot_P.Gain_Gain_a;
    } else {
      rtb_Mod2 = Autopilot_P.Constant1_Value;
    }
    break;

   default:
    rtb_Mod2 = (Autopilot_P.Gain5_Gain * rtb_Minup + Autopilot_P.Gain_Gain_b * result[2]) + rtb_Saturation;
    break;
  }

  rtb_ROLLLIM1 = look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data,
    Autopilot_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1 = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value_nu)) + Autopilot_P.Constant3_Value_nu, Autopilot_P.Constant3_Value_nu);
  Autopilot_Chart(rtb_Mod1, Autopilot_P.Gain_Gain_p * rt_modd(Autopilot_P.Constant3_Value_nu - rtb_Mod1,
    Autopilot_P.Constant3_Value_nu), Autopilot_P.Constant_Value_c, &rtb_out_lg, &Autopilot_DWork.sf_Chart_f);
  rtb_Mod1 = rt_modd((Autopilot_U.in.data.ap_Psi_c_deg - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_gl)) + Autopilot_P.Constant3_Value_gl, Autopilot_P.Constant3_Value_gl);
  Autopilot_Chart(rtb_Mod1, Autopilot_P.Gain_Gain_ee * rt_modd(Autopilot_P.Constant3_Value_gl - rtb_Mod1,
    Autopilot_P.Constant3_Value_gl), Autopilot_P.Constant_Value_i, &rtb_out_ii, &Autopilot_DWork.sf_Chart_nr);
  rtb_Minup = Autopilot_P.Gain_Gain_nu * Autopilot_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_Minup > Autopilot_P.Saturation_UpperSat_kr) {
    rtb_Minup = Autopilot_P.Saturation_UpperSat_kr;
  } else {
    if (rtb_Minup < Autopilot_P.Saturation_LowerSat_p) {
      rtb_Minup = Autopilot_P.Saturation_LowerSat_p;
    }
  }

  rtb_Mod1 = rt_modd((rt_modd(rt_modd((Autopilot_P.Gain2_Gain_f * Autopilot_U.in.data.flight_guidance_tae_deg +
    rtb_Minup) * Autopilot_P.Gain1_Gain_n + Autopilot_U.in.data.Psi_magnetic_track_deg, Autopilot_P.Constant3_Value_cm)
    + Autopilot_P.Constant3_Value_cm, Autopilot_P.Constant3_Value_cm) - (Autopilot_U.in.data.Psi_magnetic_track_deg +
    Autopilot_P.Constant3_Value_ck)) + Autopilot_P.Constant3_Value_ck, Autopilot_P.Constant3_Value_ck);
  Autopilot_Chart(rtb_Mod1, Autopilot_P.Gain_Gain_lb * rt_modd(Autopilot_P.Constant3_Value_ck - rtb_Mod1,
    Autopilot_P.Constant3_Value_ck), Autopilot_P.Constant_Value_cw, &rtb_out_f, &Autopilot_DWork.sf_Chart_i);
  rtb_Saturation1 = rt_modd((Autopilot_U.in.data.Psi_magnetic_track_deg - (rt_modd(rt_modd(rtb_Saturation1,
    Autopilot_P.Constant3_Value_be) + Autopilot_P.Constant3_Value_be, Autopilot_P.Constant3_Value_be) +
    Autopilot_P.Constant3_Value_h)) + Autopilot_P.Constant3_Value_h, Autopilot_P.Constant3_Value_h);
  rtb_Mod1 = rt_modd(Autopilot_P.Constant3_Value_h - rtb_Saturation1, Autopilot_P.Constant3_Value_h);
  if (Autopilot_U.in.data.nav_dme_nmi > Autopilot_P.Saturation_UpperSat_m) {
    rtb_Minup = Autopilot_P.Saturation_UpperSat_m;
  } else if (Autopilot_U.in.data.nav_dme_nmi < Autopilot_P.Saturation_LowerSat_k) {
    rtb_Minup = Autopilot_P.Saturation_LowerSat_k;
  } else {
    rtb_Minup = Autopilot_U.in.data.nav_dme_nmi;
  }

  rtb_Minup = std::sin(Autopilot_P.Gain1_Gain_m * Autopilot_U.in.data.nav_radial_error_deg) * rtb_Minup * look1_binlxpw
    (Autopilot_U.in.data.nav_dme_nmi, Autopilot_P.ScheduledGain_BreakpointsForDimension1_a,
     Autopilot_P.ScheduledGain_Table_p, 4U);
  if (rtb_Minup > Autopilot_P.Saturation1_UpperSat_i) {
    rtb_Minup = Autopilot_P.Saturation1_UpperSat_i;
  } else {
    if (rtb_Minup < Autopilot_P.Saturation1_LowerSat_g) {
      rtb_Minup = Autopilot_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Saturation1 < rtb_Mod1) {
    rtb_Saturation1 *= Autopilot_P.Gain1_Gain;
  } else {
    rtb_Saturation1 = Autopilot_P.Gain_Gain * rtb_Mod1;
  }

  rtb_Saturation1 = rt_modd((rt_modd(rt_modd((rtb_Minup + rtb_Saturation1) * Autopilot_P.Gain3_Gain +
    Autopilot_U.in.data.Psi_magnetic_track_deg, Autopilot_P.Constant3_Value_bd) + Autopilot_P.Constant3_Value_bd,
    Autopilot_P.Constant3_Value_bd) - (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_d)) +
    Autopilot_P.Constant3_Value_d, Autopilot_P.Constant3_Value_d);
  Autopilot_Chart(rtb_Saturation1, Autopilot_P.Gain_Gain_av * rt_modd(Autopilot_P.Constant3_Value_d - rtb_Saturation1,
    Autopilot_P.Constant3_Value_d), Autopilot_P.Constant_Value_e, &rtb_out_p, &Autopilot_DWork.sf_Chart_g);
  rtb_Saturation1 = Autopilot_P.DiscreteDerivativeVariableTs_Gain * Autopilot_U.in.data.nav_radial_error_deg;
  rtb_Mod1 = (rtb_Saturation1 - Autopilot_DWork.Delay_DSTATE_i) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_i +
    Autopilot_U.in.data.nav_radial_error_deg;
  rtb_out_aa = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1;
  rtb_Minup = rtb_out_aa + Autopilot_P.Constant_Value_h;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_Minup * (Autopilot_P.Constant_Value_h - rtb_out_aa) *
    Autopilot_DWork.Delay1_DSTATE + (rtb_Mod1 + Autopilot_DWork.Delay_DSTATE_k) * (rtb_out_aa / rtb_Minup);
  rtb_Minup = rt_modd((rt_modd(rt_modd(Autopilot_DWork.Delay1_DSTATE * look1_binlxpw(Autopilot_U.in.data.H_radio_ft,
    Autopilot_P.ScheduledGain_BreakpointsForDimension1_e, Autopilot_P.ScheduledGain_Table_pf, 4U) +
    Autopilot_U.in.data.Psi_magnetic_track_deg, Autopilot_P.Constant3_Value_hd) + Autopilot_P.Constant3_Value_hd,
    Autopilot_P.Constant3_Value_hd) - (Autopilot_U.in.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_oy)) +
                      Autopilot_P.Constant3_Value_oy, Autopilot_P.Constant3_Value_oy);
  Autopilot_Chart(rtb_Minup, Autopilot_P.Gain_Gain_nd * rt_modd(Autopilot_P.Constant3_Value_oy - rtb_Minup,
    Autopilot_P.Constant3_Value_oy), Autopilot_P.Constant_Value_p1, &rtb_out_aa, &Autopilot_DWork.sf_Chart_gi);
  switch (static_cast<int32_T>(rtb_Switch)) {
   case 1:
    rtb_out_aa = rtb_out_lg * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_j, Autopilot_P.ScheduledGain_Table_k, 3U);
    break;

   case 2:
    rtb_out_aa = rtb_out_ii * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_o, Autopilot_P.ScheduledGain_Table_h, 3U);
    break;

   case 3:
    rtb_out_aa = rtb_out_f * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_j5, Autopilot_P.ScheduledGain_Table_l, 3U);
    break;

   case 4:
    rtb_out_aa = rtb_out_p * look1_binlxpw(Autopilot_U.in.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_k, Autopilot_P.ScheduledGain_Table_g, 3U);
    break;

   case 5:
    rtb_out_aa *= look1_binlxpw(Autopilot_U.in.data.V_tas_kn, Autopilot_P.ScheduledGain_BreakpointsForDimension1_m,
      Autopilot_P.ScheduledGain_Table_hb, 3U);
    break;

   default:
    rtb_out_aa = Autopilot_P.Constant3_Value;
    break;
  }

  if (rtb_out_aa > rtb_ROLLLIM1) {
    rtb_out_aa = rtb_ROLLLIM1;
  } else {
    rtb_Minup = Autopilot_P.Gain1_Gain_l * rtb_ROLLLIM1;
    if (rtb_out_aa < rtb_Minup) {
      rtb_out_aa = rtb_Minup;
    }
  }

  rtb_Switch = Autopilot_P.Gain_Gain_d * rtb_GainTheta1;
  if (rtb_out == 0) {
    Autopilot_DWork.icLoad = 1U;
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE_h = rtb_Switch;
  }

  rtb_Minup = rtb_out_aa - Autopilot_DWork.Delay_DSTATE_h;
  rtb_kntoms_o = Autopilot_P.Constant2_Value_h * Autopilot_U.in.time.dt;
  if (rtb_Minup < rtb_kntoms_o) {
    rtb_kntoms_o = rtb_Minup;
  }

  rtb_Minup = Autopilot_P.Gain1_Gain_kf * Autopilot_P.Constant2_Value_h * Autopilot_U.in.time.dt;
  if (rtb_kntoms_o > rtb_Minup) {
    rtb_Minup = rtb_kntoms_o;
  }

  Autopilot_DWork.Delay_DSTATE_h += rtb_Minup;
  rtb_Sum_j = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_l;
  rtb_Minup = rtb_Sum_j + Autopilot_P.Constant_Value_cq;
  Autopilot_DWork.Delay1_DSTATE_b = 1.0 / rtb_Minup * (Autopilot_P.Constant_Value_cq - rtb_Sum_j) *
    Autopilot_DWork.Delay1_DSTATE_b + (Autopilot_DWork.Delay_DSTATE_h + Autopilot_DWork.Delay_DSTATE_g) * (rtb_Sum_j /
    rtb_Minup);
  rtb_Minup = static_cast<real_T>(rtb_out) - Autopilot_DWork.Delay_DSTATE_c;
  rtb_kntoms_o = Autopilot_P.RateLimiterVariableTs_up * Autopilot_U.in.time.dt;
  if (rtb_Minup < rtb_kntoms_o) {
    rtb_kntoms_o = rtb_Minup;
  }

  rtb_Minup = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo;
  if (rtb_kntoms_o > rtb_Minup) {
    rtb_Minup = rtb_kntoms_o;
  }

  Autopilot_DWork.Delay_DSTATE_c += rtb_Minup;
  if (Autopilot_DWork.Delay_DSTATE_c > Autopilot_P.Saturation_UpperSat_d) {
    rtb_Sum_p = Autopilot_P.Saturation_UpperSat_d;
  } else if (Autopilot_DWork.Delay_DSTATE_c < Autopilot_P.Saturation_LowerSat_a) {
    rtb_Sum_p = Autopilot_P.Saturation_LowerSat_a;
  } else {
    rtb_Sum_p = Autopilot_DWork.Delay_DSTATE_c;
  }

  rtb_out_lg = Autopilot_DWork.Delay1_DSTATE_b * rtb_Sum_p;
  rtb_Product1_o = (Autopilot_P.Constant_Value_l - rtb_Sum_p) * rtb_Switch;
  Autopilot_Y.out.output.flight_director.Beta_c_deg = rtb_Mod2;
  Autopilot_Y.out.output.autopilot.Beta_c_deg = rtb_Mod2;
  Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_out_aa;
  if (Autopilot_DWork.is_active_c10_Autopilot == 0U) {
    Autopilot_DWork.is_active_c10_Autopilot = 1U;
    Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_HOLD;
    rtb_LAW = 1;
  } else {
    switch (Autopilot_DWork.is_c10_Autopilot) {
     case Autopilot_IN_ALT_ACQ:
      rtb_Minup = std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft);
      if (rtb_Minup <= 20.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_LAW = 1;
      } else if (rtb_Minup > 1000.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_LAW = 3;
      } else {
        rtb_LAW = 2;
      }
      break;

     case Autopilot_IN_ALT_HOLD:
      if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_GS;
        rtb_LAW = 6;
      } else if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) > 250.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_LAW = 3;
      } else {
        rtb_LAW = 1;
      }
      break;

     case Autopilot_IN_FLARE:
      if (Autopilot_U.in.data.H_radio_ft > 50.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_GS;
        rtb_LAW = 6;
      } else if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_LAW = 1;
      } else {
        rtb_LAW = 7;
      }
      break;

     case Autopilot_IN_GS:
      if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_HOLD;
        rtb_LAW = 1;
      } else if (Autopilot_U.in.data.H_radio_ft <= 50.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_FLARE;
        rtb_LAW = 7;
      } else {
        rtb_LAW = 6;
      }
      break;

     case Autopilot_IN_SPD_MACH:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_vs_mode == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_VS;
        rtb_LAW = 4;
      } else if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_GS;
        rtb_LAW = 6;
      } else {
        rtb_LAW = 3;
      }
      break;

     default:
      if (std::abs(Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) <= std::abs
          (Autopilot_U.in.data.H_dot_ft_min) / 8.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_ALT_ACQ;
        rtb_LAW = 2;
      } else if (Autopilot_U.in.input.trigger_alt_mode != 0.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_SPD_MACH;
        rtb_LAW = 3;
      } else if (Autopilot_U.in.input.trigger_appr == 1.0) {
        Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_GS;
        rtb_LAW = 6;
      } else {
        rtb_LAW = 4;
      }
      break;
    }
  }

  if (Autopilot_P.ManualSwitch_CurrentSetting_b == 1) {
    rtb_Sum_p = Autopilot_P.Constant_Value_m;
  } else {
    rtb_Sum_p = Autopilot_U.in.mode.vertical_mode;
  }

  if (rtb_Sum_p > Autopilot_P.Switch_Threshold_d) {
    rtb_Switch_mx = rtb_Sum_p;
  } else {
    rtb_Switch_mx = rtb_LAW;
  }

  rtb_kntoms = Autopilot_P.kntoms_Gain * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime) {
    Autopilot_DWork.IC_FirstOutputTime = false;
    rtb_IC = Autopilot_P.IC_Value;
  } else {
    rtb_IC = rtb_kntoms;
  }

  rtb_kntoms_o = Autopilot_P.kntoms_Gain_e * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_n) {
    Autopilot_DWork.IC_FirstOutputTime_n = false;
    rtb_IC_c = Autopilot_P.IC_Value_m;
  } else {
    rtb_IC_c = rtb_kntoms_o;
  }

  rtb_Switch = Autopilot_P.DiscreteDerivativeVariableTs_Gain_m * Autopilot_U.in.data.V_ias_kn;
  rtb_ROLLLIM1 = (rtb_Switch - Autopilot_DWork.Delay_DSTATE_ky) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_f +
    Autopilot_U.in.data.V_ias_kn;
  rtb_Sum_p = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_Sum_j = rtb_Sum_p + Autopilot_P.Constant_Value_lq;
  Autopilot_DWork.Delay1_DSTATE_k = 1.0 / rtb_Sum_j * (Autopilot_P.Constant_Value_lq - rtb_Sum_p) *
    Autopilot_DWork.Delay1_DSTATE_k + (rtb_ROLLLIM1 + Autopilot_DWork.Delay_DSTATE_o) * (rtb_Sum_p / rtb_Sum_j);
  rtb_Minup = Autopilot_DWork.Delay1_DSTATE_k - Autopilot_U.in.data.ap_V_c_kn;
  if (rtb_Minup > Autopilot_P.Saturation_UpperSat_h) {
    rtb_Minup = Autopilot_P.Saturation_UpperSat_h;
  } else {
    if (rtb_Minup < Autopilot_P.Saturation_LowerSat_c) {
      rtb_Minup = Autopilot_P.Saturation_LowerSat_c;
    }
  }

  rtb_Sum_p = Autopilot_P.Gain1_Gain_bz * rtb_Minup;
  rtb_out_aa = Autopilot_P.Gain1_Gain_a * Autopilot_U.in.data.alpha_deg;
  rtb_Mod2 = Autopilot_U.in.data.bz_m_s2 * std::sin(rtb_out_aa);
  rtb_out_aa = std::cos(rtb_out_aa);
  rtb_out_aa *= Autopilot_U.in.data.bx_m_s2;
  rtb_out_aa = (rtb_Mod2 + rtb_out_aa) * Autopilot_P.Gain_Gain_fr * Autopilot_P.Gain_Gain_dg;
  rtb_Minup = Autopilot_P.Gain1_Gain_k0 * std::abs(Autopilot_P.Constant_Value_em) + std::abs(rtb_out_aa);
  if (rtb_Minup <= Autopilot_P.Constant1_Value_ef) {
    rtb_Minup = Autopilot_P.Constant1_Value_ef;
  }

  rtb_Mod2 = std::abs(rtb_Sum_p);
  if (rtb_Mod2 < rtb_Minup) {
    rtb_Minup = rtb_Mod2;
  }

  if (rtb_out_aa < 0.0) {
    rtb_out_aa = -1.0;
  } else {
    if (rtb_out_aa > 0.0) {
      rtb_out_aa = 1.0;
    }
  }

  rtb_Sum1_l5 = rtb_Sum_p - rtb_Minup * rtb_out_aa * Autopilot_P.Gain_Gain_h;
  rtb_Sum_p = Autopilot_P.kntoms_Gain_i * Autopilot_U.in.data.V_tas_kn;
  rtb_Mod2 = std::sin((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_p * rtb_GainTheta1) *
                       Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_kh) * rtb_Sum_p *
    Autopilot_P.msftmin_Gain_j;
  if (Autopilot_DWork.IC_FirstOutputTime_i) {
    Autopilot_DWork.IC_FirstOutputTime_i = false;
    rtb_Sum_p = Autopilot_P.IC_Value_l;
  }

  rtb_Minup = (Autopilot_P.Constant_Value_k - rtb_Mod2) * Autopilot_P.ftmintoms_Gain_f / rtb_Sum_p;
  rtb_kntoms_k = Autopilot_P.kntoms_Gain_a * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_d) {
    Autopilot_DWork.IC_FirstOutputTime_d = false;
    rtb_IC_n = Autopilot_P.IC_Value_k;
  } else {
    rtb_IC_n = rtb_kntoms_k;
  }

  rtb_out_ii = Autopilot_P.DiscreteDerivativeVariableTs_Gain_l * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_out_f = (rtb_out_ii - Autopilot_DWork.Delay_DSTATE_e) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_o +
    Autopilot_P.Gain1_Gain_j * Autopilot_U.in.data.nav_gs_error_deg;
  rtb_Sum_p = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_n;
  rtb_Sum_j = rtb_Sum_p + Autopilot_P.Constant_Value_ik;
  Autopilot_DWork.Delay1_DSTATE_l = 1.0 / rtb_Sum_j * (Autopilot_P.Constant_Value_ik - rtb_Sum_p) *
    Autopilot_DWork.Delay1_DSTATE_l + (rtb_out_f + Autopilot_DWork.Delay_DSTATE_b) * (rtb_Sum_p / rtb_Sum_j);
  if (Autopilot_U.in.data.H_radio_ft <= Autopilot_P.CompareToConstant_const_j) {
    Autopilot_B.u = Autopilot_U.in.data.H_dot_ft_min;
  }

  rtb_kntoms_i = Autopilot_P.kntoms_Gain_av * Autopilot_U.in.data.V_tas_kn;
  if (Autopilot_DWork.IC_FirstOutputTime_b) {
    Autopilot_DWork.IC_FirstOutputTime_b = false;
    rtb_IC_k = Autopilot_P.IC_Value_i;
  } else {
    rtb_IC_k = rtb_kntoms_i;
  }

  rtb_out_p = Autopilot_P.DiscreteDerivativeVariableTs_Gain_lf * Autopilot_U.in.data.V_ias_kn;
  rtb_Sum4_f = (rtb_out_p - Autopilot_DWork.Delay_DSTATE_ob) / Autopilot_U.in.time.dt * Autopilot_P.Gain3_Gain_fd +
    Autopilot_U.in.data.V_ias_kn;
  rtb_Sum_p = Autopilot_U.in.time.dt * Autopilot_P.LagFilter1_C1_p;
  rtb_Sum_j = rtb_Sum_p + Autopilot_P.Constant_Value_md;
  Autopilot_DWork.Delay1_DSTATE_lp = 1.0 / rtb_Sum_j * (Autopilot_P.Constant_Value_md - rtb_Sum_p) *
    Autopilot_DWork.Delay1_DSTATE_lp + (rtb_Sum4_f + Autopilot_DWork.Delay_DSTATE_ed) * (rtb_Sum_p / rtb_Sum_j);
  rtb_Mod2 = Autopilot_DWork.Delay1_DSTATE_lp - Autopilot_U.in.data.V_c_srs_kn;
  if (rtb_Mod2 > Autopilot_P.Saturation_UpperSat_n) {
    rtb_Mod2 = Autopilot_P.Saturation_UpperSat_n;
  } else {
    if (rtb_Mod2 < Autopilot_P.Saturation_LowerSat_h) {
      rtb_Mod2 = Autopilot_P.Saturation_LowerSat_h;
    }
  }

  rtb_Sum_p = Autopilot_P.Gain1_Gain_pc * rtb_Mod2;
  rtb_out_aa = Autopilot_P.Gain1_Gain_kw * Autopilot_U.in.data.alpha_deg;
  rtb_Mod2 = Autopilot_U.in.data.bz_m_s2 * std::sin(rtb_out_aa);
  rtb_out_aa = std::cos(rtb_out_aa);
  rtb_out_aa *= Autopilot_U.in.data.bx_m_s2;
  rtb_out_aa = (rtb_Mod2 + rtb_out_aa) * Autopilot_P.Gain_Gain_eg * Autopilot_P.Gain_Gain_hq;
  rtb_Mod2 = Autopilot_P.Gain1_Gain_k4 * std::abs(Autopilot_P.Constant_Value_j) + std::abs(rtb_out_aa);
  if (rtb_Mod2 <= Autopilot_P.Constant1_Value_l) {
    rtb_Mod2 = Autopilot_P.Constant1_Value_l;
  }

  rtb_Sum_j = std::abs(rtb_Sum_p);
  if (rtb_Sum_j < rtb_Mod2) {
    rtb_Mod2 = rtb_Sum_j;
  }

  if (rtb_out_aa < 0.0) {
    rtb_out_aa = -1.0;
  } else {
    if (rtb_out_aa > 0.0) {
      rtb_out_aa = 1.0;
    }
  }

  rtb_out_aa = rtb_Sum_p - rtb_Mod2 * rtb_out_aa * Autopilot_P.Gain_Gain_hk;
  rtb_Sum_p = Autopilot_P.kntoms_Gain_f * Autopilot_U.in.data.V_tas_kn;
  rtb_Mod2 = std::sin((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_fq * rtb_GainTheta1) *
                       Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_mn) * rtb_Sum_p;
  if (Autopilot_DWork.IC_FirstOutputTime_j) {
    Autopilot_DWork.IC_FirstOutputTime_j = false;
    rtb_Sum_p = Autopilot_P.IC_Value_e;
  }

  rtb_Mod2 = (Autopilot_P.Constant_Value_in - Autopilot_P.msftmin_Gain_ia * rtb_Mod2) * Autopilot_P.ftmintoms_Gain_i /
    rtb_Sum_p;
  x[0] = Autopilot_P.Constant1_Value_d - rtb_GainTheta;
  x[1] = rtb_out_aa;
  if (rtb_Mod2 > 1.0) {
    rtb_Mod2 = 1.0;
  } else {
    if (rtb_Mod2 < -1.0) {
      rtb_Mod2 = -1.0;
    }
  }

  x[2] = Autopilot_P.Gain_Gain_ic * std::asin(rtb_Mod2) * Autopilot_P.Gain_Gain_kp;
  switch (static_cast<int32_T>(rtb_Switch_mx)) {
   case 1:
    rtb_Minup = (Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft) * Autopilot_P.Gain_Gain_f;
    if (rtb_Minup > Autopilot_P.Saturation_UpperSat) {
      rtb_Minup = Autopilot_P.Saturation_UpperSat;
    } else {
      if (rtb_Minup < Autopilot_P.Saturation_LowerSat) {
        rtb_Minup = Autopilot_P.Saturation_LowerSat;
      }
    }

    rtb_Minup = (rtb_Minup - std::sin((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_it * rtb_GainTheta1) *
      Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_iu) * rtb_kntoms * Autopilot_P.msftmin_Gain_i) *
      Autopilot_P.ftmintoms_Gain_m / rtb_IC;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_p = Autopilot_P.Gain_Gain_j4 * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_bk;
    break;

   case 2:
    rtb_Mod2 = Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft;
    if (rtb_Mod2 < 0.0) {
      rtb_Minup = -1.0;
    } else if (rtb_Mod2 > 0.0) {
      rtb_Minup = 1.0;
    } else {
      rtb_Minup = rtb_Mod2;
    }

    rtb_Minup = ((Autopilot_P.Constant_Value_b * rtb_Minup + rtb_Mod2) * Autopilot_P.Gain_Gain_e - std::sin
                 ((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_e * rtb_GainTheta1) * Autopilot_U.in.data.alpha_deg) *
                  Autopilot_P.Gain1_Gain_h5) * rtb_kntoms_o * Autopilot_P.msftmin_Gain_o) * Autopilot_P.ftmintoms_Gain_l
      / rtb_IC_c;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_p = Autopilot_P.Gain_Gain_o * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_js;
    break;

   case 3:
    if (Autopilot_U.in.data.ap_H_c_ft - Autopilot_U.in.data.H_ind_ft > Autopilot_P.Switch_Threshold_k) {
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_p = Autopilot_P.Gain_Gain_cy * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_a2;
      if (rtb_Sum1_l5 > rtb_Sum_p) {
        rtb_Sum_p = rtb_Sum1_l5;
      }
    } else {
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_p = Autopilot_P.Gain_Gain_cy * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_a2;
      if (rtb_Sum1_l5 < rtb_Sum_p) {
        rtb_Sum_p = rtb_Sum1_l5;
      }
    }
    break;

   case 4:
    rtb_Minup = (Autopilot_U.in.data.ap_H_dot_c_ft_min - std::sin((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_o4 *
      rtb_GainTheta1) * Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_i) * rtb_kntoms_k *
                 Autopilot_P.msftmin_Gain_k) * Autopilot_P.ftmintoms_Gain_g / rtb_IC_n;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_p = Autopilot_P.Gain_Gain_m * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_i;
    break;

   case 5:
    rtb_Sum_p = (Autopilot_U.in.data.ap_FPA_c_deg - (rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_ld * rtb_GainTheta1)
      * Autopilot_U.in.data.alpha_deg)) * Autopilot_P.Gain_Gain_c;
    break;

   case 6:
    rtb_Sum_p = Autopilot_DWork.Delay1_DSTATE_l * look1_binlxpw(Autopilot_U.in.data.H_radio_ft,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_h, Autopilot_P.ScheduledGain_Table_i, 4U);
    break;

   case 7:
    if (rtb_on_ground > Autopilot_P.Switch_Threshold) {
      rtb_Sum_p = (rtb_GainTheta - Autopilot_P.Constant2_Value) * Autopilot_P.Gain4_Gain;
    } else {
      rtb_kntoms_o = Autopilot_B.u / Autopilot_P.Constant_Value_p * Autopilot_U.in.data.H_radio_ft;
      if (Autopilot_P.Constant3_Value_n < rtb_kntoms_o) {
        rtb_kntoms_o = Autopilot_P.Constant3_Value_n;
      }

      rtb_Minup = (rtb_kntoms_o - std::sin((rtb_GainTheta - std::cos(Autopilot_P.Gain1_Gain_h * rtb_GainTheta1) *
        Autopilot_U.in.data.alpha_deg) * Autopilot_P.Gain1_Gain_k) * rtb_kntoms_i * Autopilot_P.msftmin_Gain) *
        Autopilot_P.ftmintoms_Gain / rtb_IC_k;
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_p = Autopilot_P.Gain_Gain_j * std::asin(rtb_Minup) * Autopilot_P.Gain_Gain_n;
    }
    break;

   default:
    if (x[0] < rtb_out_aa) {
      if (rtb_out_aa < x[2]) {
        rtb_LAW = 1;
      } else if (x[0] < x[2]) {
        rtb_LAW = 2;
      } else {
        rtb_LAW = 0;
      }
    } else if (x[0] < x[2]) {
      rtb_LAW = 0;
    } else if (rtb_out_aa < x[2]) {
      rtb_LAW = 2;
    } else {
      rtb_LAW = 1;
    }

    rtb_Sum_p = x[rtb_LAW];
    break;
  }

  rtb_Sum_p += rtb_GainTheta;
  if (rtb_Sum_p > Autopilot_P.Constant1_Value_i) {
    rtb_Sum_p = Autopilot_P.Constant1_Value_i;
  } else {
    rtb_Mod2 = Autopilot_P.Gain1_Gain_c * Autopilot_P.Constant1_Value_i;
    if (rtb_Sum_p < rtb_Mod2) {
      rtb_Sum_p = rtb_Mod2;
    }
  }

  if (rtb_out == 0) {
    Autopilot_DWork.icLoad_h = 1U;
  }

  if (Autopilot_DWork.icLoad_h != 0) {
    Autopilot_DWork.Delay_DSTATE_ip = rtb_GainTheta;
  }

  rtb_Minup = rtb_Sum_p - Autopilot_DWork.Delay_DSTATE_ip;
  rtb_kntoms_o = Autopilot_P.Constant2_Value_h1 * Autopilot_U.in.time.dt;
  if (rtb_Minup < rtb_kntoms_o) {
    rtb_kntoms_o = rtb_Minup;
  }

  rtb_Minup = Autopilot_P.Gain1_Gain_g * Autopilot_P.Constant2_Value_h1 * Autopilot_U.in.time.dt;
  if (rtb_kntoms_o > rtb_Minup) {
    rtb_Minup = rtb_kntoms_o;
  }

  Autopilot_DWork.Delay_DSTATE_ip += rtb_Minup;
  rtb_Sum_j = Autopilot_U.in.time.dt * Autopilot_P.LagFilter_C1_k;
  rtb_out_aa = rtb_Sum_j + Autopilot_P.Constant_Value_iy;
  Autopilot_DWork.Delay1_DSTATE_b1 = 1.0 / rtb_out_aa * (Autopilot_P.Constant_Value_iy - rtb_Sum_j) *
    Autopilot_DWork.Delay1_DSTATE_b1 + (Autopilot_DWork.Delay_DSTATE_ip + Autopilot_DWork.Delay_DSTATE_f) * (rtb_Sum_j /
    rtb_out_aa);
  rtb_Minup = static_cast<real_T>(rtb_out) - Autopilot_DWork.Delay_DSTATE_l;
  rtb_kntoms_o = Autopilot_P.RateLimiterVariableTs_up_i * Autopilot_U.in.time.dt;
  if (rtb_Minup < rtb_kntoms_o) {
    rtb_kntoms_o = rtb_Minup;
  }

  rtb_Minup = Autopilot_U.in.time.dt * Autopilot_P.RateLimiterVariableTs_lo_o;
  if (rtb_kntoms_o > rtb_Minup) {
    rtb_Minup = rtb_kntoms_o;
  }

  Autopilot_DWork.Delay_DSTATE_l += rtb_Minup;
  if (Autopilot_DWork.Delay_DSTATE_l > Autopilot_P.Saturation_UpperSat_f) {
    rtb_Sum_j = Autopilot_P.Saturation_UpperSat_f;
  } else if (Autopilot_DWork.Delay_DSTATE_l < Autopilot_P.Saturation_LowerSat_a2) {
    rtb_Sum_j = Autopilot_P.Saturation_LowerSat_a2;
  } else {
    rtb_Sum_j = Autopilot_DWork.Delay_DSTATE_l;
  }

  rtb_Mod2 = Autopilot_DWork.Delay1_DSTATE_b1 * rtb_Sum_j;
  rtb_Sum_j = Autopilot_P.Constant_Value_ha - rtb_Sum_j;
  rtb_Sum_j *= rtb_GainTheta;
  Autopilot_Y.out.output.autopilot.Theta_c_deg = rtb_Mod2 + rtb_Sum_j;
  Autopilot_Y.out.time = Autopilot_U.in.time;
  Autopilot_Y.out.mode = Autopilot_U.in.mode;
  Autopilot_Y.out.input = Autopilot_U.in.input;
  Autopilot_Y.out.data.Theta_deg = rtb_GainTheta;
  Autopilot_Y.out.data.Phi_deg = rtb_GainTheta1;
  Autopilot_Y.out.data.qk_deg_s = result[1];
  Autopilot_Y.out.data.rk_deg_s = result[2];
  Autopilot_Y.out.data.pk_deg_s = result[0];
  Autopilot_Y.out.data.V_ias_kn = Autopilot_U.in.data.V_ias_kn;
  Autopilot_Y.out.data.V_tas_kn = Autopilot_U.in.data.V_tas_kn;
  Autopilot_Y.out.data.V_mach = Autopilot_U.in.data.V_mach;
  Autopilot_Y.out.data.V_gnd_kn = Autopilot_U.in.data.V_gnd_kn;
  Autopilot_Y.out.data.alpha_deg = Autopilot_U.in.data.alpha_deg;
  Autopilot_Y.out.data.H_ft = Autopilot_U.in.data.H_ft;
  Autopilot_Y.out.data.H_ind_ft = Autopilot_U.in.data.H_ind_ft;
  Autopilot_Y.out.data.H_radio_ft = Autopilot_U.in.data.H_radio_ft;
  Autopilot_Y.out.data.H_dot_ft_min = Autopilot_U.in.data.H_dot_ft_min;
  Autopilot_Y.out.data.Psi_magnetic_deg = Autopilot_U.in.data.Psi_magnetic_deg;
  Autopilot_Y.out.data.Psi_magnetic_track_deg = Autopilot_U.in.data.Psi_magnetic_track_deg;
  Autopilot_Y.out.data.Psi_true_deg = Autopilot_U.in.data.Psi_true_deg;
  Autopilot_Y.out.data.bx_m_s2 = Autopilot_U.in.data.bx_m_s2;
  Autopilot_Y.out.data.by_m_s2 = Autopilot_U.in.data.by_m_s2;
  Autopilot_Y.out.data.bz_m_s2 = Autopilot_U.in.data.bz_m_s2;
  Autopilot_Y.out.data.ap_V_c_kn = Autopilot_U.in.data.ap_V_c_kn;
  Autopilot_Y.out.data.ap_H_c_ft = Autopilot_U.in.data.ap_H_c_ft;
  Autopilot_Y.out.data.ap_Psi_c_deg = Autopilot_U.in.data.ap_Psi_c_deg;
  Autopilot_Y.out.data.ap_H_dot_c_ft_min = Autopilot_U.in.data.ap_H_dot_c_ft_min;
  Autopilot_Y.out.data.ap_FPA_c_deg = Autopilot_U.in.data.ap_FPA_c_deg;
  Autopilot_Y.out.data.nav_loc_deg = Autopilot_U.in.data.nav_loc_deg;
  Autopilot_Y.out.data.nav_radial_error_deg = Autopilot_U.in.data.nav_radial_error_deg;
  Autopilot_Y.out.data.nav_dme_nmi = Autopilot_U.in.data.nav_dme_nmi;
  Autopilot_Y.out.data.nav_gs_error_deg = Autopilot_U.in.data.nav_gs_error_deg;
  Autopilot_Y.out.data.flight_guidance_xtk_nmi = Autopilot_U.in.data.flight_guidance_xtk_nmi;
  Autopilot_Y.out.data.flight_guidance_tae_deg = Autopilot_U.in.data.flight_guidance_tae_deg;
  Autopilot_Y.out.data.V_c_srs_kn = Autopilot_U.in.data.V_c_srs_kn;
  Autopilot_Y.out.data.on_ground = rtb_on_ground;
  Autopilot_Y.out.data.zeta_deg = rtb_Saturation;
  Autopilot_Y.out.output.ap_on = rtb_out;
  Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_p;
  Autopilot_Y.out.output.autopilot.Phi_c_deg = rtb_out_lg + rtb_Product1_o;
  Autopilot_DWork.Delay_DSTATE_i = rtb_Saturation1;
  Autopilot_DWork.Delay_DSTATE_k = rtb_Mod1;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_DWork.Delay_DSTATE_h;
  Autopilot_DWork.Delay_DSTATE_ky = rtb_Switch;
  Autopilot_DWork.Delay_DSTATE_o = rtb_ROLLLIM1;
  Autopilot_DWork.Delay_DSTATE_e = rtb_out_ii;
  Autopilot_DWork.Delay_DSTATE_b = rtb_out_f;
  Autopilot_DWork.Delay_DSTATE_ob = rtb_out_p;
  Autopilot_DWork.Delay_DSTATE_ed = rtb_Sum4_f;
  Autopilot_DWork.icLoad_h = 0U;
  Autopilot_DWork.Delay_DSTATE_f = Autopilot_DWork.Delay_DSTATE_ip;
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
  Autopilot_DWork.IC_FirstOutputTime_n = true;
  Autopilot_DWork.IC_FirstOutputTime_i = true;
  Autopilot_DWork.IC_FirstOutputTime_d = true;
  Autopilot_DWork.IC_FirstOutputTime_b = true;
  Autopilot_DWork.IC_FirstOutputTime_j = true;
  Autopilot_DWork.Delay_DSTATE = Autopilot_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_i = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_g = Autopilot_P.Delay_InitialCondition_m;
  Autopilot_DWork.Delay1_DSTATE_b = Autopilot_P.Delay1_InitialCondition_i;
  Autopilot_DWork.Delay_DSTATE_c = Autopilot_P.RateLimiterVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_ky = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_e;
  Autopilot_DWork.Delay_DSTATE_o = Autopilot_P.Delay_InitialCondition_my;
  Autopilot_DWork.Delay1_DSTATE_k = Autopilot_P.Delay1_InitialCondition_m;
  Autopilot_DWork.Delay_DSTATE_e = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_h;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_P.Delay_InitialCondition_i;
  Autopilot_DWork.Delay1_DSTATE_l = Autopilot_P.Delay1_InitialCondition_g;
  Autopilot_DWork.Delay_DSTATE_ob = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_b;
  Autopilot_DWork.Delay_DSTATE_ed = Autopilot_P.Delay_InitialCondition_j;
  Autopilot_DWork.Delay1_DSTATE_lp = Autopilot_P.Delay1_InitialCondition_d;
  Autopilot_DWork.icLoad_h = 1U;
  Autopilot_DWork.Delay_DSTATE_f = Autopilot_P.Delay_InitialCondition_k;
  Autopilot_DWork.Delay1_DSTATE_b1 = Autopilot_P.Delay1_InitialCondition_e;
  Autopilot_DWork.Delay_DSTATE_l = Autopilot_P.RateLimiterVariableTs_InitialCondition_p;
  Autopilot_DWork.is_active_c5_Autopilot = 0U;
  Autopilot_DWork.is_c5_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_j;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_j;
  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_j;
  Autopilot_DWork.is_active_c6_Autopilot = 0U;
  Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_j;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_p);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_n);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_f);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_nr);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_i);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_g);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_gi);
  Autopilot_DWork.is_active_c10_Autopilot = 0U;
  Autopilot_DWork.is_c10_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_j;
  Autopilot_B.u = Autopilot_P.Y_Y0;
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
