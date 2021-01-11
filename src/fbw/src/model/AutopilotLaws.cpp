#include "AutopilotLaws.h"
#include "AutopilotLaws_private.h"
#include "look1_binlxpw.h"
#include "rt_modd.h"

const uint8_T AutopilotLaws_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T AutopilotLaws_IN_any = 1U;
const uint8_T AutopilotLaws_IN_left = 2U;
const uint8_T AutopilotLaws_IN_right = 3U;
const uint8_T AutopilotLaws_IN_InAir = 1U;
const uint8_T AutopilotLaws_IN_NO_ACTIVE_CHILD_n = 0U;
const uint8_T AutopilotLaws_IN_OnGround = 2U;
const ap_laws_output AutopilotLaws_rtZap_laws_output = {
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
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    false,
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

const ap_laws_input AutopilotLaws_rtZap_laws_input = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
};

void AutopilotLawsModelClass::AutopilotLaws_Chart_Init(rtDW_Chart_AutopilotLaws_T *localDW)
{
  localDW->is_active_c3_AutopilotLaws = 0U;
  localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD;
}

void AutopilotLawsModelClass::AutopilotLaws_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T
  *rty_out, rtDW_Chart_AutopilotLaws_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  if (localDW->is_active_c3_AutopilotLaws == 0U) {
    localDW->is_active_c3_AutopilotLaws = 1U;
    localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c3_AutopilotLaws) {
     case AutopilotLaws_IN_any:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_left;
        *rty_out = rtu_left;
      } else if (tmp_0 < tmp) {
        *rty_out = rtu_left;
      } else {
        *rty_out = rtu_right;
      }
      break;

     case AutopilotLaws_IN_left:
      tmp = std::abs(rtu_left);
      tmp_0 = std::abs(rtu_right);
      if ((rtu_use_short_path != 0.0) || (tmp_0 < 10.0) || (tmp < 10.0)) {
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
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
        localDW->is_c3_AutopilotLaws = AutopilotLaws_IN_any;
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

void AutopilotLawsModelClass::step()
{
  real_T result[3];
  real_T x[3];
  real_T rtb_Minup;
  real_T rtb_Tsxlo;
  real_T rtb_Sum_nb;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  real_T rtb_ManualSwitch;
  real_T rtb_Mod1_pg;
  real_T rtb_Mod2;
  real_T rtb_Mod2_d;
  real_T rtb_ManualSwitch_i;
  real_T rtb_Sum_fl;
  real_T rtb_out_br;
  real_T rtb_out;
  real_T rtb_out_m;
  real_T rtb_out_l;
  real_T rtb_out_d3;
  real_T rtb_Sum1_l5;
  real_T tmp[9];
  int32_T x_0;
  rtb_GainTheta = AutopilotLaws_P.GainTheta_Gain * AutopilotLaws_U.in.data.Theta_deg;
  rtb_GainTheta1 = AutopilotLaws_P.GainTheta1_Gain * AutopilotLaws_U.in.data.Phi_deg;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta;
  rtb_Mod1_pg = 0.017453292519943295 * rtb_GainTheta1;
  rtb_out_d3 = std::tan(rtb_Saturation1);
  rtb_Saturation = std::sin(rtb_Mod1_pg);
  rtb_Mod1_pg = std::cos(rtb_Mod1_pg);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation * rtb_out_d3;
  tmp[6] = rtb_Mod1_pg * rtb_out_d3;
  tmp[1] = 0.0;
  tmp[4] = rtb_Mod1_pg;
  tmp[7] = -rtb_Saturation;
  tmp[2] = 0.0;
  rtb_out_d3 = 1.0 / std::cos(rtb_Saturation1);
  tmp[5] = rtb_out_d3 * rtb_Saturation;
  tmp[8] = rtb_out_d3 * rtb_Mod1_pg;
  rtb_Saturation = AutopilotLaws_P.Gain_Gain_k * AutopilotLaws_U.in.data.p_rad_s * AutopilotLaws_P.Gainpk_Gain;
  rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_l * AutopilotLaws_U.in.data.q_rad_s * AutopilotLaws_P.Gainqk_Gain;
  rtb_out_d3 = AutopilotLaws_P.Gain_Gain_aq * AutopilotLaws_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_out_d3 + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
      tmp[rtb_on_ground] * rtb_Saturation);
  }

  rtb_Saturation = AutopilotLaws_P.Gain_Gain_af * AutopilotLaws_U.in.data.gear_strut_compression_1 -
    AutopilotLaws_P.Constant1_Value_p;
  if (rtb_Saturation > AutopilotLaws_P.Saturation_UpperSat_k) {
    rtb_Saturation = AutopilotLaws_P.Saturation_UpperSat_k;
  } else {
    if (rtb_Saturation < AutopilotLaws_P.Saturation_LowerSat_m) {
      rtb_Saturation = AutopilotLaws_P.Saturation_LowerSat_m;
    }
  }

  rtb_Saturation1 = AutopilotLaws_P.Gain1_Gain_k1e * AutopilotLaws_U.in.data.gear_strut_compression_2 -
    AutopilotLaws_P.Constant1_Value_p;
  if (rtb_Saturation1 > AutopilotLaws_P.Saturation1_UpperSat) {
    rtb_Saturation1 = AutopilotLaws_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < AutopilotLaws_P.Saturation1_LowerSat) {
      rtb_Saturation1 = AutopilotLaws_P.Saturation1_LowerSat;
    }
  }

  if (AutopilotLaws_DWork.is_active_c5_AutopilotLaws == 0U) {
    AutopilotLaws_DWork.is_active_c5_AutopilotLaws = 1U;
    AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_OnGround;
    rtb_on_ground = 1;
  } else if (AutopilotLaws_DWork.is_c5_AutopilotLaws == AutopilotLaws_IN_InAir) {
    if ((rtb_Saturation > 0.1) || (rtb_Saturation1 > 0.1)) {
      AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_Saturation = AutopilotLaws_P.Gain2_Gain * AutopilotLaws_U.in.data.zeta_pos;
  rtb_Saturation1 = rt_modd((AutopilotLaws_U.in.data.nav_loc_deg - (AutopilotLaws_U.in.data.Psi_magnetic_deg +
    AutopilotLaws_P.Constant3_Value_i)) + AutopilotLaws_P.Constant3_Value_i, AutopilotLaws_P.Constant3_Value_i);
  AutopilotLaws_Chart(rtb_Saturation1, AutopilotLaws_P.Gain_Gain_o * rt_modd(AutopilotLaws_P.Constant3_Value_i -
    rtb_Saturation1, AutopilotLaws_P.Constant3_Value_i), AutopilotLaws_P.Constant2_Value_l, &rtb_out_d3,
                      &AutopilotLaws_DWork.sf_Chart_k);
  if (AutopilotLaws_U.in.data.H_radio_ft <= AutopilotLaws_P.CompareToConstant_const) {
    rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_a * rtb_out_d3;
  } else {
    rtb_Saturation1 = AutopilotLaws_P.Constant1_Value;
  }

  rtb_Minup = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1;
  rtb_Tsxlo = rtb_Minup + AutopilotLaws_P.Constant_Value_o;
  AutopilotLaws_DWork.Delay1_DSTATE = 1.0 / rtb_Tsxlo * (AutopilotLaws_P.Constant_Value_o - rtb_Minup) *
    AutopilotLaws_DWork.Delay1_DSTATE + (rtb_Saturation1 + AutopilotLaws_DWork.Delay_DSTATE) * (rtb_Minup / rtb_Tsxlo);
  if (AutopilotLaws_P.ManualSwitch_CurrentSetting == 1) {
    rtb_ManualSwitch = AutopilotLaws_P.Constant_Value;
  } else {
    rtb_ManualSwitch = AutopilotLaws_U.in.input.lateral_law;
  }

  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_o) {
    rtb_out_d3 = AutopilotLaws_P.Saturation_UpperSat_o;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_o5) {
    rtb_out_d3 = AutopilotLaws_P.Saturation_LowerSat_o5;
  } else {
    rtb_out_d3 = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_Tsxlo = std::sin(AutopilotLaws_P.Gain1_Gain_a * AutopilotLaws_U.in.data.nav_radial_error_deg) * rtb_out_d3 *
    AutopilotLaws_P.Gain2_Gain_g;
  if (rtb_Tsxlo > AutopilotLaws_P.Saturation1_UpperSat_g) {
    rtb_Tsxlo = AutopilotLaws_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_Tsxlo < AutopilotLaws_P.Saturation1_LowerSat_k) {
      rtb_Tsxlo = AutopilotLaws_P.Saturation1_LowerSat_k;
    }
  }

  if (rtb_ManualSwitch != AutopilotLaws_P.CompareToConstant_const_k) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  AutopilotLaws_DWork.Delay_DSTATE_h += AutopilotLaws_P.Gain6_Gain * rtb_Tsxlo *
    AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_Gain * AutopilotLaws_U.in.time.dt;
  if (AutopilotLaws_DWork.Delay_DSTATE_h > AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (AutopilotLaws_DWork.Delay_DSTATE_h < AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  rtb_out_d3 = AutopilotLaws_U.in.data.nav_radial_error_deg + AutopilotLaws_U.in.data.nav_loc_deg;
  rtb_Mod1_pg = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_deg - (rt_modd(rt_modd(rtb_out_d3,
    AutopilotLaws_P.Constant3_Value_l) + AutopilotLaws_P.Constant3_Value_l, AutopilotLaws_P.Constant3_Value_l) +
    AutopilotLaws_P.Constant3_Value_p)) + AutopilotLaws_P.Constant3_Value_p, AutopilotLaws_P.Constant3_Value_p);
  rtb_Mod2 = rt_modd(AutopilotLaws_P.Constant3_Value_p - rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_p);
  if (rtb_Mod1_pg < rtb_Mod2) {
    rtb_Mod1_pg *= AutopilotLaws_P.Gain1_Gain_b;
  } else {
    rtb_Mod1_pg = AutopilotLaws_P.Gain_Gain_n * rtb_Mod2;
  }

  rtb_Mod1_pg = rt_modd((rt_modd(rt_modd(((rtb_Tsxlo * look1_binlxpw(AutopilotLaws_U.in.data.V_gnd_kn,
    AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1, AutopilotLaws_P.ScheduledGain_Table, 2U) +
    AutopilotLaws_DWork.Delay_DSTATE_h) + AutopilotLaws_P.Gain1_Gain_fq * rtb_Mod1_pg) +
    AutopilotLaws_U.in.data.Psi_magnetic_deg, AutopilotLaws_P.Constant3_Value_a) + AutopilotLaws_P.Constant3_Value_a,
    AutopilotLaws_P.Constant3_Value_a) - (AutopilotLaws_U.in.data.Psi_magnetic_deg + AutopilotLaws_P.Constant3_Value_io))
                        + AutopilotLaws_P.Constant3_Value_io, AutopilotLaws_P.Constant3_Value_io);
  AutopilotLaws_Chart(rtb_Mod1_pg, AutopilotLaws_P.Gain_Gain_o0 * rt_modd(AutopilotLaws_P.Constant3_Value_io -
    rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_io), AutopilotLaws_P.Constant1_Value_e, &rtb_Tsxlo,
                      &AutopilotLaws_DWork.sf_Chart_j);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_Tsxlo = AutopilotLaws_P.beta_Value_ed;
    break;

   case 1:
    rtb_Tsxlo = AutopilotLaws_P.beta_Value_e;
    break;

   case 2:
    rtb_Tsxlo = AutopilotLaws_P.beta_Value_b;
    break;

   case 3:
    rtb_Tsxlo = AutopilotLaws_P.beta_Value_m;
    break;

   case 4:
    rtb_Tsxlo = AutopilotLaws_P.beta_Value;
    break;

   case 5:
    rtb_Tsxlo = AutopilotLaws_DWork.Delay1_DSTATE;
    break;

   default:
    rtb_Tsxlo = (AutopilotLaws_P.Gain5_Gain * rtb_Tsxlo + AutopilotLaws_P.Gain_Gain_b * result[2]) + rtb_Saturation;
    break;
  }

  rtb_Mod2 = look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn, AutopilotLaws_P.ROLLLIM1_bp01Data,
    AutopilotLaws_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1_pg = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_deg +
    AutopilotLaws_P.Constant3_Value_d)) + AutopilotLaws_P.Constant3_Value_d, AutopilotLaws_P.Constant3_Value_d);
  AutopilotLaws_Chart(rtb_Mod1_pg, AutopilotLaws_P.Gain_Gain_dq * rt_modd(AutopilotLaws_P.Constant3_Value_d -
    rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_d), AutopilotLaws_P.Constant_Value_c, &rtb_out_br,
                      &AutopilotLaws_DWork.sf_Chart_f);
  rtb_Mod1_pg = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_track_deg +
    AutopilotLaws_P.Constant3_Value_k)) + AutopilotLaws_P.Constant3_Value_k, AutopilotLaws_P.Constant3_Value_k);
  AutopilotLaws_Chart(rtb_Mod1_pg, AutopilotLaws_P.Gain_Gain_ee * rt_modd(AutopilotLaws_P.Constant3_Value_k -
    rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_k), AutopilotLaws_P.Constant_Value_i, &rtb_out,
                      &AutopilotLaws_DWork.sf_Chart_l);
  rtb_Minup = AutopilotLaws_P.Gain_Gain_nu * AutopilotLaws_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_Minup > AutopilotLaws_P.Saturation_UpperSat_kr) {
    rtb_Minup = AutopilotLaws_P.Saturation_UpperSat_kr;
  } else {
    if (rtb_Minup < AutopilotLaws_P.Saturation_LowerSat_p) {
      rtb_Minup = AutopilotLaws_P.Saturation_LowerSat_p;
    }
  }

  rtb_Mod1_pg = rt_modd((rt_modd(rt_modd((AutopilotLaws_P.Gain2_Gain_f * AutopilotLaws_U.in.data.flight_guidance_tae_deg
    + rtb_Minup) * AutopilotLaws_P.Gain1_Gain_n + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_nx) + AutopilotLaws_P.Constant3_Value_nx, AutopilotLaws_P.Constant3_Value_nx) -
    (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_c)) +
                        AutopilotLaws_P.Constant3_Value_c, AutopilotLaws_P.Constant3_Value_c);
  AutopilotLaws_Chart(rtb_Mod1_pg, AutopilotLaws_P.Gain_Gain_kr * rt_modd(AutopilotLaws_P.Constant3_Value_c -
    rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_c), AutopilotLaws_P.Constant_Value_cw, &rtb_out_m,
                      &AutopilotLaws_DWork.sf_Chart);
  rtb_Mod1_pg = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_track_deg - (rt_modd(rt_modd(rtb_out_d3,
    AutopilotLaws_P.Constant3_Value_dg) + AutopilotLaws_P.Constant3_Value_dg, AutopilotLaws_P.Constant3_Value_dg) +
    AutopilotLaws_P.Constant3_Value_g)) + AutopilotLaws_P.Constant3_Value_g, AutopilotLaws_P.Constant3_Value_g);
  rtb_Mod2_d = rt_modd(AutopilotLaws_P.Constant3_Value_g - rtb_Mod1_pg, AutopilotLaws_P.Constant3_Value_g);
  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_m) {
    rtb_out_d3 = AutopilotLaws_P.Saturation_UpperSat_m;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_k) {
    rtb_out_d3 = AutopilotLaws_P.Saturation_LowerSat_k;
  } else {
    rtb_out_d3 = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_Minup = std::sin(AutopilotLaws_P.Gain1_Gain_h2 * AutopilotLaws_U.in.data.nav_radial_error_deg) * rtb_out_d3 *
    look1_binlxpw(AutopilotLaws_U.in.data.nav_dme_nmi, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_ah,
                  AutopilotLaws_P.ScheduledGain_Table_p, 4U);
  if (rtb_Minup > AutopilotLaws_P.Saturation1_UpperSat_i) {
    rtb_Minup = AutopilotLaws_P.Saturation1_UpperSat_i;
  } else {
    if (rtb_Minup < AutopilotLaws_P.Saturation1_LowerSat_g) {
      rtb_Minup = AutopilotLaws_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Mod1_pg < rtb_Mod2_d) {
    rtb_Mod1_pg *= AutopilotLaws_P.Gain1_Gain;
  } else {
    rtb_Mod1_pg = AutopilotLaws_P.Gain_Gain * rtb_Mod2_d;
  }

  rtb_out_d3 = rt_modd((rt_modd(rt_modd((rtb_Minup + rtb_Mod1_pg) * AutopilotLaws_P.Gain3_Gain +
    AutopilotLaws_U.in.data.Psi_magnetic_track_deg, AutopilotLaws_P.Constant3_Value_af) +
    AutopilotLaws_P.Constant3_Value_af, AutopilotLaws_P.Constant3_Value_af) -
                        (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_kw)) +
                       AutopilotLaws_P.Constant3_Value_kw, AutopilotLaws_P.Constant3_Value_kw);
  AutopilotLaws_Chart(rtb_out_d3, AutopilotLaws_P.Gain_Gain_bq * rt_modd(AutopilotLaws_P.Constant3_Value_kw - rtb_out_d3,
    AutopilotLaws_P.Constant3_Value_kw), AutopilotLaws_P.Constant_Value_e, &rtb_Mod2_d, &AutopilotLaws_DWork.sf_Chart_h);
  rtb_out_d3 = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain * AutopilotLaws_U.in.data.nav_radial_error_deg;
  rtb_Mod1_pg = (rtb_out_d3 - AutopilotLaws_DWork.Delay_DSTATE_n) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_i + AutopilotLaws_U.in.data.nav_radial_error_deg;
  rtb_out_l = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1;
  rtb_Minup = rtb_out_l + AutopilotLaws_P.Constant_Value_c5;
  AutopilotLaws_DWork.Delay1_DSTATE_j = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_c5 - rtb_out_l) *
    AutopilotLaws_DWork.Delay1_DSTATE_j + (rtb_Mod1_pg + AutopilotLaws_DWork.Delay_DSTATE_b) * (rtb_out_l / rtb_Minup);
  rtb_Minup = rt_modd((rt_modd(rt_modd(AutopilotLaws_DWork.Delay1_DSTATE_j * look1_binlxpw
    (AutopilotLaws_U.in.data.H_radio_ft, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_e,
     AutopilotLaws_P.ScheduledGain_Table_pf, 4U) + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_f) + AutopilotLaws_P.Constant3_Value_f, AutopilotLaws_P.Constant3_Value_f) -
                       (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_m)) +
                      AutopilotLaws_P.Constant3_Value_m, AutopilotLaws_P.Constant3_Value_m);
  AutopilotLaws_Chart(rtb_Minup, AutopilotLaws_P.Gain_Gain_j * rt_modd(AutopilotLaws_P.Constant3_Value_m - rtb_Minup,
    AutopilotLaws_P.Constant3_Value_m), AutopilotLaws_P.Constant_Value_p1, &rtb_out_l, &AutopilotLaws_DWork.sf_Chart_b);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_out_l = rtb_GainTheta1;
    break;

   case 1:
    rtb_out_l = rtb_out_br * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_l, AutopilotLaws_P.ScheduledGain_Table_e, 3U);
    break;

   case 2:
    rtb_out_l = rtb_out * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_a, AutopilotLaws_P.ScheduledGain_Table_m, 3U);
    break;

   case 3:
    rtb_out_l = rtb_out_m * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_lw, AutopilotLaws_P.ScheduledGain_Table_a, 3U);
    break;

   case 4:
    rtb_out_l = rtb_Mod2_d * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_c, AutopilotLaws_P.ScheduledGain_Table_d, 3U);
    break;

   case 5:
    rtb_out_l *= look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_g, AutopilotLaws_P.ScheduledGain_Table_j, 3U);
    break;

   default:
    rtb_out_l = AutopilotLaws_P.Constant3_Value;
    break;
  }

  if (rtb_out_l > rtb_Mod2) {
    rtb_out_l = rtb_Mod2;
  } else {
    rtb_ManualSwitch = AutopilotLaws_P.Gain1_Gain_l * rtb_Mod2;
    if (rtb_out_l < rtb_ManualSwitch) {
      rtb_out_l = rtb_ManualSwitch;
    }
  }

  if (AutopilotLaws_U.in.input.enabled == 0.0) {
    AutopilotLaws_DWork.icLoad = 1U;
  }

  if (AutopilotLaws_DWork.icLoad != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_hc = rtb_GainTheta1;
  }

  rtb_Minup = rtb_out_l - AutopilotLaws_DWork.Delay_DSTATE_hc;
  rtb_ManualSwitch_i = AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_Minup < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Minup;
  }

  rtb_Minup = AutopilotLaws_P.Gain1_Gain_kf * AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_ManualSwitch_i > rtb_Minup) {
    rtb_Minup = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_hc += rtb_Minup;
  rtb_Sum_nb = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_l;
  rtb_Minup = rtb_Sum_nb + AutopilotLaws_P.Constant_Value_p2;
  AutopilotLaws_DWork.Delay1_DSTATE_f = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_p2 - rtb_Sum_nb) *
    AutopilotLaws_DWork.Delay1_DSTATE_f + (AutopilotLaws_DWork.Delay_DSTATE_hc + AutopilotLaws_DWork.Delay_DSTATE_hr) *
    (rtb_Sum_nb / rtb_Minup);
  rtb_Minup = AutopilotLaws_U.in.input.enabled - AutopilotLaws_DWork.Delay_DSTATE_i;
  rtb_ManualSwitch_i = AutopilotLaws_P.RateLimiterVariableTs_up * AutopilotLaws_U.in.time.dt;
  if (rtb_Minup < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Minup;
  }

  rtb_Minup = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo;
  if (rtb_ManualSwitch_i > rtb_Minup) {
    rtb_Minup = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_i += rtb_Minup;
  if (AutopilotLaws_DWork.Delay_DSTATE_i > AutopilotLaws_P.Saturation_UpperSat_ma) {
    rtb_Sum_fl = AutopilotLaws_P.Saturation_UpperSat_ma;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_i < AutopilotLaws_P.Saturation_LowerSat_ej) {
    rtb_Sum_fl = AutopilotLaws_P.Saturation_LowerSat_ej;
  } else {
    rtb_Sum_fl = AutopilotLaws_DWork.Delay_DSTATE_i;
  }

  rtb_ManualSwitch = AutopilotLaws_DWork.Delay1_DSTATE_f * rtb_Sum_fl;
  rtb_Sum_fl = AutopilotLaws_P.Constant_Value_g - rtb_Sum_fl;
  rtb_Sum_fl *= rtb_GainTheta1;
  AutopilotLaws_Y.out.output.autopilot.Phi_c_deg = rtb_ManualSwitch + rtb_Sum_fl;
  AutopilotLaws_Y.out.output.flight_director.Beta_c_deg = rtb_Tsxlo;
  AutopilotLaws_Y.out.output.autopilot.Beta_c_deg = rtb_Tsxlo;
  AutopilotLaws_Y.out.output.flight_director.Phi_c_deg = rtb_out_l;
  if (AutopilotLaws_P.ManualSwitch_CurrentSetting_b == 1) {
    rtb_ManualSwitch_i = AutopilotLaws_P.Constant_Value_m;
  } else {
    rtb_ManualSwitch_i = AutopilotLaws_U.in.input.vertical_law;
  }

  if (rtb_ManualSwitch_i != AutopilotLaws_P.CompareToConstant5_const) {
    AutopilotLaws_B.u_n = (AutopilotLaws_U.in.input.H_c_ft + AutopilotLaws_U.in.data.H_ft) -
      AutopilotLaws_U.in.data.H_ind_ft;
  }

  rtb_ManualSwitch = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_m * AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Mod2 = (rtb_ManualSwitch - AutopilotLaws_DWork.Delay_DSTATE_f) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_f + AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Sum_fl = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1_a;
  rtb_Sum_nb = rtb_Sum_fl + AutopilotLaws_P.Constant_Value_h;
  AutopilotLaws_DWork.Delay1_DSTATE_c = 1.0 / rtb_Sum_nb * (AutopilotLaws_P.Constant_Value_h - rtb_Sum_fl) *
    AutopilotLaws_DWork.Delay1_DSTATE_c + (rtb_Mod2 + AutopilotLaws_DWork.Delay_DSTATE_l) * (rtb_Sum_fl / rtb_Sum_nb);
  rtb_Minup = AutopilotLaws_DWork.Delay1_DSTATE_c - AutopilotLaws_U.in.data.ap_V_c_kn;
  if (rtb_Minup > AutopilotLaws_P.Saturation_UpperSat_h) {
    rtb_Minup = AutopilotLaws_P.Saturation_UpperSat_h;
  } else {
    if (rtb_Minup < AutopilotLaws_P.Saturation_LowerSat_c) {
      rtb_Minup = AutopilotLaws_P.Saturation_LowerSat_c;
    }
  }

  rtb_Sum_fl = AutopilotLaws_P.Gain1_Gain_bz * rtb_Minup;
  rtb_out_l = AutopilotLaws_P.Gain1_Gain_bn * AutopilotLaws_U.in.data.alpha_deg;
  rtb_Tsxlo = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_out_l);
  rtb_out_l = std::cos(rtb_out_l);
  rtb_out_l *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_out_l = (rtb_Tsxlo + rtb_out_l) * AutopilotLaws_P.Gain_Gain_fs * AutopilotLaws_P.Gain_Gain_i;
  rtb_Minup = AutopilotLaws_P.Gain1_Gain_k0 * std::abs(AutopilotLaws_P.Constant_Value_em) + std::abs(rtb_out_l);
  if (rtb_Minup <= AutopilotLaws_P.Constant1_Value_ef) {
    rtb_Minup = AutopilotLaws_P.Constant1_Value_ef;
  }

  rtb_Tsxlo = std::abs(rtb_Sum_fl);
  if (rtb_Tsxlo < rtb_Minup) {
    rtb_Minup = rtb_Tsxlo;
  }

  if (rtb_out_l < 0.0) {
    rtb_out_l = -1.0;
  } else {
    if (rtb_out_l > 0.0) {
      rtb_out_l = 1.0;
    }
  }

  rtb_Sum1_l5 = rtb_Sum_fl - rtb_Minup * rtb_out_l * AutopilotLaws_P.Gain_Gain_h4;
  rtb_Sum_fl = AutopilotLaws_P.kntoms_Gain_i * AutopilotLaws_U.in.data.V_tas_kn;
  rtb_Tsxlo = std::sin((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_j * rtb_GainTheta1) *
                        AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_kp) * rtb_Sum_fl *
    AutopilotLaws_P.msftmin_Gain_j;
  if (rtb_Sum_fl > AutopilotLaws_P.Saturation_UpperSat_ea) {
    rtb_Sum_fl = AutopilotLaws_P.Saturation_UpperSat_ea;
  } else {
    if (rtb_Sum_fl < AutopilotLaws_P.Saturation_LowerSat_fv) {
      rtb_Sum_fl = AutopilotLaws_P.Saturation_LowerSat_fv;
    }
  }

  rtb_Minup = (AutopilotLaws_P.Constant_Value_k - rtb_Tsxlo) * AutopilotLaws_P.ftmintoms_Gain_b / rtb_Sum_fl;
  rtb_out_br = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_l * AutopilotLaws_U.in.data.nav_gs_error_deg;
  rtb_out = (rtb_out_br - AutopilotLaws_DWork.Delay_DSTATE_nb) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_o + AutopilotLaws_P.Gain1_Gain_jl * AutopilotLaws_U.in.data.nav_gs_error_deg;
  rtb_Sum_fl = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_n;
  rtb_Sum_nb = rtb_Sum_fl + AutopilotLaws_P.Constant_Value_py;
  AutopilotLaws_DWork.Delay1_DSTATE_je = 1.0 / rtb_Sum_nb * (AutopilotLaws_P.Constant_Value_py - rtb_Sum_fl) *
    AutopilotLaws_DWork.Delay1_DSTATE_je + (rtb_out + AutopilotLaws_DWork.Delay_DSTATE_p) * (rtb_Sum_fl / rtb_Sum_nb);
  if (AutopilotLaws_U.in.data.H_radio_ft <= AutopilotLaws_P.CompareToConstant_const_j) {
    AutopilotLaws_B.u = AutopilotLaws_U.in.data.H_dot_ft_min;
  }

  rtb_out_m = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_lf * AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Mod2_d = (rtb_out_m - AutopilotLaws_DWork.Delay_DSTATE_d) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_fd + AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Sum_fl = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1_p;
  rtb_Sum_nb = rtb_Sum_fl + AutopilotLaws_P.Constant_Value_pk;
  AutopilotLaws_DWork.Delay1_DSTATE_i = 1.0 / rtb_Sum_nb * (AutopilotLaws_P.Constant_Value_pk - rtb_Sum_fl) *
    AutopilotLaws_DWork.Delay1_DSTATE_i + (rtb_Mod2_d + AutopilotLaws_DWork.Delay_DSTATE_fs) * (rtb_Sum_fl / rtb_Sum_nb);
  rtb_Tsxlo = AutopilotLaws_DWork.Delay1_DSTATE_i - (AutopilotLaws_U.in.data.V2_kn + AutopilotLaws_P.Bias_Bias);
  if (rtb_Tsxlo > AutopilotLaws_P.Saturation_UpperSat_nk) {
    rtb_Tsxlo = AutopilotLaws_P.Saturation_UpperSat_nk;
  } else {
    if (rtb_Tsxlo < AutopilotLaws_P.Saturation_LowerSat_h) {
      rtb_Tsxlo = AutopilotLaws_P.Saturation_LowerSat_h;
    }
  }

  rtb_Sum_fl = AutopilotLaws_P.Gain1_Gain_p * rtb_Tsxlo;
  rtb_out_l = AutopilotLaws_P.Gain1_Gain_d * AutopilotLaws_U.in.data.alpha_deg;
  rtb_Tsxlo = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_out_l);
  rtb_out_l = std::cos(rtb_out_l);
  rtb_out_l *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_out_l = (rtb_Tsxlo + rtb_out_l) * AutopilotLaws_P.Gain_Gain_cv * AutopilotLaws_P.Gain_Gain_pm;
  rtb_Tsxlo = AutopilotLaws_P.Gain1_Gain_k4 * std::abs(AutopilotLaws_P.Constant_Value_j) + std::abs(rtb_out_l);
  if (rtb_Tsxlo <= AutopilotLaws_P.Constant1_Value_l) {
    rtb_Tsxlo = AutopilotLaws_P.Constant1_Value_l;
  }

  rtb_Sum_nb = std::abs(rtb_Sum_fl);
  if (rtb_Sum_nb < rtb_Tsxlo) {
    rtb_Tsxlo = rtb_Sum_nb;
  }

  if (rtb_out_l < 0.0) {
    rtb_out_l = -1.0;
  } else {
    if (rtb_out_l > 0.0) {
      rtb_out_l = 1.0;
    }
  }

  rtb_out_l = rtb_Sum_fl - rtb_Tsxlo * rtb_out_l * AutopilotLaws_P.Gain_Gain_hk;
  rtb_Sum_fl = AutopilotLaws_P.kntoms_Gain_f * AutopilotLaws_U.in.data.V_tas_kn;
  rtb_Tsxlo = std::sin((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_bzv * rtb_GainTheta1) *
                        AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_fe) * rtb_Sum_fl;
  if (rtb_Sum_fl > AutopilotLaws_P.Saturation_UpperSat_md) {
    rtb_Sum_fl = AutopilotLaws_P.Saturation_UpperSat_md;
  } else {
    if (rtb_Sum_fl < AutopilotLaws_P.Saturation_LowerSat_g) {
      rtb_Sum_fl = AutopilotLaws_P.Saturation_LowerSat_g;
    }
  }

  rtb_Tsxlo = (AutopilotLaws_P.Constant_Value_in - AutopilotLaws_P.msftmin_Gain_a * rtb_Tsxlo) *
    AutopilotLaws_P.ftmintoms_Gain_m / rtb_Sum_fl;
  x[0] = AutopilotLaws_P.Constant1_Value_d - rtb_GainTheta;
  x[1] = rtb_out_l;
  if (rtb_Tsxlo > 1.0) {
    rtb_Tsxlo = 1.0;
  } else {
    if (rtb_Tsxlo < -1.0) {
      rtb_Tsxlo = -1.0;
    }
  }

  x[2] = AutopilotLaws_P.Gain_Gain_n5 * std::asin(rtb_Tsxlo) * AutopilotLaws_P.Gain_Gain_ax;
  switch (static_cast<int32_T>(rtb_ManualSwitch_i)) {
   case 0:
    rtb_Sum_fl = rtb_GainTheta;
    break;

   case 1:
    rtb_Tsxlo = AutopilotLaws_P.kntoms_Gain_n * AutopilotLaws_U.in.data.V_tas_kn;
    rtb_Minup = (AutopilotLaws_B.u_n - AutopilotLaws_U.in.data.H_ft) * AutopilotLaws_P.Gain_Gain_f;
    if (rtb_Minup > AutopilotLaws_P.Saturation_UpperSat_n) {
      rtb_Minup = AutopilotLaws_P.Saturation_UpperSat_n;
    } else {
      if (rtb_Minup < AutopilotLaws_P.Saturation_LowerSat_d) {
        rtb_Minup = AutopilotLaws_P.Saturation_LowerSat_d;
      }
    }

    if (rtb_Tsxlo > AutopilotLaws_P.Saturation_UpperSat_f) {
      rtb_out_l = AutopilotLaws_P.Saturation_UpperSat_f;
    } else if (rtb_Tsxlo < AutopilotLaws_P.Saturation_LowerSat_o) {
      rtb_out_l = AutopilotLaws_P.Saturation_LowerSat_o;
    } else {
      rtb_out_l = rtb_Tsxlo;
    }

    rtb_Minup = (rtb_Minup - std::sin((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_c * rtb_GainTheta1) *
      AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_lz) * rtb_Tsxlo * AutopilotLaws_P.msftmin_Gain_d) *
      AutopilotLaws_P.ftmintoms_Gain_n / rtb_out_l;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_cf * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_p0;
    break;

   case 2:
    rtb_Tsxlo = AutopilotLaws_P.kntoms_Gain_e * AutopilotLaws_U.in.data.V_tas_kn;
    rtb_Minup = AutopilotLaws_U.in.input.H_c_ft - AutopilotLaws_U.in.data.H_ind_ft;
    if (rtb_Minup < 0.0) {
      rtb_out_l = -1.0;
    } else if (rtb_Minup > 0.0) {
      rtb_out_l = 1.0;
    } else {
      rtb_out_l = rtb_Minup;
    }

    if (rtb_Tsxlo > AutopilotLaws_P.Saturation_UpperSat_g) {
      rtb_Sum_fl = AutopilotLaws_P.Saturation_UpperSat_g;
    } else if (rtb_Tsxlo < AutopilotLaws_P.Saturation_LowerSat_e) {
      rtb_Sum_fl = AutopilotLaws_P.Saturation_LowerSat_e;
    } else {
      rtb_Sum_fl = rtb_Tsxlo;
    }

    rtb_Minup = ((AutopilotLaws_P.Constant_Value_b * rtb_out_l + rtb_Minup) * AutopilotLaws_P.Gain_Gain_el - std::sin
                 ((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_e5 * rtb_GainTheta1) *
                   AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_o) * rtb_Tsxlo *
                 AutopilotLaws_P.msftmin_Gain_k) * AutopilotLaws_P.ftmintoms_Gain_c / rtb_Sum_fl;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_d * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_p;
    break;

   case 3:
    if (AutopilotLaws_U.in.input.H_c_ft - AutopilotLaws_U.in.data.H_ind_ft > AutopilotLaws_P.Switch_Threshold_k) {
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_js * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_ko;
      if (rtb_Sum1_l5 > rtb_Sum_fl) {
        rtb_Sum_fl = rtb_Sum1_l5;
      }
    } else {
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_js * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_ko;
      if (rtb_Sum1_l5 < rtb_Sum_fl) {
        rtb_Sum_fl = rtb_Sum1_l5;
      }
    }
    break;

   case 4:
    rtb_Tsxlo = AutopilotLaws_P.kntoms_Gain_a * AutopilotLaws_U.in.data.V_tas_kn;
    if (rtb_Tsxlo > AutopilotLaws_P.Saturation_UpperSat_e) {
      rtb_Minup = AutopilotLaws_P.Saturation_UpperSat_e;
    } else if (rtb_Tsxlo < AutopilotLaws_P.Saturation_LowerSat_f) {
      rtb_Minup = AutopilotLaws_P.Saturation_LowerSat_f;
    } else {
      rtb_Minup = rtb_Tsxlo;
    }

    rtb_Minup = (AutopilotLaws_U.in.input.H_dot_c_fpm - std::sin((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_f *
      rtb_GainTheta1) * AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_k1) * rtb_Tsxlo *
                 AutopilotLaws_P.msftmin_Gain_p) * AutopilotLaws_P.ftmintoms_Gain_d / rtb_Minup;
    if (rtb_Minup > 1.0) {
      rtb_Minup = 1.0;
    } else {
      if (rtb_Minup < -1.0) {
        rtb_Minup = -1.0;
      }
    }

    rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_h * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_bk;
    break;

   case 5:
    rtb_Sum_fl = (AutopilotLaws_U.in.input.FPA_c_deg - (rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_m *
      rtb_GainTheta1) * AutopilotLaws_U.in.data.alpha_deg)) * AutopilotLaws_P.Gain_Gain_c;
    break;

   case 6:
    rtb_Sum_fl = AutopilotLaws_DWork.Delay1_DSTATE_je * look1_binlxpw(AutopilotLaws_U.in.data.H_radio_ft,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_h, AutopilotLaws_P.ScheduledGain_Table_i, 4U);
    break;

   case 7:
    if (rtb_on_ground > AutopilotLaws_P.Switch_Threshold) {
      rtb_Sum_fl = (rtb_GainTheta - AutopilotLaws_P.Constant2_Value) * AutopilotLaws_P.Gain4_Gain;
    } else {
      rtb_Tsxlo = AutopilotLaws_P.kntoms_Gain * AutopilotLaws_U.in.data.V_tas_kn;
      rtb_ManualSwitch_i = AutopilotLaws_B.u / AutopilotLaws_P.Constant_Value_p * AutopilotLaws_U.in.data.H_radio_ft;
      if (AutopilotLaws_P.Constant3_Value_n < rtb_ManualSwitch_i) {
        rtb_ManualSwitch_i = AutopilotLaws_P.Constant3_Value_n;
      }

      if (rtb_Tsxlo > AutopilotLaws_P.Saturation_UpperSat) {
        rtb_Minup = AutopilotLaws_P.Saturation_UpperSat;
      } else if (rtb_Tsxlo < AutopilotLaws_P.Saturation_LowerSat) {
        rtb_Minup = AutopilotLaws_P.Saturation_LowerSat;
      } else {
        rtb_Minup = rtb_Tsxlo;
      }

      rtb_Minup = (rtb_ManualSwitch_i - std::sin((rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_k * rtb_GainTheta1)
        * AutopilotLaws_U.in.data.alpha_deg) * AutopilotLaws_P.Gain1_Gain_e) * rtb_Tsxlo * AutopilotLaws_P.msftmin_Gain)
        * AutopilotLaws_P.ftmintoms_Gain / rtb_Minup;
      if (rtb_Minup > 1.0) {
        rtb_Minup = 1.0;
      } else {
        if (rtb_Minup < -1.0) {
          rtb_Minup = -1.0;
        }
      }

      rtb_Sum_fl = AutopilotLaws_P.Gain_Gain_bo * std::asin(rtb_Minup) * AutopilotLaws_P.Gain_Gain_e;
    }
    break;

   default:
    if (x[0] < rtb_out_l) {
      if (rtb_out_l < x[2]) {
        x_0 = 1;
      } else if (x[0] < x[2]) {
        x_0 = 2;
      } else {
        x_0 = 0;
      }
    } else if (x[0] < x[2]) {
      x_0 = 0;
    } else if (rtb_out_l < x[2]) {
      x_0 = 2;
    } else {
      x_0 = 1;
    }

    rtb_Sum_fl = x[x_0];
    break;
  }

  rtb_Sum_fl += rtb_GainTheta;
  if (rtb_Sum_fl > AutopilotLaws_P.Constant1_Value_i) {
    rtb_Sum_fl = AutopilotLaws_P.Constant1_Value_i;
  } else {
    rtb_Tsxlo = AutopilotLaws_P.Gain1_Gain_h * AutopilotLaws_P.Constant1_Value_i;
    if (rtb_Sum_fl < rtb_Tsxlo) {
      rtb_Sum_fl = rtb_Tsxlo;
    }
  }

  if (AutopilotLaws_U.in.input.enabled == 0.0) {
    AutopilotLaws_DWork.icLoad_k = 1U;
  }

  if (AutopilotLaws_DWork.icLoad_k != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_j = rtb_GainTheta;
  }

  rtb_Minup = rtb_Sum_fl - AutopilotLaws_DWork.Delay_DSTATE_j;
  rtb_ManualSwitch_i = AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_Minup < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Minup;
  }

  rtb_Minup = AutopilotLaws_P.Gain1_Gain_co * AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_ManualSwitch_i > rtb_Minup) {
    rtb_Minup = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_j += rtb_Minup;
  rtb_Sum_nb = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_lz;
  rtb_out_l = rtb_Sum_nb + AutopilotLaws_P.Constant_Value_ok;
  AutopilotLaws_DWork.Delay1_DSTATE_h = 1.0 / rtb_out_l * (AutopilotLaws_P.Constant_Value_ok - rtb_Sum_nb) *
    AutopilotLaws_DWork.Delay1_DSTATE_h + (AutopilotLaws_DWork.Delay_DSTATE_j + AutopilotLaws_DWork.Delay_DSTATE_lb) *
    (rtb_Sum_nb / rtb_out_l);
  rtb_Minup = AutopilotLaws_U.in.input.enabled - AutopilotLaws_DWork.Delay_DSTATE_a;
  rtb_ManualSwitch_i = AutopilotLaws_P.RateLimiterVariableTs_up_i * AutopilotLaws_U.in.time.dt;
  if (rtb_Minup < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Minup;
  }

  rtb_Minup = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo_o;
  if (rtb_ManualSwitch_i > rtb_Minup) {
    rtb_Minup = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_a += rtb_Minup;
  if (AutopilotLaws_DWork.Delay_DSTATE_a > AutopilotLaws_P.Saturation_UpperSat_gz) {
    rtb_Sum_nb = AutopilotLaws_P.Saturation_UpperSat_gz;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_a < AutopilotLaws_P.Saturation_LowerSat_l) {
    rtb_Sum_nb = AutopilotLaws_P.Saturation_LowerSat_l;
  } else {
    rtb_Sum_nb = AutopilotLaws_DWork.Delay_DSTATE_a;
  }

  rtb_Tsxlo = AutopilotLaws_DWork.Delay1_DSTATE_h * rtb_Sum_nb;
  rtb_Sum_nb = AutopilotLaws_P.Constant_Value_kk - rtb_Sum_nb;
  rtb_Sum_nb *= rtb_GainTheta;
  AutopilotLaws_Y.out.output.autopilot.Theta_c_deg = rtb_Tsxlo + rtb_Sum_nb;
  AutopilotLaws_Y.out.time = AutopilotLaws_U.in.time;
  AutopilotLaws_Y.out.data.Theta_deg = rtb_GainTheta;
  AutopilotLaws_Y.out.data.Phi_deg = rtb_GainTheta1;
  AutopilotLaws_Y.out.data.qk_deg_s = result[1];
  AutopilotLaws_Y.out.data.rk_deg_s = result[2];
  AutopilotLaws_Y.out.data.pk_deg_s = result[0];
  AutopilotLaws_Y.out.data.V_ias_kn = AutopilotLaws_U.in.data.V_ias_kn;
  AutopilotLaws_Y.out.data.V_tas_kn = AutopilotLaws_U.in.data.V_tas_kn;
  AutopilotLaws_Y.out.data.V_mach = AutopilotLaws_U.in.data.V_mach;
  AutopilotLaws_Y.out.data.V_gnd_kn = AutopilotLaws_U.in.data.V_gnd_kn;
  AutopilotLaws_Y.out.data.alpha_deg = AutopilotLaws_U.in.data.alpha_deg;
  AutopilotLaws_Y.out.data.H_ft = AutopilotLaws_U.in.data.H_ft;
  AutopilotLaws_Y.out.data.H_ind_ft = AutopilotLaws_U.in.data.H_ind_ft;
  AutopilotLaws_Y.out.data.H_radio_ft = AutopilotLaws_U.in.data.H_radio_ft;
  AutopilotLaws_Y.out.data.H_dot_ft_min = AutopilotLaws_U.in.data.H_dot_ft_min;
  AutopilotLaws_Y.out.data.Psi_magnetic_deg = AutopilotLaws_U.in.data.Psi_magnetic_deg;
  AutopilotLaws_Y.out.data.Psi_magnetic_track_deg = AutopilotLaws_U.in.data.Psi_magnetic_track_deg;
  AutopilotLaws_Y.out.data.Psi_true_deg = AutopilotLaws_U.in.data.Psi_true_deg;
  AutopilotLaws_Y.out.data.bx_m_s2 = AutopilotLaws_U.in.data.bx_m_s2;
  AutopilotLaws_Y.out.data.by_m_s2 = AutopilotLaws_U.in.data.by_m_s2;
  AutopilotLaws_Y.out.data.bz_m_s2 = AutopilotLaws_U.in.data.bz_m_s2;
  AutopilotLaws_Y.out.data.ap_fd_active = AutopilotLaws_U.in.data.ap_fd_active;
  AutopilotLaws_Y.out.data.ap_V_c_kn = AutopilotLaws_U.in.data.ap_V_c_kn;
  AutopilotLaws_Y.out.data.ap_H_c_ft = AutopilotLaws_U.in.data.ap_H_c_ft;
  AutopilotLaws_Y.out.data.ap_Psi_c_deg = AutopilotLaws_U.in.data.ap_Psi_c_deg;
  AutopilotLaws_Y.out.data.ap_H_dot_c_ft_min = AutopilotLaws_U.in.data.ap_H_dot_c_ft_min;
  AutopilotLaws_Y.out.data.ap_FPA_c_deg = AutopilotLaws_U.in.data.ap_FPA_c_deg;
  AutopilotLaws_Y.out.data.nav_valid = AutopilotLaws_U.in.data.nav_valid;
  AutopilotLaws_Y.out.data.nav_loc_deg = AutopilotLaws_U.in.data.nav_loc_deg;
  AutopilotLaws_Y.out.data.nav_radial_error_deg = AutopilotLaws_U.in.data.nav_radial_error_deg;
  AutopilotLaws_Y.out.data.nav_dme_nmi = AutopilotLaws_U.in.data.nav_dme_nmi;
  AutopilotLaws_Y.out.data.nav_gs_error_deg = AutopilotLaws_U.in.data.nav_gs_error_deg;
  AutopilotLaws_Y.out.data.flight_guidance_xtk_nmi = AutopilotLaws_U.in.data.flight_guidance_xtk_nmi;
  AutopilotLaws_Y.out.data.flight_guidance_tae_deg = AutopilotLaws_U.in.data.flight_guidance_tae_deg;
  AutopilotLaws_Y.out.data.flight_phase = AutopilotLaws_U.in.data.flight_phase;
  AutopilotLaws_Y.out.data.V2_kn = AutopilotLaws_U.in.data.V2_kn;
  AutopilotLaws_Y.out.data.is_flight_plan_available = AutopilotLaws_U.in.data.is_flight_plan_available;
  AutopilotLaws_Y.out.data.thrust_reduction_altitude = AutopilotLaws_U.in.data.thrust_reduction_altitude;
  AutopilotLaws_Y.out.data.thrust_reduction_altitude_go_around =
    AutopilotLaws_U.in.data.thrust_reduction_altitude_go_around;
  AutopilotLaws_Y.out.data.on_ground = rtb_on_ground;
  AutopilotLaws_Y.out.data.zeta_deg = rtb_Saturation;
  AutopilotLaws_Y.out.data.throttle_lever_1_pos = AutopilotLaws_U.in.data.throttle_lever_1_pos;
  AutopilotLaws_Y.out.data.throttle_lever_2_pos = AutopilotLaws_U.in.data.throttle_lever_2_pos;
  AutopilotLaws_Y.out.input = AutopilotLaws_U.in.input;
  AutopilotLaws_Y.out.output.ap_on = AutopilotLaws_U.in.input.enabled;
  AutopilotLaws_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_fl;
  AutopilotLaws_DWork.Delay_DSTATE = rtb_Saturation1;
  AutopilotLaws_DWork.Delay_DSTATE_n = rtb_out_d3;
  AutopilotLaws_DWork.Delay_DSTATE_b = rtb_Mod1_pg;
  AutopilotLaws_DWork.icLoad = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_hr = AutopilotLaws_DWork.Delay_DSTATE_hc;
  AutopilotLaws_DWork.Delay_DSTATE_f = rtb_ManualSwitch;
  AutopilotLaws_DWork.Delay_DSTATE_l = rtb_Mod2;
  AutopilotLaws_DWork.Delay_DSTATE_nb = rtb_out_br;
  AutopilotLaws_DWork.Delay_DSTATE_p = rtb_out;
  AutopilotLaws_DWork.Delay_DSTATE_d = rtb_out_m;
  AutopilotLaws_DWork.Delay_DSTATE_fs = rtb_Mod2_d;
  AutopilotLaws_DWork.icLoad_k = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_lb = AutopilotLaws_DWork.Delay_DSTATE_j;
}

void AutopilotLawsModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&AutopilotLaws_B)), 0,
                     sizeof(BlockIO_AutopilotLaws_T));
  (void) std::memset(static_cast<void *>(&AutopilotLaws_DWork), 0,
                     sizeof(D_Work_AutopilotLaws_T));
  AutopilotLaws_U.in = AutopilotLaws_rtZap_laws_input;
  AutopilotLaws_Y.out = AutopilotLaws_rtZap_laws_output;
  AutopilotLaws_DWork.Delay_DSTATE = AutopilotLaws_P.Delay_InitialCondition;
  AutopilotLaws_DWork.Delay1_DSTATE = AutopilotLaws_P.Delay1_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_n = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_b = AutopilotLaws_P.Delay_InitialCondition_l;
  AutopilotLaws_DWork.Delay1_DSTATE_j = AutopilotLaws_P.Delay1_InitialCondition_n;
  AutopilotLaws_DWork.icLoad = 1U;
  AutopilotLaws_DWork.Delay_DSTATE_hr = AutopilotLaws_P.Delay_InitialCondition_i;
  AutopilotLaws_DWork.Delay1_DSTATE_f = AutopilotLaws_P.Delay1_InitialCondition_k;
  AutopilotLaws_DWork.Delay_DSTATE_i = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_f = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_e;
  AutopilotLaws_DWork.Delay_DSTATE_l = AutopilotLaws_P.Delay_InitialCondition_h;
  AutopilotLaws_DWork.Delay1_DSTATE_c = AutopilotLaws_P.Delay1_InitialCondition_c;
  AutopilotLaws_DWork.Delay_DSTATE_nb = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_h;
  AutopilotLaws_DWork.Delay_DSTATE_p = AutopilotLaws_P.Delay_InitialCondition_n;
  AutopilotLaws_DWork.Delay1_DSTATE_je = AutopilotLaws_P.Delay1_InitialCondition_g;
  AutopilotLaws_DWork.Delay_DSTATE_d = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_b;
  AutopilotLaws_DWork.Delay_DSTATE_fs = AutopilotLaws_P.Delay_InitialCondition_j;
  AutopilotLaws_DWork.Delay1_DSTATE_i = AutopilotLaws_P.Delay1_InitialCondition_i;
  AutopilotLaws_DWork.icLoad_k = 1U;
  AutopilotLaws_DWork.Delay_DSTATE_lb = AutopilotLaws_P.Delay_InitialCondition_b;
  AutopilotLaws_DWork.Delay1_DSTATE_h = AutopilotLaws_P.Delay1_InitialCondition_ij;
  AutopilotLaws_DWork.Delay_DSTATE_a = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition_p;
  AutopilotLaws_DWork.is_active_c5_AutopilotLaws = 0U;
  AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD_n;
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_k);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_j);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_f);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_l);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_h);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_b);
  AutopilotLaws_B.u_n = AutopilotLaws_P.Y_Y0;
  AutopilotLaws_B.u = AutopilotLaws_P.Y_Y0_n;
}

void AutopilotLawsModelClass::terminate()
{
}

AutopilotLawsModelClass::AutopilotLawsModelClass()
{
}

AutopilotLawsModelClass::~AutopilotLawsModelClass()
{
}
