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
    0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 } };

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
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  real_T rtb_ManualSwitch;
  real_T rtb_Mod1_pd;
  real_T rtb_Mod2;
  real_T rtb_Mod2_o;
  real_T rtb_Mod1_b;
  real_T rtb_ROLLLIM1;
  real_T rtb_ManualSwitch_i;
  real_T rtb_Sum_ox;
  real_T rtb_out_h;
  real_T rtb_out;
  real_T rtb_out_o;
  real_T rtb_out_j;
  real_T rtb_Sum1_l5;
  real_T tmp[9];
  real_T u0;
  int32_T x_0;
  rtb_GainTheta = AutopilotLaws_P.GainTheta_Gain * AutopilotLaws_U.in.data.Theta_deg;
  rtb_GainTheta1 = AutopilotLaws_P.GainTheta1_Gain * AutopilotLaws_U.in.data.Phi_deg;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta;
  rtb_Mod1_b = 0.017453292519943295 * rtb_GainTheta1;
  rtb_out_j = std::tan(rtb_Saturation1);
  rtb_Saturation = std::sin(rtb_Mod1_b);
  rtb_Mod1_pd = std::cos(rtb_Mod1_b);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation * rtb_out_j;
  tmp[6] = rtb_Mod1_pd * rtb_out_j;
  tmp[1] = 0.0;
  tmp[4] = rtb_Mod1_pd;
  tmp[7] = -rtb_Saturation;
  tmp[2] = 0.0;
  rtb_Mod1_b = 1.0 / std::cos(rtb_Saturation1);
  tmp[5] = rtb_Mod1_b * rtb_Saturation;
  tmp[8] = rtb_Mod1_b * rtb_Mod1_pd;
  rtb_Saturation = AutopilotLaws_P.Gain_Gain_kc * AutopilotLaws_U.in.data.p_rad_s * AutopilotLaws_P.Gainpk_Gain;
  rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_lv * AutopilotLaws_U.in.data.q_rad_s * AutopilotLaws_P.Gainqk_Gain;
  rtb_out_j = AutopilotLaws_P.Gain_Gain_aq * AutopilotLaws_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_out_j + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
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

  rtb_Saturation1 = AutopilotLaws_P.Gain1_Gain_k * AutopilotLaws_U.in.data.gear_strut_compression_2 -
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
  AutopilotLaws_Chart(rtb_Saturation1, AutopilotLaws_P.Gain_Gain_ek * rt_modd(AutopilotLaws_P.Constant3_Value_i -
    rtb_Saturation1, AutopilotLaws_P.Constant3_Value_i), AutopilotLaws_P.Constant2_Value_l, &rtb_out_j,
                      &AutopilotLaws_DWork.sf_Chart_de);
  if (AutopilotLaws_U.in.data.H_radio_ft <= AutopilotLaws_P.CompareToConstant_const) {
    rtb_Saturation1 = AutopilotLaws_P.Gain_Gain_a * rtb_out_j;
  } else {
    rtb_Saturation1 = AutopilotLaws_P.Constant1_Value;
  }

  rtb_Minup = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1;
  rtb_out_j = rtb_Minup + AutopilotLaws_P.Constant_Value_e;
  AutopilotLaws_DWork.Delay1_DSTATE = 1.0 / rtb_out_j * (AutopilotLaws_P.Constant_Value_e - rtb_Minup) *
    AutopilotLaws_DWork.Delay1_DSTATE + (rtb_Saturation1 + AutopilotLaws_DWork.Delay_DSTATE) * (rtb_Minup / rtb_out_j);
  if (AutopilotLaws_P.ManualSwitch_CurrentSetting == 1) {
    rtb_ManualSwitch = AutopilotLaws_P.Constant_Value;
  } else {
    rtb_ManualSwitch = AutopilotLaws_U.in.input.lateral_law;
  }

  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_o) {
    rtb_Mod1_b = AutopilotLaws_P.Saturation_UpperSat_o;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_o) {
    rtb_Mod1_b = AutopilotLaws_P.Saturation_LowerSat_o;
  } else {
    rtb_Mod1_b = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_out_j = std::sin(AutopilotLaws_P.Gain1_Gain_n * AutopilotLaws_U.in.data.nav_radial_error_deg) * rtb_Mod1_b *
    AutopilotLaws_P.Gain2_Gain_g;
  if (rtb_out_j > AutopilotLaws_P.Saturation1_UpperSat_g) {
    rtb_out_j = AutopilotLaws_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_out_j < AutopilotLaws_P.Saturation1_LowerSat_k) {
      rtb_out_j = AutopilotLaws_P.Saturation1_LowerSat_k;
    }
  }

  if (rtb_ManualSwitch != AutopilotLaws_P.CompareToConstant_const_k) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  AutopilotLaws_DWork.Delay_DSTATE_h += AutopilotLaws_P.Gain6_Gain * rtb_out_j *
    AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_Gain * AutopilotLaws_U.in.time.dt;
  if (AutopilotLaws_DWork.Delay_DSTATE_h > AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (AutopilotLaws_DWork.Delay_DSTATE_h < AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      AutopilotLaws_DWork.Delay_DSTATE_h = AutopilotLaws_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  rtb_Mod1_b = AutopilotLaws_U.in.data.nav_radial_error_deg + AutopilotLaws_U.in.data.nav_loc_deg;
  rtb_Mod1_pd = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_deg - (rt_modd(rt_modd(rtb_Mod1_b,
    AutopilotLaws_P.Constant3_Value_f) + AutopilotLaws_P.Constant3_Value_f, AutopilotLaws_P.Constant3_Value_f) +
    AutopilotLaws_P.Constant3_Value_p)) + AutopilotLaws_P.Constant3_Value_p, AutopilotLaws_P.Constant3_Value_p);
  rtb_Mod2 = rt_modd(AutopilotLaws_P.Constant3_Value_p - rtb_Mod1_pd, AutopilotLaws_P.Constant3_Value_p);
  if (rtb_Mod1_pd < rtb_Mod2) {
    rtb_Mod1_pd *= AutopilotLaws_P.Gain1_Gain_d;
  } else {
    rtb_Mod1_pd = AutopilotLaws_P.Gain_Gain_j * rtb_Mod2;
  }

  rtb_out_j = rt_modd((rt_modd(rt_modd(((rtb_out_j * look1_binlxpw(AutopilotLaws_U.in.data.V_gnd_kn,
    AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1, AutopilotLaws_P.ScheduledGain_Table, 2U) +
    AutopilotLaws_DWork.Delay_DSTATE_h) + AutopilotLaws_P.Gain1_Gain_f * rtb_Mod1_pd) +
    AutopilotLaws_U.in.data.Psi_magnetic_deg, AutopilotLaws_P.Constant3_Value_h) + AutopilotLaws_P.Constant3_Value_h,
    AutopilotLaws_P.Constant3_Value_h) - (AutopilotLaws_U.in.data.Psi_magnetic_deg + AutopilotLaws_P.Constant3_Value_e))
                      + AutopilotLaws_P.Constant3_Value_e, AutopilotLaws_P.Constant3_Value_e);
  AutopilotLaws_Chart(rtb_out_j, AutopilotLaws_P.Gain_Gain_ba * rt_modd(AutopilotLaws_P.Constant3_Value_e - rtb_out_j,
    AutopilotLaws_P.Constant3_Value_e), AutopilotLaws_P.Constant1_Value_e, &rtb_Mod1_pd, &AutopilotLaws_DWork.sf_Chart_n);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_out_j = AutopilotLaws_P.beta_Value_ed;
    break;

   case 1:
    rtb_out_j = AutopilotLaws_P.beta_Value_e;
    break;

   case 2:
    rtb_out_j = AutopilotLaws_P.beta_Value_b;
    break;

   case 3:
    rtb_out_j = AutopilotLaws_P.beta_Value_m;
    break;

   case 4:
    rtb_out_j = AutopilotLaws_P.beta_Value;
    break;

   case 5:
    rtb_out_j = AutopilotLaws_DWork.Delay1_DSTATE;
    break;

   default:
    rtb_out_j = (AutopilotLaws_P.Gain5_Gain * rtb_Mod1_pd + AutopilotLaws_P.Gain_Gain_b * result[2]) + rtb_Saturation;
    break;
  }

  rtb_ROLLLIM1 = look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn, AutopilotLaws_P.ROLLLIM1_bp01Data,
    AutopilotLaws_P.ROLLLIM1_tableData, 4U);
  rtb_Mod1_pd = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_deg +
    AutopilotLaws_P.Constant3_Value_m)) + AutopilotLaws_P.Constant3_Value_m, AutopilotLaws_P.Constant3_Value_m);
  AutopilotLaws_Chart(rtb_Mod1_pd, AutopilotLaws_P.Gain_Gain_j2 * rt_modd(AutopilotLaws_P.Constant3_Value_m -
    rtb_Mod1_pd, AutopilotLaws_P.Constant3_Value_m), AutopilotLaws_P.Constant_Value_c, &rtb_out_h,
                      &AutopilotLaws_DWork.sf_Chart_a);
  rtb_Mod1_pd = rt_modd((AutopilotLaws_U.in.input.Psi_c_deg - (AutopilotLaws_U.in.data.Psi_magnetic_track_deg +
    AutopilotLaws_P.Constant3_Value_c)) + AutopilotLaws_P.Constant3_Value_c, AutopilotLaws_P.Constant3_Value_c);
  AutopilotLaws_Chart(rtb_Mod1_pd, AutopilotLaws_P.Gain_Gain_eq * rt_modd(AutopilotLaws_P.Constant3_Value_c -
    rtb_Mod1_pd, AutopilotLaws_P.Constant3_Value_c), AutopilotLaws_P.Constant_Value_i, &rtb_out,
                      &AutopilotLaws_DWork.sf_Chart_p);
  rtb_Mod2 = AutopilotLaws_P.Gain_Gain_n * AutopilotLaws_U.in.data.flight_guidance_xtk_nmi;
  if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_kr) {
    rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_kr;
  } else {
    if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_p) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_p;
    }
  }

  rtb_Mod1_pd = rt_modd((rt_modd(rt_modd((AutopilotLaws_P.Gain2_Gain_f * AutopilotLaws_U.in.data.flight_guidance_tae_deg
    + rtb_Mod2) * AutopilotLaws_P.Gain1_Gain_nh + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_ci) + AutopilotLaws_P.Constant3_Value_ci, AutopilotLaws_P.Constant3_Value_ci) -
    (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_p3)) +
                        AutopilotLaws_P.Constant3_Value_p3, AutopilotLaws_P.Constant3_Value_p3);
  AutopilotLaws_Chart(rtb_Mod1_pd, AutopilotLaws_P.Gain_Gain_ke * rt_modd(AutopilotLaws_P.Constant3_Value_p3 -
    rtb_Mod1_pd, AutopilotLaws_P.Constant3_Value_p3), AutopilotLaws_P.Constant_Value_cw, &rtb_out_o,
                      &AutopilotLaws_DWork.sf_Chart);
  rtb_Mod1_pd = rt_modd((AutopilotLaws_U.in.data.Psi_magnetic_track_deg - (rt_modd(rt_modd(rtb_Mod1_b,
    AutopilotLaws_P.Constant3_Value_d) + AutopilotLaws_P.Constant3_Value_d, AutopilotLaws_P.Constant3_Value_d) +
    AutopilotLaws_P.Constant3_Value_p5)) + AutopilotLaws_P.Constant3_Value_p5, AutopilotLaws_P.Constant3_Value_p5);
  rtb_Mod2_o = rt_modd(AutopilotLaws_P.Constant3_Value_p5 - rtb_Mod1_pd, AutopilotLaws_P.Constant3_Value_p5);
  if (AutopilotLaws_U.in.data.nav_dme_nmi > AutopilotLaws_P.Saturation_UpperSat_m) {
    rtb_Mod1_b = AutopilotLaws_P.Saturation_UpperSat_m;
  } else if (AutopilotLaws_U.in.data.nav_dme_nmi < AutopilotLaws_P.Saturation_LowerSat_kv) {
    rtb_Mod1_b = AutopilotLaws_P.Saturation_LowerSat_kv;
  } else {
    rtb_Mod1_b = AutopilotLaws_U.in.data.nav_dme_nmi;
  }

  rtb_Mod2 = std::sin(AutopilotLaws_P.Gain1_Gain_da * AutopilotLaws_U.in.data.nav_radial_error_deg) * rtb_Mod1_b *
    look1_binlxpw(AutopilotLaws_U.in.data.nav_dme_nmi, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_a,
                  AutopilotLaws_P.ScheduledGain_Table_pq, 4U);
  if (rtb_Mod2 > AutopilotLaws_P.Saturation1_UpperSat_i) {
    rtb_Mod2 = AutopilotLaws_P.Saturation1_UpperSat_i;
  } else {
    if (rtb_Mod2 < AutopilotLaws_P.Saturation1_LowerSat_g) {
      rtb_Mod2 = AutopilotLaws_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Mod1_pd < rtb_Mod2_o) {
    rtb_Mod1_pd *= AutopilotLaws_P.Gain1_Gain;
  } else {
    rtb_Mod1_pd = AutopilotLaws_P.Gain_Gain * rtb_Mod2_o;
  }

  rtb_Mod1_b = rt_modd((rt_modd(rt_modd((rtb_Mod2 + rtb_Mod1_pd) * AutopilotLaws_P.Gain3_Gain +
    AutopilotLaws_U.in.data.Psi_magnetic_track_deg, AutopilotLaws_P.Constant3_Value_pz) +
    AutopilotLaws_P.Constant3_Value_pz, AutopilotLaws_P.Constant3_Value_pz) -
                        (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_j)) +
                       AutopilotLaws_P.Constant3_Value_j, AutopilotLaws_P.Constant3_Value_j);
  AutopilotLaws_Chart(rtb_Mod1_b, AutopilotLaws_P.Gain_Gain_nz * rt_modd(AutopilotLaws_P.Constant3_Value_j - rtb_Mod1_b,
    AutopilotLaws_P.Constant3_Value_j), AutopilotLaws_P.Constant_Value_e5, &rtb_Mod2, &AutopilotLaws_DWork.sf_Chart_d);
  rtb_Mod1_b = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain * AutopilotLaws_U.in.data.nav_radial_error_deg;
  rtb_Mod1_pd = (rtb_Mod1_b - AutopilotLaws_DWork.Delay_DSTATE_l) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_i + AutopilotLaws_U.in.data.nav_radial_error_deg;
  rtb_Mod2_o = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1;
  rtb_Minup = rtb_Mod2_o + AutopilotLaws_P.Constant_Value_o;
  AutopilotLaws_DWork.Delay1_DSTATE_j = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_o - rtb_Mod2_o) *
    AutopilotLaws_DWork.Delay1_DSTATE_j + (rtb_Mod1_pd + AutopilotLaws_DWork.Delay_DSTATE_c) * (rtb_Mod2_o / rtb_Minup);
  rtb_Mod2_o = rt_modd((rt_modd(rt_modd(AutopilotLaws_DWork.Delay1_DSTATE_j * look1_binlxpw
    (AutopilotLaws_U.in.data.H_radio_ft, AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_e,
     AutopilotLaws_P.ScheduledGain_Table_pf, 4U) + AutopilotLaws_U.in.data.Psi_magnetic_track_deg,
    AutopilotLaws_P.Constant3_Value_d4) + AutopilotLaws_P.Constant3_Value_d4, AutopilotLaws_P.Constant3_Value_d4) -
                        (AutopilotLaws_U.in.data.Psi_magnetic_track_deg + AutopilotLaws_P.Constant3_Value_h5)) +
                       AutopilotLaws_P.Constant3_Value_h5, AutopilotLaws_P.Constant3_Value_h5);
  AutopilotLaws_Chart(rtb_Mod2_o, AutopilotLaws_P.Gain_Gain_dt * rt_modd(AutopilotLaws_P.Constant3_Value_h5 - rtb_Mod2_o,
    AutopilotLaws_P.Constant3_Value_h5), AutopilotLaws_P.Constant_Value_p1, &rtb_Minup, &AutopilotLaws_DWork.sf_Chart_h);
  switch (static_cast<int32_T>(rtb_ManualSwitch)) {
   case 0:
    rtb_Mod2_o = rtb_GainTheta1;
    break;

   case 1:
    rtb_Mod2_o = rtb_out_h * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_p, AutopilotLaws_P.ScheduledGain_Table_i, 3U);
    break;

   case 2:
    rtb_Mod2_o = rtb_out * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_k, AutopilotLaws_P.ScheduledGain_Table_h, 3U);
    break;

   case 3:
    rtb_Mod2_o = rtb_out_o * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_b, AutopilotLaws_P.ScheduledGain_Table_p, 3U);
    break;

   case 4:
    rtb_Mod2_o = rtb_Mod2 * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_g, AutopilotLaws_P.ScheduledGain_Table_m, 3U);
    break;

   case 5:
    rtb_Mod2_o = rtb_Minup * look1_binlxpw(AutopilotLaws_U.in.data.V_tas_kn,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_h, AutopilotLaws_P.ScheduledGain_Table_k, 3U);
    break;

   default:
    rtb_Mod2_o = AutopilotLaws_P.Constant3_Value;
    break;
  }

  if (rtb_Mod2_o > rtb_ROLLLIM1) {
    rtb_Mod2_o = rtb_ROLLLIM1;
  } else {
    rtb_Mod2 = AutopilotLaws_P.Gain1_Gain_l * rtb_ROLLLIM1;
    if (rtb_Mod2_o < rtb_Mod2) {
      rtb_Mod2_o = rtb_Mod2;
    }
  }

  if (AutopilotLaws_U.in.input.enabled == 0.0) {
    AutopilotLaws_DWork.icLoad = 1U;
  }

  if (AutopilotLaws_DWork.icLoad != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_hc = rtb_GainTheta1;
  }

  rtb_Mod2 = rtb_Mod2_o - AutopilotLaws_DWork.Delay_DSTATE_hc;
  rtb_ManualSwitch_i = AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_Mod2 < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Mod2;
  }

  rtb_Mod2 = AutopilotLaws_P.Gain1_Gain_kf * AutopilotLaws_P.Constant2_Value_h * AutopilotLaws_U.in.time.dt;
  if (rtb_ManualSwitch_i > rtb_Mod2) {
    rtb_Mod2 = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_hc += rtb_Mod2;
  rtb_Mod2 = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_l;
  rtb_Minup = rtb_Mod2 + AutopilotLaws_P.Constant_Value_ez;
  AutopilotLaws_DWork.Delay1_DSTATE_a = 1.0 / rtb_Minup * (AutopilotLaws_P.Constant_Value_ez - rtb_Mod2) *
    AutopilotLaws_DWork.Delay1_DSTATE_a + (AutopilotLaws_DWork.Delay_DSTATE_hc + AutopilotLaws_DWork.Delay_DSTATE_k) *
    (rtb_Mod2 / rtb_Minup);
  rtb_Mod2 = AutopilotLaws_U.in.input.enabled - AutopilotLaws_DWork.Delay_DSTATE_f;
  rtb_ManualSwitch_i = AutopilotLaws_P.RateLimiterVariableTs_up * AutopilotLaws_U.in.time.dt;
  if (rtb_Mod2 < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Mod2;
  }

  rtb_Mod2 = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo;
  if (rtb_ManualSwitch_i > rtb_Mod2) {
    rtb_Mod2 = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_f += rtb_Mod2;
  if (AutopilotLaws_DWork.Delay_DSTATE_f > AutopilotLaws_P.Saturation_UpperSat_p) {
    rtb_Sum_ox = AutopilotLaws_P.Saturation_UpperSat_p;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_f < AutopilotLaws_P.Saturation_LowerSat_n) {
    rtb_Sum_ox = AutopilotLaws_P.Saturation_LowerSat_n;
  } else {
    rtb_Sum_ox = AutopilotLaws_DWork.Delay_DSTATE_f;
  }

  rtb_Mod2 = AutopilotLaws_DWork.Delay1_DSTATE_a * rtb_Sum_ox;
  rtb_Sum_ox = AutopilotLaws_P.Constant_Value_l - rtb_Sum_ox;
  rtb_Sum_ox *= rtb_GainTheta1;
  AutopilotLaws_Y.out.output.autopilot.Phi_c_deg = rtb_Mod2 + rtb_Sum_ox;
  AutopilotLaws_Y.out.output.flight_director.Beta_c_deg = rtb_out_j;
  AutopilotLaws_Y.out.output.autopilot.Beta_c_deg = rtb_out_j;
  AutopilotLaws_Y.out.output.flight_director.Phi_c_deg = rtb_Mod2_o;
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
  rtb_ROLLLIM1 = (rtb_ManualSwitch - AutopilotLaws_DWork.Delay_DSTATE_kp) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_f + AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Sum_ox = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1_a;
  rtb_Mod2 = rtb_Sum_ox + AutopilotLaws_P.Constant_Value_n;
  AutopilotLaws_DWork.Delay1_DSTATE_d = 1.0 / rtb_Mod2 * (AutopilotLaws_P.Constant_Value_n - rtb_Sum_ox) *
    AutopilotLaws_DWork.Delay1_DSTATE_d + (rtb_ROLLLIM1 + AutopilotLaws_DWork.Delay_DSTATE_b) * (rtb_Sum_ox / rtb_Mod2);
  rtb_Mod2 = AutopilotLaws_DWork.Delay1_DSTATE_d - AutopilotLaws_U.in.data.ap_V_c_kn;
  if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_h) {
    rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_h;
  } else {
    if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_ce) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_ce;
    }
  }

  rtb_Sum_ox = AutopilotLaws_P.Gain1_Gain_b * rtb_Mod2;
  rtb_Mod2_o = AutopilotLaws_P.Gain1_Gain_a * AutopilotLaws_U.in.data.alpha_deg;
  rtb_out_j = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_Mod2_o);
  rtb_Mod2_o = std::cos(rtb_Mod2_o);
  rtb_Mod2_o *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_Mod2_o = (rtb_out_j + rtb_Mod2_o) * AutopilotLaws_P.Gain_Gain_o * AutopilotLaws_P.Gain_Gain_g1;
  rtb_Mod2 = AutopilotLaws_P.Gain1_Gain_k0 * std::abs(AutopilotLaws_P.Constant_Value_em) + std::abs(rtb_Mod2_o);
  if (rtb_Mod2 <= AutopilotLaws_P.Constant1_Value_ef) {
    rtb_Mod2 = AutopilotLaws_P.Constant1_Value_ef;
  }

  rtb_out_j = std::abs(rtb_Sum_ox);
  if (rtb_out_j < rtb_Mod2) {
    rtb_Mod2 = rtb_out_j;
  }

  if (rtb_Mod2_o < 0.0) {
    rtb_Mod2_o = -1.0;
  } else {
    if (rtb_Mod2_o > 0.0) {
      rtb_Mod2_o = 1.0;
    }
  }

  rtb_Sum1_l5 = rtb_Sum_ox - rtb_Mod2 * rtb_Mod2_o * AutopilotLaws_P.Gain_Gain_h;
  rtb_out_h = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_l * AutopilotLaws_U.in.data.nav_gs_error_deg;
  rtb_out = (rtb_out_h - AutopilotLaws_DWork.Delay_DSTATE_lp) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_o + AutopilotLaws_P.Gain1_Gain_jl * AutopilotLaws_U.in.data.nav_gs_error_deg;
  rtb_Sum_ox = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_n;
  rtb_Mod2 = rtb_Sum_ox + AutopilotLaws_P.Constant_Value_f;
  AutopilotLaws_DWork.Delay1_DSTATE_d3 = 1.0 / rtb_Mod2 * (AutopilotLaws_P.Constant_Value_f - rtb_Sum_ox) *
    AutopilotLaws_DWork.Delay1_DSTATE_d3 + (rtb_out + AutopilotLaws_DWork.Delay_DSTATE_fd) * (rtb_Sum_ox / rtb_Mod2);
  if (AutopilotLaws_U.in.data.H_radio_ft <= AutopilotLaws_P.CompareToConstant_const_j) {
    AutopilotLaws_B.u = AutopilotLaws_U.in.data.H_dot_ft_min;
  }

  rtb_out_o = AutopilotLaws_P.DiscreteDerivativeVariableTs_Gain_lf * AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Minup = (rtb_out_o - AutopilotLaws_DWork.Delay_DSTATE_by) / AutopilotLaws_U.in.time.dt *
    AutopilotLaws_P.Gain3_Gain_fd + AutopilotLaws_U.in.data.V_ias_kn;
  rtb_Sum_ox = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter1_C1_p;
  rtb_Mod2 = rtb_Sum_ox + AutopilotLaws_P.Constant_Value_is;
  AutopilotLaws_DWork.Delay1_DSTATE_ju = 1.0 / rtb_Mod2 * (AutopilotLaws_P.Constant_Value_is - rtb_Sum_ox) *
    AutopilotLaws_DWork.Delay1_DSTATE_ju + (rtb_Minup + AutopilotLaws_DWork.Delay_DSTATE_ko) * (rtb_Sum_ox / rtb_Mod2);
  rtb_Mod2 = AutopilotLaws_DWork.Delay1_DSTATE_ju - (AutopilotLaws_U.in.data.V2_kn + AutopilotLaws_P.Bias_Bias);
  if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_nk) {
    rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_nk;
  } else {
    if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_h) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_h;
    }
  }

  rtb_Sum_ox = AutopilotLaws_P.Gain1_Gain_p * rtb_Mod2;
  rtb_Mod2_o = AutopilotLaws_P.Gain1_Gain_g * AutopilotLaws_U.in.data.alpha_deg;
  rtb_out_j = AutopilotLaws_U.in.data.bz_m_s2 * std::sin(rtb_Mod2_o);
  rtb_Mod2_o = std::cos(rtb_Mod2_o);
  rtb_Mod2_o *= AutopilotLaws_U.in.data.bx_m_s2;
  rtb_Mod2_o = (rtb_out_j + rtb_Mod2_o) * AutopilotLaws_P.Gain_Gain_fq * AutopilotLaws_P.Gain_Gain_ew;
  rtb_Mod2 = std::abs(rtb_Sum_ox);
  rtb_out_j = AutopilotLaws_P.Gain1_Gain_k4 * std::abs(AutopilotLaws_P.Constant_Value_j) + std::abs(rtb_Mod2_o);
  if (rtb_out_j <= AutopilotLaws_P.Constant1_Value_l) {
    rtb_out_j = AutopilotLaws_P.Constant1_Value_l;
  }

  u0 = AutopilotLaws_P.kntoms_Gain_f * AutopilotLaws_U.in.data.V_tas_kn;
  if (u0 > AutopilotLaws_P.Saturation_UpperSat_a) {
    u0 = AutopilotLaws_P.Saturation_UpperSat_a;
  } else {
    if (u0 < AutopilotLaws_P.Saturation_LowerSat_f) {
      u0 = AutopilotLaws_P.Saturation_LowerSat_f;
    }
  }

  u0 = (AutopilotLaws_P.Constant_Value_in - AutopilotLaws_U.in.data.H_dot_ft_min) * AutopilotLaws_P.ftmintoms_Gain_k /
    u0;
  x[0] = AutopilotLaws_P.Constant1_Value_d - rtb_GainTheta;
  if (rtb_Mod2 < rtb_out_j) {
    rtb_out_j = rtb_Mod2;
  }

  if (rtb_Mod2_o < 0.0) {
    rtb_Mod2_o = -1.0;
  } else {
    if (rtb_Mod2_o > 0.0) {
      rtb_Mod2_o = 1.0;
    }
  }

  x[1] = rtb_Sum_ox - rtb_out_j * rtb_Mod2_o * AutopilotLaws_P.Gain_Gain_hk;
  if (u0 > 1.0) {
    u0 = 1.0;
  } else {
    if (u0 < -1.0) {
      u0 = -1.0;
    }
  }

  x[2] = AutopilotLaws_P.Gain_Gain_db * std::asin(u0) * AutopilotLaws_P.Gain_Gain_gc;
  switch (static_cast<int32_T>(rtb_ManualSwitch_i)) {
   case 0:
    rtb_Sum_ox = rtb_GainTheta;
    break;

   case 1:
    rtb_Mod2 = (AutopilotLaws_B.u_n - AutopilotLaws_U.in.data.H_ft) * AutopilotLaws_P.Gain_Gain_f;
    rtb_out_j = AutopilotLaws_P.kntoms_Gain_n * AutopilotLaws_U.in.data.V_tas_kn;
    if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_n) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_n;
    } else {
      if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_d) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_d;
      }
    }

    if (rtb_out_j > AutopilotLaws_P.Saturation_UpperSat_g) {
      rtb_out_j = AutopilotLaws_P.Saturation_UpperSat_g;
    } else {
      if (rtb_out_j < AutopilotLaws_P.Saturation_LowerSat_g) {
        rtb_out_j = AutopilotLaws_P.Saturation_LowerSat_g;
      }
    }

    rtb_Mod2 = (rtb_Mod2 - AutopilotLaws_U.in.data.H_dot_ft_min) * AutopilotLaws_P.ftmintoms_Gain_j / rtb_out_j;
    if (rtb_Mod2 > 1.0) {
      rtb_Mod2 = 1.0;
    } else {
      if (rtb_Mod2 < -1.0) {
        rtb_Mod2 = -1.0;
      }
    }

    rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_k * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_pm;
    break;

   case 2:
    rtb_out_j = AutopilotLaws_U.in.input.H_c_ft - AutopilotLaws_U.in.data.H_ind_ft;
    rtb_Mod2 = AutopilotLaws_P.kntoms_Gain_e * AutopilotLaws_U.in.data.V_tas_kn;
    if (rtb_out_j < 0.0) {
      rtb_Mod2_o = -1.0;
    } else if (rtb_out_j > 0.0) {
      rtb_Mod2_o = 1.0;
    } else {
      rtb_Mod2_o = rtb_out_j;
    }

    if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_d) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_d;
    } else {
      if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_c) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_c;
      }
    }

    rtb_Mod2 = ((AutopilotLaws_P.Constant_Value_b * rtb_Mod2_o + rtb_out_j) * AutopilotLaws_P.Gain_Gain_e -
                AutopilotLaws_U.in.data.H_dot_ft_min) * AutopilotLaws_P.ftmintoms_Gain_i / rtb_Mod2;
    if (rtb_Mod2 > 1.0) {
      rtb_Mod2 = 1.0;
    } else {
      if (rtb_Mod2 < -1.0) {
        rtb_Mod2 = -1.0;
      }
    }

    rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_g * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_bx;
    break;

   case 3:
    if (AutopilotLaws_U.in.input.H_c_ft - AutopilotLaws_U.in.data.H_ind_ft > AutopilotLaws_P.Switch_Threshold_k) {
      rtb_Mod2 = AutopilotLaws_P.kntoms_Gain_i * AutopilotLaws_U.in.data.V_tas_kn;
      if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_bt) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_bt;
      } else {
        if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_j) {
          rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_j;
        }
      }

      rtb_Mod2 = (AutopilotLaws_P.Constant_Value_k - AutopilotLaws_U.in.data.H_dot_ft_min) *
        AutopilotLaws_P.ftmintoms_Gain_l / rtb_Mod2;
      if (rtb_Mod2 > 1.0) {
        rtb_Mod2 = 1.0;
      } else {
        if (rtb_Mod2 < -1.0) {
          rtb_Mod2 = -1.0;
        }
      }

      rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_bs * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_ab;
      if (rtb_Sum1_l5 > rtb_Sum_ox) {
        rtb_Sum_ox = rtb_Sum1_l5;
      }
    } else {
      rtb_Mod2 = AutopilotLaws_P.kntoms_Gain_i * AutopilotLaws_U.in.data.V_tas_kn;
      if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_bt) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_bt;
      } else {
        if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_j) {
          rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_j;
        }
      }

      rtb_Mod2 = (AutopilotLaws_P.Constant_Value_k - AutopilotLaws_U.in.data.H_dot_ft_min) *
        AutopilotLaws_P.ftmintoms_Gain_l / rtb_Mod2;
      if (rtb_Mod2 > 1.0) {
        rtb_Mod2 = 1.0;
      } else {
        if (rtb_Mod2 < -1.0) {
          rtb_Mod2 = -1.0;
        }
      }

      rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_bs * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_ab;
      if (rtb_Sum1_l5 < rtb_Sum_ox) {
        rtb_Sum_ox = rtb_Sum1_l5;
      }
    }
    break;

   case 4:
    rtb_Mod2 = AutopilotLaws_P.kntoms_Gain_a * AutopilotLaws_U.in.data.V_tas_kn;
    if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat_b) {
      rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_b;
    } else {
      if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat_k) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_k;
      }
    }

    rtb_Mod2 = (AutopilotLaws_U.in.input.H_dot_c_fpm - AutopilotLaws_U.in.data.H_dot_ft_min) *
      AutopilotLaws_P.ftmintoms_Gain_c / rtb_Mod2;
    if (rtb_Mod2 > 1.0) {
      rtb_Mod2 = 1.0;
    } else {
      if (rtb_Mod2 < -1.0) {
        rtb_Mod2 = -1.0;
      }
    }

    rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_j5 * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_d;
    break;

   case 5:
    rtb_Sum_ox = (AutopilotLaws_U.in.input.FPA_c_deg - (rtb_GainTheta - std::cos(AutopilotLaws_P.Gain1_Gain_j *
      rtb_GainTheta1) * AutopilotLaws_U.in.data.alpha_deg)) * AutopilotLaws_P.Gain_Gain_c;
    break;

   case 6:
    rtb_Sum_ox = AutopilotLaws_DWork.Delay1_DSTATE_d3 * look1_binlxpw(AutopilotLaws_U.in.data.H_radio_ft,
      AutopilotLaws_P.ScheduledGain_BreakpointsForDimension1_hy, AutopilotLaws_P.ScheduledGain_Table_ir, 4U);
    break;

   case 7:
    if (rtb_on_ground > AutopilotLaws_P.Switch_Threshold) {
      rtb_Sum_ox = (rtb_GainTheta - AutopilotLaws_P.Constant2_Value) * AutopilotLaws_P.Gain4_Gain;
    } else {
      rtb_ManualSwitch_i = AutopilotLaws_B.u / AutopilotLaws_P.Constant_Value_p * AutopilotLaws_U.in.data.H_radio_ft;
      rtb_Mod2 = AutopilotLaws_P.kntoms_Gain * AutopilotLaws_U.in.data.V_tas_kn;
      if (AutopilotLaws_P.Constant3_Value_n < rtb_ManualSwitch_i) {
        rtb_ManualSwitch_i = AutopilotLaws_P.Constant3_Value_n;
      }

      if (rtb_Mod2 > AutopilotLaws_P.Saturation_UpperSat) {
        rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat;
      } else {
        if (rtb_Mod2 < AutopilotLaws_P.Saturation_LowerSat) {
          rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat;
        }
      }

      rtb_Mod2 = (rtb_ManualSwitch_i - AutopilotLaws_U.in.data.H_dot_ft_min) * AutopilotLaws_P.ftmintoms_Gain / rtb_Mod2;
      if (rtb_Mod2 > 1.0) {
        rtb_Mod2 = 1.0;
      } else {
        if (rtb_Mod2 < -1.0) {
          rtb_Mod2 = -1.0;
        }
      }

      rtb_Sum_ox = AutopilotLaws_P.Gain_Gain_l * std::asin(rtb_Mod2) * AutopilotLaws_P.Gain_Gain_p;
    }
    break;

   default:
    if (x[0] < x[1]) {
      if (x[1] < x[2]) {
        x_0 = 1;
      } else if (x[0] < x[2]) {
        x_0 = 2;
      } else {
        x_0 = 0;
      }
    } else if (x[0] < x[2]) {
      x_0 = 0;
    } else if (x[1] < x[2]) {
      x_0 = 2;
    } else {
      x_0 = 1;
    }

    rtb_Sum_ox = x[x_0];
    break;
  }

  rtb_Sum_ox += rtb_GainTheta;
  if (rtb_Sum_ox > AutopilotLaws_P.Constant1_Value_i) {
    rtb_Sum_ox = AutopilotLaws_P.Constant1_Value_i;
  } else {
    rtb_out_j = AutopilotLaws_P.Gain1_Gain_d0 * AutopilotLaws_P.Constant1_Value_i;
    if (rtb_Sum_ox < rtb_out_j) {
      rtb_Sum_ox = rtb_out_j;
    }
  }

  if (AutopilotLaws_U.in.input.enabled == 0.0) {
    AutopilotLaws_DWork.icLoad_j = 1U;
  }

  if (AutopilotLaws_DWork.icLoad_j != 0) {
    AutopilotLaws_DWork.Delay_DSTATE_e = rtb_GainTheta;
  }

  rtb_Mod2 = rtb_Sum_ox - AutopilotLaws_DWork.Delay_DSTATE_e;
  rtb_ManualSwitch_i = AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_Mod2 < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Mod2;
  }

  rtb_Mod2 = AutopilotLaws_P.Gain1_Gain_gs * AutopilotLaws_P.Constant2_Value_h1 * AutopilotLaws_U.in.time.dt;
  if (rtb_ManualSwitch_i > rtb_Mod2) {
    rtb_Mod2 = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_e += rtb_Mod2;
  rtb_Mod2 = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.LagFilter_C1_o;
  rtb_Mod2_o = rtb_Mod2 + AutopilotLaws_P.Constant_Value_kr;
  AutopilotLaws_DWork.Delay1_DSTATE_f = 1.0 / rtb_Mod2_o * (AutopilotLaws_P.Constant_Value_kr - rtb_Mod2) *
    AutopilotLaws_DWork.Delay1_DSTATE_f + (AutopilotLaws_DWork.Delay_DSTATE_e + AutopilotLaws_DWork.Delay_DSTATE_n) *
    (rtb_Mod2 / rtb_Mod2_o);
  rtb_Mod2 = AutopilotLaws_U.in.input.enabled - AutopilotLaws_DWork.Delay_DSTATE_j;
  rtb_ManualSwitch_i = AutopilotLaws_P.RateLimiterVariableTs_up_i * AutopilotLaws_U.in.time.dt;
  if (rtb_Mod2 < rtb_ManualSwitch_i) {
    rtb_ManualSwitch_i = rtb_Mod2;
  }

  rtb_Mod2 = AutopilotLaws_U.in.time.dt * AutopilotLaws_P.RateLimiterVariableTs_lo_o;
  if (rtb_ManualSwitch_i > rtb_Mod2) {
    rtb_Mod2 = rtb_ManualSwitch_i;
  }

  AutopilotLaws_DWork.Delay_DSTATE_j += rtb_Mod2;
  if (AutopilotLaws_DWork.Delay_DSTATE_j > AutopilotLaws_P.Saturation_UpperSat_n2) {
    rtb_Mod2 = AutopilotLaws_P.Saturation_UpperSat_n2;
  } else if (AutopilotLaws_DWork.Delay_DSTATE_j < AutopilotLaws_P.Saturation_LowerSat_jb) {
    rtb_Mod2 = AutopilotLaws_P.Saturation_LowerSat_jb;
  } else {
    rtb_Mod2 = AutopilotLaws_DWork.Delay_DSTATE_j;
  }

  rtb_out_j = AutopilotLaws_DWork.Delay1_DSTATE_f * rtb_Mod2;
  rtb_Mod2 = AutopilotLaws_P.Constant_Value_fx - rtb_Mod2;
  rtb_Mod2 *= rtb_GainTheta;
  AutopilotLaws_Y.out.output.autopilot.Theta_c_deg = rtb_out_j + rtb_Mod2;
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
  AutopilotLaws_Y.out.data.flaps_handle_index = AutopilotLaws_U.in.data.flaps_handle_index;
  AutopilotLaws_Y.out.input = AutopilotLaws_U.in.input;
  AutopilotLaws_Y.out.output.ap_on = AutopilotLaws_U.in.input.enabled;
  AutopilotLaws_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_ox;
  AutopilotLaws_DWork.Delay_DSTATE = rtb_Saturation1;
  AutopilotLaws_DWork.Delay_DSTATE_l = rtb_Mod1_b;
  AutopilotLaws_DWork.Delay_DSTATE_c = rtb_Mod1_pd;
  AutopilotLaws_DWork.icLoad = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_k = AutopilotLaws_DWork.Delay_DSTATE_hc;
  AutopilotLaws_DWork.Delay_DSTATE_kp = rtb_ManualSwitch;
  AutopilotLaws_DWork.Delay_DSTATE_b = rtb_ROLLLIM1;
  AutopilotLaws_DWork.Delay_DSTATE_lp = rtb_out_h;
  AutopilotLaws_DWork.Delay_DSTATE_fd = rtb_out;
  AutopilotLaws_DWork.Delay_DSTATE_by = rtb_out_o;
  AutopilotLaws_DWork.Delay_DSTATE_ko = rtb_Minup;
  AutopilotLaws_DWork.icLoad_j = 0U;
  AutopilotLaws_DWork.Delay_DSTATE_n = AutopilotLaws_DWork.Delay_DSTATE_e;
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
  AutopilotLaws_DWork.Delay_DSTATE_l = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_c = AutopilotLaws_P.Delay_InitialCondition_p;
  AutopilotLaws_DWork.Delay1_DSTATE_j = AutopilotLaws_P.Delay1_InitialCondition_j;
  AutopilotLaws_DWork.icLoad = 1U;
  AutopilotLaws_DWork.Delay_DSTATE_k = AutopilotLaws_P.Delay_InitialCondition_j;
  AutopilotLaws_DWork.Delay1_DSTATE_a = AutopilotLaws_P.Delay1_InitialCondition_k;
  AutopilotLaws_DWork.Delay_DSTATE_f = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition;
  AutopilotLaws_DWork.Delay_DSTATE_kp = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_e;
  AutopilotLaws_DWork.Delay_DSTATE_b = AutopilotLaws_P.Delay_InitialCondition_d;
  AutopilotLaws_DWork.Delay1_DSTATE_d = AutopilotLaws_P.Delay1_InitialCondition_p;
  AutopilotLaws_DWork.Delay_DSTATE_lp = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_h;
  AutopilotLaws_DWork.Delay_DSTATE_fd = AutopilotLaws_P.Delay_InitialCondition_b;
  AutopilotLaws_DWork.Delay1_DSTATE_d3 = AutopilotLaws_P.Delay1_InitialCondition_jb;
  AutopilotLaws_DWork.Delay_DSTATE_by = AutopilotLaws_P.DiscreteDerivativeVariableTs_InitialCondition_b;
  AutopilotLaws_DWork.Delay_DSTATE_ko = AutopilotLaws_P.Delay_InitialCondition_c;
  AutopilotLaws_DWork.Delay1_DSTATE_ju = AutopilotLaws_P.Delay1_InitialCondition_f;
  AutopilotLaws_DWork.icLoad_j = 1U;
  AutopilotLaws_DWork.Delay_DSTATE_n = AutopilotLaws_P.Delay_InitialCondition_a;
  AutopilotLaws_DWork.Delay1_DSTATE_f = AutopilotLaws_P.Delay1_InitialCondition_c;
  AutopilotLaws_DWork.Delay_DSTATE_j = AutopilotLaws_P.RateLimiterVariableTs_InitialCondition_p;
  AutopilotLaws_DWork.is_active_c5_AutopilotLaws = 0U;
  AutopilotLaws_DWork.is_c5_AutopilotLaws = AutopilotLaws_IN_NO_ACTIVE_CHILD_n;
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_de);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_n);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_a);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_p);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_d);
  AutopilotLaws_Chart_Init(&AutopilotLaws_DWork.sf_Chart_h);
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
