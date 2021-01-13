#include "AutopilotStateMachine.h"
#include "AutopilotStateMachine_private.h"
#include "Double2MultiWord.h"
#include "MultiWordIor.h"
#include "mod_tnBo173x.h"
#include "uMultiWord2Double.h"

const uint8_T AutopilotStateMachine_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T AutopilotStateMachine_IN_OFF = 1U;
const uint8_T AutopilotStateMachine_IN_ON = 2U;
const uint8_T AutopilotStateMachine_IN_FLARE = 1U;
const uint8_T AutopilotStateMachine_IN_GA_TRK = 1U;
const uint8_T AutopilotStateMachine_IN_HDG = 2U;
const uint8_T AutopilotStateMachine_IN_LAND = 2U;
const uint8_T AutopilotStateMachine_IN_LOC = 3U;
const uint8_T AutopilotStateMachine_IN_LOC_CPT = 3U;
const uint8_T AutopilotStateMachine_IN_LOC_TRACK = 4U;
const uint8_T AutopilotStateMachine_IN_NAV = 4U;
const uint8_T AutopilotStateMachine_IN_ROLL_OUT = 5U;
const uint8_T AutopilotStateMachine_IN_RWY = 5U;
const uint8_T AutopilotStateMachine_IN_RWY_TRK = 6U;
const uint8_T AutopilotStateMachine_IN_InAir = 1U;
const uint8_T AutopilotStateMachine_IN_OnGround = 2U;
const uint8_T AutopilotStateMachine_IN_ALT = 1U;
const uint8_T AutopilotStateMachine_IN_ALT_CPT = 2U;
const uint8_T AutopilotStateMachine_IN_ALT_CST = 3U;
const uint8_T AutopilotStateMachine_IN_ALT_CST_CPT = 4U;
const uint8_T AutopilotStateMachine_IN_CLB = 5U;
const uint8_T AutopilotStateMachine_IN_DES = 6U;
const uint8_T AutopilotStateMachine_IN_GS = 7U;
const uint8_T AutopilotStateMachine_IN_GS_CPT = 2U;
const uint8_T AutopilotStateMachine_IN_GS_TRACK = 3U;
const uint8_T AutopilotStateMachine_IN_LAND_k = 4U;
const uint8_T AutopilotStateMachine_IN_OP_CLB = 8U;
const uint8_T AutopilotStateMachine_IN_OP_DES = 9U;
const uint8_T AutopilotStateMachine_IN_SRS = 10U;
const uint8_T AutopilotStateMachine_IN_VS = 11U;
const ap_sm_output AutopilotStateMachine_rtZap_sm_output = {
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
    0.0
  },

  {
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    {
      false,
      false,
      false,
      false,
      0.0
    },

    {
      false,
      false
    },

    {
      false,
      false,
      false,
      false,
      false,
      false,
      false
    },

    {
      lateral_mode_NONE,
      lateral_law_NONE,
      0.0
    }
  },

  {
    {
      false,
      false,
      false,
      false,
      0.0
    },

    {
      false,
      false
    },

    {
      false,
      false,
      false,
      false,
      false,
      false,
      false
    },

    {
      lateral_mode_NONE,
      lateral_law_NONE,
      0.0
    }
  },

  {
    {
      false,
      false,
      false,
      false,
      false,
      false,
      0.0,
      0.0,
      0.0
    },

    {
      false,
      false,
      false,
      false,
      false
    },

    {
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false
    },

    {
      vertical_mode_NONE,
      athr_mode_NONE,
      vertical_law_NONE,
      0.0,
      0.0,
      0.0
    }
  },

  {
    {
      false,
      false,
      false,
      false,
      false,
      false,
      0.0,
      0.0,
      0.0
    },

    {
      false,
      false,
      false,
      false,
      false
    },

    {
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false,
      false
    },

    {
      vertical_mode_NONE,
      athr_mode_NONE,
      vertical_law_NONE,
      0.0,
      0.0,
      0.0
    }
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
  }
} ;

const ap_sm_input AutopilotStateMachine_rtZap_sm_input = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { false, false, false, false, false, false,
    false, false, false, false, false, 0.0, 0.0, 0.0, 0.0 } };

void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift1(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 1);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_OFF(const ap_sm_output *BusAssignment1)
{
  return (BusAssignment1->input.LOC_push && (!BusAssignment1->vertical_previous.armed.GS)) ||
    (BusAssignment1->input.APPR_push && BusAssignment1->vertical_previous.armed.GS) ||
    BusAssignment1->lateral_previous.armed.NAV || (BusAssignment1->lateral_previous.output.mode == lateral_mode_LOC_CPT)
    || (BusAssignment1->lateral_previous.output.mode == lateral_mode_LOC_TRACK) ||
    (BusAssignment1->data.throttle_lever_1_pos == 100.0) || (BusAssignment1->data.throttle_lever_1_pos == 100.0);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_ON(const ap_sm_output *BusAssignment1)
{
  return (BusAssignment1->data.H_radio_ft > 400.0) && BusAssignment1->data.nav_valid &&
    (BusAssignment1->data.throttle_lever_1_pos < 100.0) && (BusAssignment1->input.LOC_push ||
    BusAssignment1->input.APPR_push) && (BusAssignment1->lateral_previous.output.mode != lateral_mode_LOC_CPT) &&
    (BusAssignment1->lateral_previous.output.mode != lateral_mode_LOC_TRACK) &&
    (BusAssignment1->lateral_previous.output.mode != lateral_mode_LAND) && (BusAssignment1->lateral_previous.output.mode
    != lateral_mode_FLARE);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_X_TO_OFF(const ap_sm_output *BusAssignment)
{
  return (!BusAssignment->data.ap_fd_active) || (BusAssignment->data.flight_phase >= 9.0) ||
    (BusAssignment->data.flight_phase <= 2.0);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_HDG(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.input.HDG_pull;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_NAV(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.condition.NAV && (BusAssignment->lateral.armed.NAV ||
    BusAssignment->lateral.input.HDG_push);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_NAV_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_NAV;
  AutopilotStateMachine_B.out_g.law = lateral_law_HPATH;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_HDG_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_HDG;
  AutopilotStateMachine_B.out_g.law = lateral_law_HDG;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_ON_TO_LOC(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.armed.LOC && BusAssignment->lateral.condition.LOC_CPT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_HDG_during(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->lateral.input.Psi_fcu_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LOC_CPT_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LOC_CPT;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_CPT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_NONE;
  AutopilotStateMachine_B.out_g.law = lateral_law_NONE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ROLL_OUT_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_ROLL_OUT;
  AutopilotStateMachine_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_FLARE_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_FLARE;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LOC_TRACK_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LOC_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LAND_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_LAND;
  AutopilotStateMachine_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GA_TRK_entry(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_GA_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->lateral.input.Psi_fcu_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LOC(const ap_sm_output *BusAssignment)
{
  boolean_T guard1 = false;
  if (BusAssignment->lateral.condition.GA_TRACK) {
    AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_GA_TRK;
    AutopilotStateMachine_GA_TRK_entry(BusAssignment);
  } else if (AutopilotStateMachine_ON_TO_HDG(BusAssignment)) {
    AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
    AutopilotStateMachine_HDG_entry();
  } else if (AutopilotStateMachine_ON_TO_NAV(BusAssignment)) {
    AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
    AutopilotStateMachine_NAV_entry();
  } else {
    guard1 = false;
    switch (AutopilotStateMachine_DWork.is_LOC) {
     case AutopilotStateMachine_IN_FLARE:
      if (BusAssignment->lateral.condition.ROLL_OUT) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_ROLL_OUT;
        AutopilotStateMachine_ROLL_OUT_entry();
      }
      break;

     case AutopilotStateMachine_IN_LAND:
      if (BusAssignment->lateral.condition.FLARE) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_FLARE;
        AutopilotStateMachine_FLARE_entry();
      } else {
        if (!BusAssignment->lateral.condition.LAND) {
          if (BusAssignment->data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry();
          } else {
            if (BusAssignment->data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry();
            }
          }
        }
      }
      break;

     case AutopilotStateMachine_IN_LOC_CPT:
      if (BusAssignment->lateral.input.LOC_push || BusAssignment->lateral.input.APPR_push ||
          (!BusAssignment->lateral.condition.LOC_CPT)) {
        if (BusAssignment->data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else if (BusAssignment->data.on_ground != 0.0) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
          AutopilotStateMachine_OFF_entry();
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;

     case AutopilotStateMachine_IN_LOC_TRACK:
      if (BusAssignment->lateral.condition.LAND) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LAND;
        AutopilotStateMachine_LAND_entry();
      } else {
        if (BusAssignment->lateral.input.LOC_push || BusAssignment->lateral.input.APPR_push ||
            (!BusAssignment->lateral.condition.LOC_TRACK)) {
          if (BusAssignment->data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
            AutopilotStateMachine_HDG_entry();
          } else {
            if (BusAssignment->data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry();
            }
          }
        }
      }
      break;

     default:
      if (!BusAssignment->lateral.condition.ROLL_OUT) {
        if (BusAssignment->data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else {
          if (BusAssignment->data.on_ground != 0.0) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
            AutopilotStateMachine_OFF_entry();
          }
        }
      }
      break;
    }

    if (guard1) {
      if (BusAssignment->lateral.condition.LOC_TRACK) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_TRACK;
        AutopilotStateMachine_LOC_TRACK_entry();
      }
    }
  }
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_NAV_TO_HDG(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.input.HDG_pull || (!BusAssignment->lateral.condition.NAV);
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_TO_RWY_TRK(const ap_sm_output *BusAssignment)
{
  boolean_T y;
  real_T L;
  real_T R;
  real_T R_tmp;
  L = -mod_tnBo173x((BusAssignment->data.Psi_magnetic_deg - (BusAssignment->data.nav_loc_deg + 360.0)) + 360.0);
  R_tmp = std::abs(L);
  R = mod_tnBo173x(360.0 - R_tmp);
  if (R_tmp < std::abs(R)) {
    R = L;
  }

  y = (((BusAssignment->data.H_radio_ft >= 30.0) && (!BusAssignment->lateral.armed.NAV)) ||
       (!BusAssignment->data.nav_valid) || (std::abs(R) > 20.0));
  return y;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_TRK_entry(const ap_sm_output *BusAssignment)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusAssignment->data.Psi_magnetic_track_deg;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_HDG(const ap_sm_output *BusAssignment)
{
  return (BusAssignment->data_computed.time_since_lift_off >= 5.0) && (BusAssignment->lateral.input.HDG_pull ||
    (BusAssignment->data.ap_fd_active && (!BusAssignment->lateral.armed.NAV)));
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_NAV(const ap_sm_output *BusAssignment)
{
  return BusAssignment->lateral.armed.NAV && BusAssignment->lateral.condition.NAV;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_RWY(const ap_sm_output *BusAssignment)
{
  boolean_T y;
  real_T L;
  real_T R;
  real_T R_tmp;
  L = -mod_tnBo173x((BusAssignment->data.Psi_magnetic_deg - (BusAssignment->data.nav_loc_deg + 360.0)) + 360.0);
  R_tmp = std::abs(L);
  R = mod_tnBo173x(360.0 - R_tmp);
  if (R_tmp < std::abs(R)) {
    R = L;
  }

  y = ((BusAssignment->data.V2_kn > 100.0) && (BusAssignment->data.flaps_handle_index >= 1.0) &&
       (BusAssignment->data_computed.time_since_touchdown >= 30.0) && ((BusAssignment->data.throttle_lever_1_pos >= 95.0)
        || (BusAssignment->data.throttle_lever_2_pos >= 95.0)) && BusAssignment->data.nav_valid && ((std::abs
         (BusAssignment->data.nav_radial_error_deg) <= 0.4) && (std::abs(R) <= 20.0)));
  return y;
}

boolean_T AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_TO_RWY_TRK(const ap_sm_output *BusAssignment)
{
  return (!BusAssignment->lateral.armed.NAV) && (BusAssignment->data.H_radio_ft >= 30.0) &&
    (BusAssignment->data.H_radio_ft < 100.0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY;
  AutopilotStateMachine_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_dot_fcu_fpm;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_DES;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_VS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_CLB;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_VS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_OP_CLB;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_DES_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_OP_DES;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft) <= 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
    AutopilotStateMachine_B.out.law = vertical_law_VS;
    AutopilotStateMachine_B.out.H_dot_c_fpm = -1000.0;
  } else {
    AutopilotStateMachine_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_GS_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_ACQ;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT(void)
{
  real_T tmp;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
        (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
         AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
          (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
           AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_VS;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_VS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT(void)
{
  real_T tmp;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
      AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
        (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
         AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
      AutopilotStateMachine_DES_entry();
    } else if (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
      AutopilotStateMachine_VS_entry();
    } else {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT;
        AutopilotStateMachine_ALT_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CST;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT(void)
{
  real_T tmp;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST;
    AutopilotStateMachine_ALT_CST_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
        (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
         AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
               (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
          (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
           AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = 1500.0;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CST_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_ACQ;
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB(void)
{
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
    AutopilotStateMachine_ALT_CST_CPT_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
    AutopilotStateMachine_ALT_CPT_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_push ||
             AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
    AutopilotStateMachine_VS_entry();
  } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
             AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    AutopilotStateMachine_CLB_during();
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = -1500.0;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_entry_p(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_NONE;
  AutopilotStateMachine_B.out.law = vertical_law_NONE;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_NONE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ROLL_OUT_entry_o(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ROLL_OUT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
  AutopilotStateMachine_B.out.law = vertical_law_FLARE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS_TRACK_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_GS_TRACK;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_LAND_entry_i(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_LAND;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_FLARE_entry_g(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_FLARE;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
  AutopilotStateMachine_B.out.law = vertical_law_FLARE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_SRS_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_SRS;
  AutopilotStateMachine_B.out.law = vertical_law_SRS;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_NONE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS(void)
{
  boolean_T guard1 = false;
  if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS) {
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_SRS;
    AutopilotStateMachine_SRS_entry();
  } else {
    guard1 = false;
    switch (AutopilotStateMachine_DWork.is_GS) {
     case AutopilotStateMachine_IN_FLARE:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_ROLL_OUT;
        AutopilotStateMachine_ROLL_OUT_entry_o();
      }
      break;

     case AutopilotStateMachine_IN_GS_CPT:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.LOC_push ||
          AutopilotStateMachine_B.BusAssignment_g.vertical.input.APPR_push ||
          (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT)) {
        if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
          AutopilotStateMachine_OFF_entry_p();
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;

     case AutopilotStateMachine_IN_GS_TRACK:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.LAND) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_LAND_k;
        AutopilotStateMachine_LAND_entry_i();
      } else {
        if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.LOC_push ||
            AutopilotStateMachine_B.BusAssignment_g.vertical.input.APPR_push ||
            (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_TRACK)) {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
            AutopilotStateMachine_VS_entry();
          } else {
            if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry_p();
            }
          }
        }
      }
      break;

     case AutopilotStateMachine_IN_LAND_k:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.FLARE) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_FLARE;
        AutopilotStateMachine_FLARE_entry_g();
      } else {
        if (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.LAND) {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
            AutopilotStateMachine_VS_entry();
          } else {
            if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry_p();
            }
          }
        }
      }
      break;

     default:
      if (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT) {
        if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else {
          if (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
            AutopilotStateMachine_OFF_entry_p();
          }
        }
      }
      break;
    }

    if (guard1) {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_TRACK) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_TRACK;
        AutopilotStateMachine_GS_TRACK_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft = AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft;
  if (std::abs(AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft -
               AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft) > 1200.0) {
    AutopilotStateMachine_B.out.mode_autothrust = athr_mode_THRUST_CLB;
    AutopilotStateMachine_B.out.law = vertical_law_SPD_MACH;
    AutopilotStateMachine_B.out.H_dot_c_fpm = 0.0;
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_exit_internal_ON(void)
{
  AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ON(void)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  if ((!AutopilotStateMachine_B.BusAssignment_g.data.ap_fd_active) ||
      (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase >= 9.0) ||
      (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase <= 2.0)) {
    AutopilotStateMachine_exit_internal_ON();
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry_p();
  } else {
    guard1 = false;
    guard2 = false;
    switch (AutopilotStateMachine_DWork.is_ON) {
     case AutopilotStateMachine_IN_ALT:
      AutopilotStateMachine_ALT();
      break;

     case AutopilotStateMachine_IN_ALT_CPT:
      AutopilotStateMachine_ALT_CPT();
      break;

     case AutopilotStateMachine_IN_ALT_CST:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      }
      break;

     case AutopilotStateMachine_IN_ALT_CST_CPT:
      AutopilotStateMachine_ALT_CST_CPT();
      break;

     case AutopilotStateMachine_IN_CLB:
      AutopilotStateMachine_CLB();
      break;

     case AutopilotStateMachine_IN_DES:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
        AutopilotStateMachine_ALT_CST_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_DES_during();
      }
      break;

     case AutopilotStateMachine_IN_GS:
      AutopilotStateMachine_GS();
      break;

     case AutopilotStateMachine_IN_OP_CLB:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_OP_CLB_during();
      }
      break;

     case AutopilotStateMachine_IN_OP_DES:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_push ||
                 AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_OP_CLB_during();
      }
      break;

     case AutopilotStateMachine_IN_SRS:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 6.0)) {
        guard1 = true;
      } else if ((AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) &&
                 (!AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
        AutopilotStateMachine_OFF_entry_p();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft -
                       AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
            (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
             AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          guard2 = true;
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          guard1 = true;
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else {
          if ((!AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB) &&
              (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 6.0)) {
            guard2 = true;
          }
        }
      }
      break;

     default:
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS &&
                 AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft -
                       AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
            (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
             AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
          AutopilotStateMachine_OP_CLB_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
          AutopilotStateMachine_CLB_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push && (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft <
                    AutopilotStateMachine_B.BusAssignment_g.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT;
          AutopilotStateMachine_ALT_entry();
        } else {
          AutopilotStateMachine_VS_during();
        }
      }
      break;
    }

    if (guard2) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    }

    if (guard1) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    }
  }
}

void AutopilotStateMachineModelClass::step()
{
  real_T Phi;
  real_T result[3];
  boolean_T rtb_AND7;
  boolean_T rtb_AND6;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  boolean_T rtb_AND11;
  boolean_T rtb_armed;
  boolean_T rtb_AND;
  boolean_T rtb_OR4;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT;
  boolean_T rtb_AND8;
  boolean_T rtb_armed_d;
  boolean_T rtb_AND3;
  boolean_T rtb_AND10;
  uint64m_T rtb_DataTypeConversion5;
  uint64m_T rtb_DataTypeConversion1;
  uint64m_T rtb_DataTypeConversion;
  ap_sm_output BusAssignment1;
  boolean_T armed;
  real_T result_tmp;
  real_T tmp[9];
  uint64m_T tmp_0;
  uint64m_T tmp_1;
  uint64m_T tmp_2;
  uint64m_T tmp_3;
  uint64m_T tmp_4;
  uint64m_T tmp_5;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.AP_1_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.AP_2_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_b));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = (static_cast<int32_T>
    (AutopilotStateMachine_U.in.input.AP_DISCONNECT_push) > static_cast<int32_T>
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_d));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_e));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_f));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_pull) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_i));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.APPR_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_h));
  rtb_GainTheta = AutopilotStateMachine_P.GainTheta_Gain * AutopilotStateMachine_U.in.data.Theta_deg;
  rtb_GainTheta1 = AutopilotStateMachine_P.GainTheta1_Gain * AutopilotStateMachine_U.in.data.Phi_deg;
  rtb_Saturation1 = 0.017453292519943295 * rtb_GainTheta;
  Phi = 0.017453292519943295 * rtb_GainTheta1;
  result_tmp = std::tan(rtb_Saturation1);
  rtb_Saturation = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation * result_tmp;
  tmp[6] = Phi * result_tmp;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -rtb_Saturation;
  tmp[2] = 0.0;
  rtb_Saturation1 = 1.0 / std::cos(rtb_Saturation1);
  tmp[5] = rtb_Saturation1 * rtb_Saturation;
  tmp[8] = rtb_Saturation1 * Phi;
  rtb_Saturation = AutopilotStateMachine_P.Gain_Gain_k * AutopilotStateMachine_U.in.data.p_rad_s *
    AutopilotStateMachine_P.Gainpk_Gain;
  rtb_Saturation1 = AutopilotStateMachine_P.Gain_Gain * AutopilotStateMachine_U.in.data.q_rad_s *
    AutopilotStateMachine_P.Gainqk_Gain;
  result_tmp = AutopilotStateMachine_P.Gain_Gain_a * AutopilotStateMachine_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * result_tmp + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
      tmp[rtb_on_ground] * rtb_Saturation);
  }

  rtb_Saturation = AutopilotStateMachine_P.Gain_Gain_af * AutopilotStateMachine_U.in.data.gear_strut_compression_1 -
    AutopilotStateMachine_P.Constant1_Value;
  if (rtb_Saturation > AutopilotStateMachine_P.Saturation_UpperSat) {
    rtb_Saturation = AutopilotStateMachine_P.Saturation_UpperSat;
  } else {
    if (rtb_Saturation < AutopilotStateMachine_P.Saturation_LowerSat) {
      rtb_Saturation = AutopilotStateMachine_P.Saturation_LowerSat;
    }
  }

  rtb_Saturation1 = AutopilotStateMachine_P.Gain1_Gain * AutopilotStateMachine_U.in.data.gear_strut_compression_2 -
    AutopilotStateMachine_P.Constant1_Value;
  if (rtb_Saturation1 > AutopilotStateMachine_P.Saturation1_UpperSat) {
    rtb_Saturation1 = AutopilotStateMachine_P.Saturation1_UpperSat;
  } else {
    if (rtb_Saturation1 < AutopilotStateMachine_P.Saturation1_LowerSat) {
      rtb_Saturation1 = AutopilotStateMachine_P.Saturation1_LowerSat;
    }
  }

  if (AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_OnGround;
    rtb_on_ground = 1;
  } else if (AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine == AutopilotStateMachine_IN_InAir) {
    if ((rtb_Saturation > 0.1) || (rtb_Saturation1 > 0.1)) {
      AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Saturation == 0.0) && (rtb_Saturation1 == 0.0)) {
      AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  if (!AutopilotStateMachine_DWork.eventTime_not_empty_k) {
    AutopilotStateMachine_DWork.eventTime_a = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty_k = true;
  }

  if ((rtb_on_ground == 0) || (AutopilotStateMachine_DWork.eventTime_a == 0.0)) {
    AutopilotStateMachine_DWork.eventTime_a = AutopilotStateMachine_U.in.time.simulation_time;
  }

  if (!AutopilotStateMachine_DWork.eventTime_not_empty) {
    AutopilotStateMachine_DWork.eventTime = AutopilotStateMachine_U.in.time.simulation_time;
    AutopilotStateMachine_DWork.eventTime_not_empty = true;
  }

  if ((rtb_on_ground != 0) || (AutopilotStateMachine_DWork.eventTime == 0.0)) {
    AutopilotStateMachine_DWork.eventTime = AutopilotStateMachine_U.in.time.simulation_time;
  }

  if (AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    BusAssignment1.output.enabled = 0.0;
  } else if (AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (AutopilotStateMachine_DWork.DelayInput1_DSTATE) {
      AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      BusAssignment1.output.enabled = 1.0;
    } else {
      BusAssignment1.output.enabled = 0.0;
    }
  } else {
    if (AutopilotStateMachine_DWork.DelayInput1_DSTATE || AutopilotStateMachine_DWork.DelayInput1_DSTATE_d) {
      AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      BusAssignment1.output.enabled = 0.0;
    } else {
      BusAssignment1.output.enabled = 1.0;
    }
  }

  BusAssignment1.time = AutopilotStateMachine_U.in.time;
  BusAssignment1.data.Theta_deg = rtb_GainTheta;
  BusAssignment1.data.Phi_deg = rtb_GainTheta1;
  BusAssignment1.data.qk_deg_s = result[1];
  BusAssignment1.data.rk_deg_s = result[2];
  BusAssignment1.data.pk_deg_s = result[0];
  BusAssignment1.data.V_ias_kn = AutopilotStateMachine_U.in.data.V_ias_kn;
  BusAssignment1.data.V_tas_kn = AutopilotStateMachine_U.in.data.V_tas_kn;
  BusAssignment1.data.V_mach = AutopilotStateMachine_U.in.data.V_mach;
  BusAssignment1.data.V_gnd_kn = AutopilotStateMachine_U.in.data.V_gnd_kn;
  BusAssignment1.data.alpha_deg = AutopilotStateMachine_U.in.data.alpha_deg;
  BusAssignment1.data.H_ft = AutopilotStateMachine_U.in.data.H_ft;
  BusAssignment1.data.H_ind_ft = AutopilotStateMachine_U.in.data.H_ind_ft;
  BusAssignment1.data.H_radio_ft = AutopilotStateMachine_U.in.data.H_radio_ft;
  BusAssignment1.data.H_dot_ft_min = AutopilotStateMachine_U.in.data.H_dot_ft_min;
  BusAssignment1.data.Psi_magnetic_deg = AutopilotStateMachine_U.in.data.Psi_magnetic_deg;
  BusAssignment1.data.Psi_magnetic_track_deg = AutopilotStateMachine_U.in.data.Psi_magnetic_track_deg;
  BusAssignment1.data.Psi_true_deg = AutopilotStateMachine_U.in.data.Psi_true_deg;
  BusAssignment1.data.bx_m_s2 = AutopilotStateMachine_U.in.data.bx_m_s2;
  BusAssignment1.data.by_m_s2 = AutopilotStateMachine_U.in.data.by_m_s2;
  BusAssignment1.data.bz_m_s2 = AutopilotStateMachine_U.in.data.bz_m_s2;
  BusAssignment1.data.ap_fd_active = AutopilotStateMachine_U.in.data.ap_fd_active;
  BusAssignment1.data.ap_V_c_kn = AutopilotStateMachine_U.in.data.ap_V_c_kn;
  BusAssignment1.data.ap_H_c_ft = AutopilotStateMachine_U.in.data.ap_H_c_ft;
  BusAssignment1.data.ap_Psi_c_deg = AutopilotStateMachine_U.in.data.ap_Psi_c_deg;
  BusAssignment1.data.ap_H_dot_c_ft_min = AutopilotStateMachine_U.in.data.ap_H_dot_c_ft_min;
  BusAssignment1.data.ap_FPA_c_deg = AutopilotStateMachine_U.in.data.ap_FPA_c_deg;
  BusAssignment1.data.nav_valid = AutopilotStateMachine_U.in.data.nav_valid;
  BusAssignment1.data.nav_loc_deg = AutopilotStateMachine_U.in.data.nav_loc_deg;
  BusAssignment1.data.nav_radial_error_deg = AutopilotStateMachine_U.in.data.nav_radial_error_deg;
  BusAssignment1.data.nav_dme_nmi = AutopilotStateMachine_U.in.data.nav_dme_nmi;
  BusAssignment1.data.nav_gs_error_deg = AutopilotStateMachine_U.in.data.nav_gs_error_deg;
  BusAssignment1.data.flight_guidance_xtk_nmi = AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi;
  BusAssignment1.data.flight_guidance_tae_deg = AutopilotStateMachine_U.in.data.flight_guidance_tae_deg;
  BusAssignment1.data.flight_phase = AutopilotStateMachine_U.in.data.flight_phase;
  BusAssignment1.data.V2_kn = AutopilotStateMachine_U.in.data.V2_kn;
  BusAssignment1.data.is_flight_plan_available = AutopilotStateMachine_U.in.data.is_flight_plan_available;
  BusAssignment1.data.thrust_reduction_altitude = AutopilotStateMachine_U.in.data.thrust_reduction_altitude;
  BusAssignment1.data.thrust_reduction_altitude_go_around =
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude_go_around;
  BusAssignment1.data.on_ground = rtb_on_ground;
  BusAssignment1.data.zeta_deg = AutopilotStateMachine_P.Gain2_Gain * AutopilotStateMachine_U.in.data.zeta_pos;
  BusAssignment1.data.throttle_lever_1_pos = AutopilotStateMachine_U.in.data.throttle_lever_1_pos;
  BusAssignment1.data.throttle_lever_2_pos = AutopilotStateMachine_U.in.data.throttle_lever_2_pos;
  BusAssignment1.data.flaps_handle_index = AutopilotStateMachine_U.in.data.flaps_handle_index;
  BusAssignment1.data_computed.time_since_touchdown = AutopilotStateMachine_U.in.time.simulation_time -
    AutopilotStateMachine_DWork.eventTime_a;
  BusAssignment1.data_computed.time_since_lift_off = AutopilotStateMachine_U.in.time.simulation_time -
    AutopilotStateMachine_DWork.eventTime;
  BusAssignment1.input.AP_1_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE;
  BusAssignment1.input.AP_2_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_b;
  BusAssignment1.input.AP_DISCONNECT_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_d;
  BusAssignment1.input.HDG_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_e;
  BusAssignment1.input.HDG_pull = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_pull) > static_cast<int32_T>
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_g));
  BusAssignment1.input.ALT_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_f;
  BusAssignment1.input.ALT_pull = AutopilotStateMachine_DWork.DelayInput1_DSTATE_i;
  BusAssignment1.input.VS_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd;
  BusAssignment1.input.VS_pull = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_pull) > static_cast<int32_T>
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_a));
  BusAssignment1.input.LOC_push = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.LOC_push) > static_cast<int32_T>
    (AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn));
  BusAssignment1.input.APPR_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE_h;
  BusAssignment1.input.Psi_fcu_deg = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  BusAssignment1.input.H_fcu_ft = AutopilotStateMachine_U.in.input.H_fcu_ft;
  BusAssignment1.input.H_dot_fcu_fpm = AutopilotStateMachine_U.in.input.H_dot_fcu_fpm;
  BusAssignment1.input.FPA_fcu_deg = AutopilotStateMachine_U.in.input.FPA_fcu_deg;
  BusAssignment1.lateral = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.lateral;
  BusAssignment1.vertical = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.vertical;
  BusAssignment1.output.lateral_law = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_law;
  BusAssignment1.output.lateral_mode = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_mode;
  BusAssignment1.output.lateral_mode_armed = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.lateral_mode_armed;
  BusAssignment1.output.vertical_law = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_law;
  BusAssignment1.output.vertical_mode = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_mode;
  BusAssignment1.output.vertical_mode_armed =
    AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.vertical_mode_armed;
  BusAssignment1.output.Psi_c_deg = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.Psi_c_deg;
  BusAssignment1.output.H_c_ft = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.H_c_ft;
  BusAssignment1.output.H_dot_c_fpm = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.H_dot_c_fpm;
  BusAssignment1.output.FPA_c_deg = AutopilotStateMachine_P.ap_sm_output_MATLABStruct.output.FPA_c_deg;
  BusAssignment1.lateral_previous = AutopilotStateMachine_DWork.Delay_DSTATE;
  BusAssignment1.vertical_previous = AutopilotStateMachine_DWork.Delay1_DSTATE;
  if (AutopilotStateMachine_DWork.is_active_c8_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c8_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c8_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    armed = false;
  } else if (AutopilotStateMachine_DWork.is_c8_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    armed = false;
    if (AutopilotStateMachine_OFF_TO_ON(&BusAssignment1)) {
      AutopilotStateMachine_DWork.is_c8_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      armed = true;
    }
  } else {
    armed = true;
    if (AutopilotStateMachine_ON_TO_OFF(&BusAssignment1)) {
      AutopilotStateMachine_DWork.is_c8_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      armed = false;
    }
  }

  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi <=
    AutopilotStateMachine_P.CompareToConstant35_const);
  rtb_AND11 = (AutopilotStateMachine_U.in.data.is_flight_plan_available &&
               AutopilotStateMachine_DWork.DelayInput1_DSTATE_h && (rtb_on_ground == 0.0) &&
               (AutopilotStateMachine_U.in.data.H_radio_ft >= AutopilotStateMachine_P.CompareToConstant36_const));
  if (AutopilotStateMachine_DWork.is_active_c10_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c10_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c10_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    rtb_armed = false;
  } else if (AutopilotStateMachine_DWork.is_c10_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    rtb_armed = false;
    if (AutopilotStateMachine_P.Constant1_Value_b && (!rtb_AND11) && AutopilotStateMachine_DWork.DelayInput1_DSTATE_e &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_NAV) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_CPT) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LOC_TRACK) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_LAND) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode != lateral_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c10_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      rtb_armed = true;
    }
  } else {
    rtb_armed = true;
    if (BusAssignment1.input.HDG_pull || armed || (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
         lateral_mode_NAV) || (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_LAND) ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == lateral_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c10_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      rtb_armed = false;
    }
  }

  rtb_GainTheta = std::abs(AutopilotStateMachine_U.in.data.nav_radial_error_deg);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (rtb_GainTheta < AutopilotStateMachine_P.CompareToConstant_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_h &&
    AutopilotStateMachine_U.in.data.nav_valid);
  rtb_AND6 = ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
               AutopilotStateMachine_P.CompareToConstant3_const_p) ||
              (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
               AutopilotStateMachine_P.CompareToConstant4_const_a));
  if ((!rtb_AND6) || (rtb_GainTheta >= AutopilotStateMachine_P.CompareToConstant1_const) ||
      (!AutopilotStateMachine_U.in.data.nav_valid)) {
    AutopilotStateMachine_B.in = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_AND = ((AutopilotStateMachine_U.in.time.simulation_time - AutopilotStateMachine_B.in >=
              AutopilotStateMachine_P.CompareToConstant2_const) && rtb_AND6);
  rtb_AND6 = (((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
                AutopilotStateMachine_P.CompareToConstant12_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
                AutopilotStateMachine_P.CompareToConstant20_const)) &&
              ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
                AutopilotStateMachine_P.CompareToConstant11_const) ||
               (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
                AutopilotStateMachine_P.CompareToConstant21_const)) && (AutopilotStateMachine_U.in.data.H_radio_ft <=
    AutopilotStateMachine_P.CompareToConstant13_const));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant25_const);
  rtb_OR4 = ((AutopilotStateMachine_DWork.Delay_DSTATE.output.mode == AutopilotStateMachine_P.CompareToConstant23_const)
             || AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant26_const);
  rtb_OR4 = (rtb_OR4 && ((AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant22_const) || AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd) &&
             (AutopilotStateMachine_U.in.data.H_radio_ft <= AutopilotStateMachine_P.CompareToConstant24_const));
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant15_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.Delay_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant17_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd ||
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_i);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant14_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant18_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_i ||
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_f);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (AutopilotStateMachine_U.in.data.V_gnd_kn >
    AutopilotStateMachine_P.CompareToConstant34_const);
  rtb_AND7 = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd && AutopilotStateMachine_DWork.DelayInput1_DSTATE_i &&
              (rtb_on_ground != 0.0) && AutopilotStateMachine_DWork.DelayInput1_DSTATE_f);
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT =
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_h;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
    AutopilotStateMachine_P.CompareToConstant16_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
    AutopilotStateMachine_P.CompareToConstant27_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
    AutopilotStateMachine_P.CompareToConstant28_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode !=
    AutopilotStateMachine_P.CompareToConstant29_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = (AutopilotStateMachine_U.in.data.flight_phase >=
    AutopilotStateMachine_P.CompareToConstant1_const_d);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = (AutopilotStateMachine_U.in.data.flight_phase <=
    AutopilotStateMachine_P.CompareToConstant2_const_d);
  rtb_AND8 = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_h && AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd &&
              AutopilotStateMachine_DWork.DelayInput1_DSTATE_i && AutopilotStateMachine_DWork.DelayInput1_DSTATE_f &&
              AutopilotStateMachine_DWork.DelayInput1_DSTATE_e && AutopilotStateMachine_DWork.DelayInput1_DSTATE_d);
  if (AutopilotStateMachine_DWork.is_active_c9_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c9_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c9_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    rtb_armed_d = false;
  } else if (AutopilotStateMachine_DWork.is_c9_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    rtb_armed_d = false;
    if ((rtb_on_ground == 0.0) && BusAssignment1.input.APPR_push &&
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_CPT) &&
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_GS_TRACK) &&
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_LAND) &&
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode != vertical_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c9_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      rtb_armed_d = true;
    }
  } else {
    rtb_armed_d = true;
    if (BusAssignment1.input.LOC_push || BusAssignment1.input.APPR_push ||
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_CPT) ||
        (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode == vertical_mode_GS_TRACK)) {
      AutopilotStateMachine_DWork.is_c9_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      rtb_armed_d = false;
    }
  }

  rtb_GainTheta = std::abs(AutopilotStateMachine_U.in.input.H_fcu_ft - AutopilotStateMachine_U.in.data.H_ind_ft);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (AutopilotStateMachine_U.in.data.nav_gs_error_deg >=
    AutopilotStateMachine_P.CompareToConstant19_const);
  rtb_GainTheta1 = std::abs(AutopilotStateMachine_U.in.data.nav_gs_error_deg);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (rtb_GainTheta1 < AutopilotStateMachine_P.CompareToConstant6_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_h &&
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd && AutopilotStateMachine_U.in.data.nav_valid);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant9_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.Delay1_DSTATE.output.mode ==
    AutopilotStateMachine_P.CompareToConstant10_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd ||
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_i);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (rtb_GainTheta1 < AutopilotStateMachine_P.CompareToConstant7_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd &&
    AutopilotStateMachine_DWork.DelayInput1_DSTATE_i && AutopilotStateMachine_U.in.data.nav_valid);
  if (!AutopilotStateMachine_DWork.DelayInput1_DSTATE_i) {
    AutopilotStateMachine_B.in_g = AutopilotStateMachine_U.in.time.simulation_time;
  }

  rtb_AND3 = ((AutopilotStateMachine_U.in.time.simulation_time - AutopilotStateMachine_B.in_g >=
               AutopilotStateMachine_P.CompareToConstant8_const) && AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (BusAssignment1.data_computed.time_since_touchdown >=
    AutopilotStateMachine_P.CompareToConstant33_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = (AutopilotStateMachine_U.in.data.flight_phase >=
    AutopilotStateMachine_P.CompareToConstant3_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = (AutopilotStateMachine_U.in.data.flight_phase <
    AutopilotStateMachine_P.CompareToConstant4_const);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = (AutopilotStateMachine_U.in.data.V2_kn >
    AutopilotStateMachine_P.CompareToConstant32_const);
  rtb_AND10 = (AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd && AutopilotStateMachine_DWork.DelayInput1_DSTATE_i &&
               AutopilotStateMachine_DWork.DelayInput1_DSTATE_f && AutopilotStateMachine_DWork.DelayInput1_DSTATE_e);
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = (AutopilotStateMachine_U.in.data.H_ind_ft >=
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude);
  AutopilotStateMachine_B.BusAssignment_g = BusAssignment1;
  AutopilotStateMachine_B.BusAssignment_g.lateral.input.HDG_push = BusAssignment1.input.HDG_push;
  AutopilotStateMachine_B.BusAssignment_g.lateral.input.HDG_pull = BusAssignment1.input.HDG_pull;
  AutopilotStateMachine_B.BusAssignment_g.lateral.input.LOC_push = BusAssignment1.input.LOC_push;
  AutopilotStateMachine_B.BusAssignment_g.lateral.input.APPR_push = BusAssignment1.input.APPR_push;
  AutopilotStateMachine_B.BusAssignment_g.lateral.input.Psi_fcu_deg = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  AutopilotStateMachine_B.BusAssignment_g.lateral.armed.NAV = rtb_armed;
  AutopilotStateMachine_B.BusAssignment_g.lateral.armed.LOC = armed;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.NAV = rtb_AND11;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_CPT =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LOC_TRACK = rtb_AND;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.LAND = rtb_AND6;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.FLARE = rtb_OR4;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.ROLL_OUT = rtb_AND7;
  AutopilotStateMachine_B.BusAssignment_g.lateral.condition.GA_TRACK = AutopilotStateMachine_P.Constant6_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_push = BusAssignment1.input.ALT_push;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.ALT_pull = BusAssignment1.input.ALT_pull;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_push = BusAssignment1.input.VS_push;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull = BusAssignment1.input.VS_pull;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.LOC_push = BusAssignment1.input.LOC_push;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.APPR_push = BusAssignment1.input.APPR_push;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_fcu_ft = AutopilotStateMachine_U.in.input.H_fcu_ft;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.H_dot_fcu_fpm = AutopilotStateMachine_U.in.input.H_dot_fcu_fpm;
  AutopilotStateMachine_B.BusAssignment_g.vertical.input.FPA_fcu_deg = AutopilotStateMachine_U.in.input.FPA_fcu_deg;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT = rtb_AND8;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT_CST = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS = rtb_armed_d;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT = (rtb_GainTheta <=
    AutopilotStateMachine_P.CompareToConstant5_const);
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CPT = (rtb_GainTheta <= std::abs
    (AutopilotStateMachine_U.in.data.H_dot_ft_min) / AutopilotStateMachine_P.Constant10_Value);
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ALT_CST_CPT = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.DES = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_CPT = AutopilotStateMachine_DWork.DelayInput1_DSTATE_h;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.GS_TRACK = rtb_AND3;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.LAND = rtb_AND6;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.FLARE = rtb_OR4;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.ROLL_OUT = rtb_AND7;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS = rtb_AND10;
  AutopilotStateMachine_B.BusAssignment_g.vertical.condition.THR_RED = AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd;
  if (AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry();
  } else if (AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (AutopilotStateMachine_OFF_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
      AutopilotStateMachine_HDG_entry();
    } else if (AutopilotStateMachine_OFF_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
      AutopilotStateMachine_NAV_entry();
    } else if (AutopilotStateMachine_OFF_TO_RWY(&AutopilotStateMachine_B.BusAssignment_g)) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY;
      AutopilotStateMachine_RWY_entry();
    } else {
      if (AutopilotStateMachine_OFF_TO_RWY_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
        AutopilotStateMachine_RWY_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
      }
    }
  } else {
    if (AutopilotStateMachine_X_TO_OFF(&AutopilotStateMachine_B.BusAssignment_g)) {
      AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      AutopilotStateMachine_OFF_entry();
    } else {
      switch (AutopilotStateMachine_DWork.is_ON_c) {
       case AutopilotStateMachine_IN_GA_TRK:
        if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else {
          if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
            AutopilotStateMachine_NAV_entry();
          }
        }
        break;

       case AutopilotStateMachine_IN_HDG:
        if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
          AutopilotStateMachine_NAV_entry();
        } else if (AutopilotStateMachine_ON_TO_LOC(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
          AutopilotStateMachine_LOC_CPT_entry();
        } else {
          AutopilotStateMachine_HDG_during(&AutopilotStateMachine_B.BusAssignment_g);
        }
        break;

       case AutopilotStateMachine_IN_LOC:
        AutopilotStateMachine_LOC(&AutopilotStateMachine_B.BusAssignment_g);
        break;

       case AutopilotStateMachine_IN_NAV:
        if (AutopilotStateMachine_NAV_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else {
          if (AutopilotStateMachine_ON_TO_LOC(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
            AutopilotStateMachine_LOC_CPT_entry();
          }
        }
        break;

       case AutopilotStateMachine_IN_RWY:
        if (AutopilotStateMachine_RWY_TO_RWY_TRK(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
          AutopilotStateMachine_RWY_TRK_entry(&AutopilotStateMachine_B.BusAssignment_g);
        } else if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else {
          if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
            AutopilotStateMachine_NAV_entry();
          }
        }
        break;

       default:
        if (AutopilotStateMachine_ON_TO_HDG(&AutopilotStateMachine_B.BusAssignment_g)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
          AutopilotStateMachine_HDG_entry();
        } else {
          if (AutopilotStateMachine_ON_TO_NAV(&AutopilotStateMachine_B.BusAssignment_g)) {
            AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
            AutopilotStateMachine_NAV_entry();
          }
        }
        break;
      }
    }
  }

  AutopilotStateMachine_B.BusAssignment_g.lateral.output = AutopilotStateMachine_B.out_g;
  if (AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry_p();
  } else if (AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (AutopilotStateMachine_B.BusAssignment_g.data.ap_fd_active &&
        (AutopilotStateMachine_B.BusAssignment_g.data.on_ground != 0.0) &&
        AutopilotStateMachine_B.BusAssignment_g.vertical.condition.SRS) {
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_SRS;
      AutopilotStateMachine_SRS_entry();
    } else if (AutopilotStateMachine_B.BusAssignment_g.vertical.input.VS_pull ||
               ((AutopilotStateMachine_B.BusAssignment_g.data.flight_phase >= 6.0) &&
                (AutopilotStateMachine_B.BusAssignment_g.data.on_ground == 0.0) &&
                AutopilotStateMachine_B.BusAssignment_g.data.ap_fd_active &&
                (!AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB))) {
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
      AutopilotStateMachine_VS_entry();
    } else {
      if (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB &&
          AutopilotStateMachine_B.BusAssignment_g.vertical.condition.CLB &&
          (AutopilotStateMachine_B.BusAssignment_g.data.flight_phase == 6.0)) {
        AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      }
    }
  } else {
    AutopilotStateMachine_ON();
  }

  AutopilotStateMachine_DWork.Delay1_DSTATE = AutopilotStateMachine_B.BusAssignment_g.vertical;
  AutopilotStateMachine_DWork.Delay1_DSTATE.output = AutopilotStateMachine_B.out;
  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.armed.NAV),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion5.chunks[0U], 2);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.lateral.armed.LOC),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion1.chunks[0U], 2);
  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion.chunks[0U], 2);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_B.BusAssignment_g.vertical.armed.ALT_CST),
    &rtb_GainTheta);
  AutopilotStateMachine_Y.out.time = AutopilotStateMachine_B.BusAssignment_g.time;
  AutopilotStateMachine_Y.out.data = AutopilotStateMachine_B.BusAssignment_g.data;
  AutopilotStateMachine_Y.out.data_computed = AutopilotStateMachine_B.BusAssignment_g.data_computed;
  AutopilotStateMachine_Y.out.input = AutopilotStateMachine_B.BusAssignment_g.input;
  AutopilotStateMachine_Y.out.lateral = AutopilotStateMachine_B.BusAssignment_g.lateral;
  AutopilotStateMachine_Y.out.lateral_previous = AutopilotStateMachine_B.BusAssignment_g.lateral_previous;
  AutopilotStateMachine_Y.out.vertical = AutopilotStateMachine_DWork.Delay1_DSTATE;
  AutopilotStateMachine_Y.out.vertical_previous = AutopilotStateMachine_B.BusAssignment_g.vertical_previous;
  AutopilotStateMachine_Y.out.output.enabled = AutopilotStateMachine_B.BusAssignment_g.output.enabled;
  AutopilotStateMachine_Y.out.output.lateral_law = static_cast<int32_T>
    (AutopilotStateMachine_B.BusAssignment_g.lateral.output.law);
  AutopilotStateMachine_Y.out.output.lateral_mode = static_cast<int32_T>
    (AutopilotStateMachine_B.BusAssignment_g.lateral.output.mode);
  MultiWordIor(&rtb_DataTypeConversion5.chunks[0U], &rtb_DataTypeConversion1.chunks[0U], &tmp_0.chunks[0U], 2);
  AutopilotStateMachine_Y.out.output.lateral_mode_armed = uMultiWord2Double(&tmp_0.chunks[0U], 2, 0);
  AutopilotStateMachine_Y.out.output.vertical_law = static_cast<int32_T>(AutopilotStateMachine_B.out.law);
  AutopilotStateMachine_Y.out.output.vertical_mode = static_cast<int32_T>(AutopilotStateMachine_B.out.mode);
  Double2MultiWord(std::floor(rtb_GainTheta), &tmp_5.chunks[0U], 2);
  MultiWordIor(&rtb_DataTypeConversion.chunks[0U], &tmp_5.chunks[0U], &tmp_4.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.CLB), 2))), &tmp_5.chunks[0U], 2);
  MultiWordIor(&tmp_4.chunks[0U], &tmp_5.chunks[0U], &tmp_3.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.DES), 3))), &tmp_4.chunks[0U], 2);
  MultiWordIor(&tmp_3.chunks[0U], &tmp_4.chunks[0U], &tmp_2.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_B.BusAssignment_g.vertical.armed.GS), 4))), &tmp_3.chunks[0U], 2);
  MultiWordIor(&tmp_2.chunks[0U], &tmp_3.chunks[0U], &tmp_1.chunks[0U], 2);
  AutopilotStateMachine_Y.out.output.vertical_mode_armed = uMultiWord2Double(&tmp_1.chunks[0U], 2, 0);
  AutopilotStateMachine_Y.out.output.Psi_c_deg = AutopilotStateMachine_B.BusAssignment_g.lateral.output.Psi_c_deg;
  AutopilotStateMachine_Y.out.output.H_c_ft = AutopilotStateMachine_B.out.H_c_ft;
  AutopilotStateMachine_Y.out.output.H_dot_c_fpm = AutopilotStateMachine_B.out.H_dot_c_fpm;
  AutopilotStateMachine_Y.out.output.FPA_c_deg = AutopilotStateMachine_B.out.FPA_c_deg;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE = AutopilotStateMachine_U.in.input.AP_1_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = AutopilotStateMachine_U.in.input.AP_2_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = AutopilotStateMachine_U.in.input.AP_DISCONNECT_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = AutopilotStateMachine_U.in.input.HDG_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_g = AutopilotStateMachine_U.in.input.HDG_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = AutopilotStateMachine_U.in.input.ALT_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = AutopilotStateMachine_U.in.input.ALT_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = AutopilotStateMachine_U.in.input.VS_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_a = AutopilotStateMachine_U.in.input.VS_pull;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = AutopilotStateMachine_U.in.input.LOC_push;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = AutopilotStateMachine_U.in.input.APPR_push;
  AutopilotStateMachine_DWork.Delay_DSTATE = AutopilotStateMachine_B.BusAssignment_g.lateral;
}

void AutopilotStateMachineModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&AutopilotStateMachine_B)), 0,
                     sizeof(BlockIO_AutopilotStateMachine_T));
  (void) std::memset(static_cast<void *>(&AutopilotStateMachine_DWork), 0,
                     sizeof(D_Work_AutopilotStateMachine_T));
  AutopilotStateMachine_U.in = AutopilotStateMachine_rtZap_sm_input;
  AutopilotStateMachine_Y.out = AutopilotStateMachine_rtZap_sm_output;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE = AutopilotStateMachine_P.DetectIncrease_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_b = AutopilotStateMachine_P.DetectIncrease1_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_d = AutopilotStateMachine_P.DetectIncrease2_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_e = AutopilotStateMachine_P.DetectIncrease3_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_g = AutopilotStateMachine_P.DetectIncrease4_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_f = AutopilotStateMachine_P.DetectIncrease5_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_i = AutopilotStateMachine_P.DetectIncrease6_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd = AutopilotStateMachine_P.DetectIncrease7_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_a = AutopilotStateMachine_P.DetectIncrease8_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn = AutopilotStateMachine_P.DetectIncrease9_vinit;
  AutopilotStateMachine_DWork.DelayInput1_DSTATE_h = AutopilotStateMachine_P.DetectIncrease10_vinit;
  AutopilotStateMachine_DWork.Delay_DSTATE = AutopilotStateMachine_P.Delay_InitialCondition;
  AutopilotStateMachine_DWork.Delay1_DSTATE = AutopilotStateMachine_P.Delay1_InitialCondition;
  AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.eventTime_not_empty_k = false;
  AutopilotStateMachine_DWork.eventTime_not_empty = false;
  AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c8_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c8_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c10_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c10_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_B.in = AutopilotStateMachine_P.out_Y0_j;
  AutopilotStateMachine_DWork.is_active_c9_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c9_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_B.in_g = AutopilotStateMachine_P.out_Y0;
  AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
}

void AutopilotStateMachineModelClass::terminate()
{
}

AutopilotStateMachineModelClass::AutopilotStateMachineModelClass()
{
}

AutopilotStateMachineModelClass::~AutopilotStateMachineModelClass()
{
}
