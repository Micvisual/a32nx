#include "AutopilotStateMachine.h"
#include "AutopilotStateMachine_private.h"
#include "Double2MultiWord.h"
#include "MultiWordIor.h"
#include "uMultiWord2Double.h"

const uint8_T AutopilotStateMachine_IN_InAir = 1U;
const uint8_T AutopilotStateMachine_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T AutopilotStateMachine_IN_OnGround = 2U;
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
    0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { false, false, false, false, false, false, false,
    false, false, false, false, 0.0, 0.0, 0.0, 0.0 } };

void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 0);
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_BitShift1(real_T rtu_u, real_T *rty_y)
{
  *rty_y = std::ldexp(rtu_u, 1);
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_HDG_during(const ap_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_GA_TRK_entry(const ap_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_GA_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_TRK_entry(const ap_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY_TRACK;
  AutopilotStateMachine_B.out_g.law = lateral_law_TRACK;
  AutopilotStateMachine_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ON(const ap_lateral_armed
  *BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1, const ap_lateral_condition
  *BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1, const ap_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  boolean_T guard1 = false;
  if ((!AutopilotStateMachine_B.BusAssignment.data.ap_fd_active) ||
      (AutopilotStateMachine_B.BusAssignment.data.flight_phase >= 9.0) ||
      (AutopilotStateMachine_B.BusAssignment.data.flight_phase <= 2.0)) {
    AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry();
  } else {
    guard1 = false;
    switch (AutopilotStateMachine_DWork.is_ON_c) {
     case AutopilotStateMachine_IN_GA_TRK:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
        AutopilotStateMachine_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
          AutopilotStateMachine_NAV_entry();
        }
      }
      break;

     case AutopilotStateMachine_IN_HDG:
      if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
           BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
          (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
           BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
        AutopilotStateMachine_NAV_entry();
      } else if (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->LOC &&
                 BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
        AutopilotStateMachine_LOC_CPT_entry();
      } else {
        AutopilotStateMachine_HDG_during(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      }
      break;

     case AutopilotStateMachine_IN_LOC:
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->GA_TRACK) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_GA_TRK;
        AutopilotStateMachine_GA_TRK_entry(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      } else {
        switch (AutopilotStateMachine_DWork.is_LOC) {
         case AutopilotStateMachine_IN_FLARE:
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->ROLL_OUT) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_ROLL_OUT;
            AutopilotStateMachine_ROLL_OUT_entry();
          }
          break;

         case AutopilotStateMachine_IN_LAND:
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->FLARE) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_FLARE;
            AutopilotStateMachine_FLARE_entry();
          } else {
            if (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LAND) {
              if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
                AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
                AutopilotStateMachine_HDG_entry();
              } else {
                if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
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
          if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->LOC_push ||
              BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->APPR_push ||
              (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT)) {
            if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
              AutopilotStateMachine_HDG_entry();
            } else if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
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
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LAND) {
            AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LAND;
            AutopilotStateMachine_LAND_entry();
          } else {
            if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->LOC_push ||
                BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->APPR_push ||
                (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_TRACK)) {
              if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
                AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
                AutopilotStateMachine_HDG_entry();
              } else {
                if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
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
          if (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->ROLL_OUT) {
            if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
              AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
              AutopilotStateMachine_HDG_entry();
            } else {
              if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
                AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
                AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
                AutopilotStateMachine_OFF_entry();
              }
            }
          }
          break;
        }
      }
      break;

     case AutopilotStateMachine_IN_NAV:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull ||
          (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
        AutopilotStateMachine_HDG_entry();
      } else {
        if (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->LOC &&
            BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_LOC;
          AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_CPT;
          AutopilotStateMachine_LOC_CPT_entry();
        }
      }
      break;

     case AutopilotStateMachine_IN_RWY:
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->RWY_TRACK) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
        AutopilotStateMachine_RWY_TRK_entry(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      } else if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
        AutopilotStateMachine_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
          AutopilotStateMachine_NAV_entry();
        }
      }
      break;

     default:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
        AutopilotStateMachine_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
          AutopilotStateMachine_NAV_entry();
        }
      }
      break;
    }

    if (guard1) {
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_TRACK) {
        AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_LOC_TRACK;
        AutopilotStateMachine_LOC_TRACK_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_RWY_entry(void)
{
  AutopilotStateMachine_B.out_g.mode = lateral_mode_RWY;
  AutopilotStateMachine_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_VS_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_dot_fcu_fpm;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_HOLD;
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
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
  if (std::abs(AutopilotStateMachine_B.BusAssignment.data.H_ind_ft -
               AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) <=
      1200.0) {
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
  if (std::abs(AutopilotStateMachine_B.BusAssignment.data.H_ind_ft -
               AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) <=
      1200.0) {
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
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT(const ap_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
      BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0)
        && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
            AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else {
      if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
           100.0) && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                      AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CPT(const ap_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
      BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0)
        && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
            AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
      AutopilotStateMachine_DES_entry();
    } else if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
      AutopilotStateMachine_VS_entry();
    } else {
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT) {
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
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT(const ap_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST;
    AutopilotStateMachine_ALT_CST_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    tmp = std::abs(AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
    if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0)
        && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
            AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
      AutopilotStateMachine_OP_DES_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
      AutopilotStateMachine_OP_CLB_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
                100.0) &&
               (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
      AutopilotStateMachine_CLB_entry();
    } else {
      if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >=
           100.0) && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                      AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
        AutopilotStateMachine_DES_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = 1500.0;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ALT_CST_CPT_entry(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_ALT_CST_CPT;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_ALT_ACQ;
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_CLB(const ap_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
    AutopilotStateMachine_ALT_CST_CPT_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
    AutopilotStateMachine_ALT_CPT_entry();
  } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
             AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
    AutopilotStateMachine_VS_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
    AutopilotStateMachine_GS_CPT_entry();
  } else {
    AutopilotStateMachine_CLB_during();
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_DES_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  AutopilotStateMachine_B.out.H_dot_c_fpm = -1500.0;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OFF_entry_g(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_NONE;
  AutopilotStateMachine_B.out.law = vertical_law_NONE;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_NONE;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_ROLL_OUT_entry_b(void)
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_LAND_entry_m(void)
{
  AutopilotStateMachine_B.out.mode = vertical_mode_LAND;
  AutopilotStateMachine_B.out.mode_autothrust = athr_mode_SPEED;
  AutopilotStateMachine_B.out.law = vertical_law_GS;
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_FLARE_entry_b(void)
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_GS(const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  boolean_T guard1 = false;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->SRS) {
    AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
    AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_SRS;
    AutopilotStateMachine_SRS_entry();
  } else {
    guard1 = false;
    switch (AutopilotStateMachine_DWork.is_GS) {
     case AutopilotStateMachine_IN_FLARE:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ROLL_OUT) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_ROLL_OUT;
        AutopilotStateMachine_ROLL_OUT_entry_b();
      }
      break;

     case AutopilotStateMachine_IN_GS_CPT:
      if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push ||
          AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push ||
          (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT)) {
        if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
          AutopilotStateMachine_OFF_entry_g();
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;

     case AutopilotStateMachine_IN_GS_TRACK:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->LAND) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_LAND_k;
        AutopilotStateMachine_LAND_entry_m();
      } else {
        if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push ||
            AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push ||
            (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_TRACK)) {
          if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
            AutopilotStateMachine_VS_entry();
          } else {
            if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry_g();
            }
          }
        }
      }
      break;

     case AutopilotStateMachine_IN_LAND_k:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->FLARE) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_FLARE;
        AutopilotStateMachine_FLARE_entry_b();
      } else {
        if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->LAND) {
          if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
            AutopilotStateMachine_VS_entry();
          } else {
            if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
              AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
              AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
              AutopilotStateMachine_OFF_entry_g();
            }
          }
        }
      }
      break;

     default:
      if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ROLL_OUT) {
        if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
          AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
          AutopilotStateMachine_VS_entry();
        } else {
          if (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) {
            AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
            AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
            AutopilotStateMachine_OFF_entry_g();
          }
        }
      }
      break;
    }

    if (guard1) {
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_TRACK) {
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_TRACK;
        AutopilotStateMachine_GS_TRACK_entry();
      }
    }
  }
}

void AutopilotStateMachineModelClass::AutopilotStateMachine_OP_CLB_during(void)
{
  AutopilotStateMachine_B.out.H_c_ft =
    AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  if (std::abs(AutopilotStateMachine_B.BusAssignment.data.H_ind_ft -
               AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) > 1200.0)
  {
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

void AutopilotStateMachineModelClass::AutopilotStateMachine_ON_n(const ap_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  if ((!AutopilotStateMachine_B.BusAssignment.data.ap_fd_active) ||
      (AutopilotStateMachine_B.BusAssignment.data.flight_phase >= 9.0) ||
      (AutopilotStateMachine_B.BusAssignment.data.flight_phase <= 2.0)) {
    AutopilotStateMachine_exit_internal_ON();
    AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry_g();
  } else {
    guard1 = false;
    guard2 = false;
    switch (AutopilotStateMachine_DWork.is_ON) {
     case AutopilotStateMachine_IN_ALT:
      AutopilotStateMachine_ALT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
        BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case AutopilotStateMachine_IN_ALT_CPT:
      AutopilotStateMachine_ALT_CPT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
        BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case AutopilotStateMachine_IN_ALT_CST:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      }
      break;

     case AutopilotStateMachine_IN_ALT_CST_CPT:
      AutopilotStateMachine_ALT_CST_CPT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
        BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case AutopilotStateMachine_IN_CLB:
      AutopilotStateMachine_CLB(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
        BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case AutopilotStateMachine_IN_DES:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CST_CPT;
        AutopilotStateMachine_ALT_CST_CPT_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_DES_during();
      }
      break;

     case AutopilotStateMachine_IN_GS:
      AutopilotStateMachine_GS(BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case AutopilotStateMachine_IN_OP_CLB:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_OP_CLB_during();
      }
      break;

     case AutopilotStateMachine_IN_OP_DES:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
        AutopilotStateMachine_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        AutopilotStateMachine_OP_CLB_during();
      }
      break;

     case AutopilotStateMachine_IN_SRS:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->CLB &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->CLB &&
          (AutopilotStateMachine_B.BusAssignment.data.flight_phase == 6.0)) {
        guard1 = true;
      } else if ((AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) &&
                 (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->SRS)) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
        AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
        AutopilotStateMachine_OFF_entry_g();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                       AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >=
             100.0) && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                        AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          guard2 = true;
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          guard1 = true;
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else {
          if ((!BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->CLB) &&
              (AutopilotStateMachine_B.BusAssignment.data.flight_phase == 6.0)) {
            guard2 = true;
          }
        }
      }
      break;

     default:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_ALT_CPT;
        AutopilotStateMachine_ALT_CPT_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_GS;
        AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_GS_CPT;
        AutopilotStateMachine_GS_CPT_entry();
      } else {
        tmp = std::abs(AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                       AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
        if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >=
             100.0) && (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                        AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_DES;
          AutopilotStateMachine_OP_DES_entry();
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_OP_CLB;
          AutopilotStateMachine_OP_CLB_entry();
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
          AutopilotStateMachine_CLB_entry();
        } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push &&
                   (tmp >= 100.0) &&
                   (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft)) {
          AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_DES;
          AutopilotStateMachine_DES_entry();
        } else if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT) {
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
  boolean_T rtb_RelationalOperator1;
  boolean_T rtb_AND7;
  boolean_T rtb_AND10;
  boolean_T rtb_AND3;
  boolean_T rtb_AND5;
  boolean_T rtb_AND9;
  boolean_T rtb_AND6;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_Compare_i;
  boolean_T rtb_AND8;
  real_T rtb_GainTheta;
  real_T rtb_GainTheta1;
  real_T rtb_Saturation;
  real_T rtb_Saturation1;
  int32_T rtb_on_ground;
  boolean_T rtb_armed;
  boolean_T rtb_armed_n;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_NAV;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT;
  boolean_T rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_TRACK;
  boolean_T rtb_armed_g;
  uint64m_T rtb_DataTypeConversion5;
  uint64m_T rtb_DataTypeConversion1;
  uint64m_T rtb_DataTypeConversion;
  ap_lateral_armed BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1;
  ap_lateral_condition BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1;
  ap_lateral_input BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1;
  ap_vertical_armed BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1;
  ap_vertical_condition BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1;
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
  rtb_AND7 = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.AP_DISCONNECT_push) > static_cast<int32_T>
              (AutopilotStateMachine_DWork.DelayInput1_DSTATE_d));
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

  AutopilotStateMachine_B.BusAssignment = AutopilotStateMachine_P.ap_sm_output_MATLABStruct;
  AutopilotStateMachine_B.BusAssignment.time = AutopilotStateMachine_U.in.time;
  AutopilotStateMachine_B.BusAssignment.input.AP_1_push = AutopilotStateMachine_DWork.DelayInput1_DSTATE;
  AutopilotStateMachine_B.BusAssignment.input.AP_2_push = (static_cast<int32_T>
    (AutopilotStateMachine_U.in.input.AP_2_push) > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_b));
  AutopilotStateMachine_B.BusAssignment.input.AP_DISCONNECT_push = rtb_AND7;
  AutopilotStateMachine_B.BusAssignment.input.HDG_push = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_push)
    > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_e));
  AutopilotStateMachine_B.BusAssignment.input.HDG_pull = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.HDG_pull)
    > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_g));
  AutopilotStateMachine_B.BusAssignment.input.ALT_push = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_push)
    > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_f));
  AutopilotStateMachine_B.BusAssignment.input.ALT_pull = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.ALT_pull)
    > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_i));
  AutopilotStateMachine_B.BusAssignment.input.VS_push = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_push) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_bd));
  AutopilotStateMachine_B.BusAssignment.input.VS_pull = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.VS_pull) >
    static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_a));
  AutopilotStateMachine_B.BusAssignment.input.LOC_push = (static_cast<int32_T>(AutopilotStateMachine_U.in.input.LOC_push)
    > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_fn));
  AutopilotStateMachine_B.BusAssignment.input.APPR_push = (static_cast<int32_T>
    (AutopilotStateMachine_U.in.input.APPR_push) > static_cast<int32_T>(AutopilotStateMachine_DWork.DelayInput1_DSTATE_h));
  AutopilotStateMachine_B.BusAssignment.input.Psi_fcu_deg = AutopilotStateMachine_U.in.input.Psi_fcu_deg;
  AutopilotStateMachine_B.BusAssignment.input.H_fcu_ft = AutopilotStateMachine_U.in.input.H_fcu_ft;
  AutopilotStateMachine_B.BusAssignment.input.H_dot_fcu_fpm = AutopilotStateMachine_U.in.input.H_dot_fcu_fpm;
  AutopilotStateMachine_B.BusAssignment.input.FPA_fcu_deg = AutopilotStateMachine_U.in.input.FPA_fcu_deg;
  AutopilotStateMachine_B.BusAssignment.data.Theta_deg = rtb_GainTheta;
  AutopilotStateMachine_B.BusAssignment.data.Phi_deg = rtb_GainTheta1;
  AutopilotStateMachine_B.BusAssignment.data.qk_deg_s = result[1];
  AutopilotStateMachine_B.BusAssignment.data.rk_deg_s = result[2];
  AutopilotStateMachine_B.BusAssignment.data.pk_deg_s = result[0];
  AutopilotStateMachine_B.BusAssignment.data.V_ias_kn = AutopilotStateMachine_U.in.data.V_ias_kn;
  AutopilotStateMachine_B.BusAssignment.data.V_tas_kn = AutopilotStateMachine_U.in.data.V_tas_kn;
  AutopilotStateMachine_B.BusAssignment.data.V_mach = AutopilotStateMachine_U.in.data.V_mach;
  AutopilotStateMachine_B.BusAssignment.data.V_gnd_kn = AutopilotStateMachine_U.in.data.V_gnd_kn;
  AutopilotStateMachine_B.BusAssignment.data.alpha_deg = AutopilotStateMachine_U.in.data.alpha_deg;
  AutopilotStateMachine_B.BusAssignment.data.H_ft = AutopilotStateMachine_U.in.data.H_ft;
  AutopilotStateMachine_B.BusAssignment.data.H_ind_ft = AutopilotStateMachine_U.in.data.H_ind_ft;
  AutopilotStateMachine_B.BusAssignment.data.H_radio_ft = AutopilotStateMachine_U.in.data.H_radio_ft;
  AutopilotStateMachine_B.BusAssignment.data.H_dot_ft_min = AutopilotStateMachine_U.in.data.H_dot_ft_min;
  AutopilotStateMachine_B.BusAssignment.data.Psi_magnetic_deg = AutopilotStateMachine_U.in.data.Psi_magnetic_deg;
  AutopilotStateMachine_B.BusAssignment.data.Psi_magnetic_track_deg =
    AutopilotStateMachine_U.in.data.Psi_magnetic_track_deg;
  AutopilotStateMachine_B.BusAssignment.data.Psi_true_deg = AutopilotStateMachine_U.in.data.Psi_true_deg;
  AutopilotStateMachine_B.BusAssignment.data.bx_m_s2 = AutopilotStateMachine_U.in.data.bx_m_s2;
  AutopilotStateMachine_B.BusAssignment.data.by_m_s2 = AutopilotStateMachine_U.in.data.by_m_s2;
  AutopilotStateMachine_B.BusAssignment.data.bz_m_s2 = AutopilotStateMachine_U.in.data.bz_m_s2;
  AutopilotStateMachine_B.BusAssignment.data.ap_fd_active = AutopilotStateMachine_U.in.data.ap_fd_active;
  AutopilotStateMachine_B.BusAssignment.data.ap_V_c_kn = AutopilotStateMachine_U.in.data.ap_V_c_kn;
  AutopilotStateMachine_B.BusAssignment.data.ap_H_c_ft = AutopilotStateMachine_U.in.data.ap_H_c_ft;
  AutopilotStateMachine_B.BusAssignment.data.ap_Psi_c_deg = AutopilotStateMachine_U.in.data.ap_Psi_c_deg;
  AutopilotStateMachine_B.BusAssignment.data.ap_H_dot_c_ft_min = AutopilotStateMachine_U.in.data.ap_H_dot_c_ft_min;
  AutopilotStateMachine_B.BusAssignment.data.ap_FPA_c_deg = AutopilotStateMachine_U.in.data.ap_FPA_c_deg;
  AutopilotStateMachine_B.BusAssignment.data.nav_valid = AutopilotStateMachine_U.in.data.nav_valid;
  AutopilotStateMachine_B.BusAssignment.data.nav_loc_deg = AutopilotStateMachine_U.in.data.nav_loc_deg;
  AutopilotStateMachine_B.BusAssignment.data.nav_radial_error_deg = AutopilotStateMachine_U.in.data.nav_radial_error_deg;
  AutopilotStateMachine_B.BusAssignment.data.nav_dme_nmi = AutopilotStateMachine_U.in.data.nav_dme_nmi;
  AutopilotStateMachine_B.BusAssignment.data.nav_gs_error_deg = AutopilotStateMachine_U.in.data.nav_gs_error_deg;
  AutopilotStateMachine_B.BusAssignment.data.flight_guidance_xtk_nmi =
    AutopilotStateMachine_U.in.data.flight_guidance_xtk_nmi;
  AutopilotStateMachine_B.BusAssignment.data.flight_guidance_tae_deg =
    AutopilotStateMachine_U.in.data.flight_guidance_tae_deg;
  AutopilotStateMachine_B.BusAssignment.data.flight_phase = AutopilotStateMachine_U.in.data.flight_phase;
  AutopilotStateMachine_B.BusAssignment.data.V2_kn = AutopilotStateMachine_U.in.data.V2_kn;
  AutopilotStateMachine_B.BusAssignment.data.is_flight_plan_available =
    AutopilotStateMachine_U.in.data.is_flight_plan_available;
  AutopilotStateMachine_B.BusAssignment.data.thrust_reduction_altitude =
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude;
  AutopilotStateMachine_B.BusAssignment.data.thrust_reduction_altitude_go_around =
    AutopilotStateMachine_U.in.data.thrust_reduction_altitude_go_around;
  AutopilotStateMachine_B.BusAssignment.data.on_ground = rtb_on_ground;
  AutopilotStateMachine_B.BusAssignment.data.zeta_deg = AutopilotStateMachine_P.Gain2_Gain *
    AutopilotStateMachine_U.in.data.zeta_pos;
  AutopilotStateMachine_B.BusAssignment.data.throttle_lever_1_pos = AutopilotStateMachine_U.in.data.throttle_lever_1_pos;
  AutopilotStateMachine_B.BusAssignment.data.throttle_lever_2_pos = AutopilotStateMachine_U.in.data.throttle_lever_2_pos;
  if (AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_B.BusAssignment.output.enabled = 0.0;
  } else if (AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (AutopilotStateMachine_DWork.DelayInput1_DSTATE) {
      AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_B.BusAssignment.output.enabled = 1.0;
    } else {
      AutopilotStateMachine_B.BusAssignment.output.enabled = 0.0;
    }
  } else {
    if (AutopilotStateMachine_DWork.DelayInput1_DSTATE || rtb_AND7) {
      AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      AutopilotStateMachine_B.BusAssignment.output.enabled = 0.0;
    } else {
      AutopilotStateMachine_B.BusAssignment.output.enabled = 1.0;
    }
  }

  if (AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    rtb_armed = false;
  } else if (AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    rtb_armed = false;
    if (AutopilotStateMachine_P.Constant1_Value_e && AutopilotStateMachine_B.BusAssignment.input.HDG_push &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_NAV) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_CPT) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_TRACK) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LAND) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      rtb_armed = true;
    }
  } else {
    rtb_armed = true;
    if (AutopilotStateMachine_B.BusAssignment.input.HDG_pull ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode == lateral_mode_NAV)) {
      AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      rtb_armed = false;
    }
  }

  if (AutopilotStateMachine_DWork.is_active_c4_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c4_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c4_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    rtb_armed_n = false;
  } else if (AutopilotStateMachine_DWork.is_c4_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    rtb_armed_n = false;
    if ((AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) &&
        (AutopilotStateMachine_B.BusAssignment.input.LOC_push || AutopilotStateMachine_B.BusAssignment.input.APPR_push) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_CPT) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_TRACK) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LAND) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c4_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      rtb_armed_n = true;
    }
  } else {
    rtb_armed_n = true;
    if ((AutopilotStateMachine_B.BusAssignment.input.LOC_push &&
         (!AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.GS)) ||
        (AutopilotStateMachine_B.BusAssignment.input.APPR_push &&
         AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.GS) ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode == lateral_mode_LOC_CPT) ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode == lateral_mode_LOC_TRACK)) {
      AutopilotStateMachine_DWork.is_c4_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      rtb_armed_n = false;
    }
  }

  rtb_AND8 = ((AutopilotStateMachine_B.BusAssignment.data.flight_guidance_xtk_nmi <=
               AutopilotStateMachine_P.CompareToConstant35_const) &&
              (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) &&
              (AutopilotStateMachine_B.BusAssignment.data.H_radio_ft >=
               AutopilotStateMachine_P.CompareToConstant36_const));
  rtb_GainTheta = std::abs(AutopilotStateMachine_B.BusAssignment.data.nav_radial_error_deg);
  rtb_Compare_i = ((rtb_GainTheta < AutopilotStateMachine_P.CompareToConstant_const) &&
                   AutopilotStateMachine_B.BusAssignment.data.nav_valid);
  rtb_RelationalOperator = ((AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
    AutopilotStateMachine_P.CompareToConstant3_const_f) || (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode
    == AutopilotStateMachine_P.CompareToConstant4_const_k));
  rtb_AND10 = !AutopilotStateMachine_B.BusAssignment.data.nav_valid;
  if ((!rtb_RelationalOperator) || (rtb_GainTheta >= AutopilotStateMachine_P.CompareToConstant1_const) || rtb_AND10) {
    AutopilotStateMachine_B.in = AutopilotStateMachine_B.BusAssignment.time.simulation_time;
  }

  rtb_RelationalOperator = ((AutopilotStateMachine_B.BusAssignment.time.simulation_time - AutopilotStateMachine_B.in >=
    AutopilotStateMachine_P.CompareToConstant2_const) && rtb_RelationalOperator);
  rtb_AND6 = (((AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant12_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant20_const)) &&
              ((AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant11_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant21_const)) &&
              (AutopilotStateMachine_B.BusAssignment.data.H_radio_ft <=
               AutopilotStateMachine_P.CompareToConstant13_const));
  rtb_AND9 = (((AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant23_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant25_const)) &&
              ((AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant22_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant26_const)) &&
              (AutopilotStateMachine_B.BusAssignment.data.H_radio_ft <=
               AutopilotStateMachine_P.CompareToConstant24_const));
  rtb_AND7 = (((AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant15_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode ==
                AutopilotStateMachine_P.CompareToConstant17_const)) &&
              ((AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant14_const) ||
               (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
                AutopilotStateMachine_P.CompareToConstant18_const)) &&
              (AutopilotStateMachine_B.BusAssignment.data.on_ground != 0.0) &&
              (AutopilotStateMachine_B.BusAssignment.data.V_gnd_kn > AutopilotStateMachine_P.CompareToConstant34_const));
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_NAV = rtb_AND8;
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT = rtb_Compare_i;
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_TRACK = rtb_RelationalOperator;
  BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1.NAV = rtb_armed;
  BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1.LOC = rtb_armed_n;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.NAV = rtb_AND8;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LOC_CPT = rtb_Compare_i;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LOC_TRACK = rtb_RelationalOperator;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LAND = rtb_AND6;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.FLARE = rtb_AND9;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.ROLL_OUT = rtb_AND7;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.GA_TRACK = AutopilotStateMachine_P.Constant6_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.RWY = AutopilotStateMachine_P.Constant4_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.RWY_TRACK = AutopilotStateMachine_P.Constant5_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.HDG_push =
    AutopilotStateMachine_B.BusAssignment.input.HDG_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.HDG_pull =
    AutopilotStateMachine_B.BusAssignment.input.HDG_pull;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.LOC_push =
    AutopilotStateMachine_B.BusAssignment.input.LOC_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.APPR_push =
    AutopilotStateMachine_B.BusAssignment.input.APPR_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.Psi_fcu_deg =
    AutopilotStateMachine_B.BusAssignment.input.Psi_fcu_deg;
  if (AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry();
  } else if (AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (((AutopilotStateMachine_B.BusAssignment.data.flight_phase >= 6.0) &&
         AutopilotStateMachine_B.BusAssignment.input.HDG_pull) ||
        ((AutopilotStateMachine_B.BusAssignment.data.flight_phase >= 6.0) &&
         AutopilotStateMachine_B.BusAssignment.data.ap_fd_active && (!rtb_armed))) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_HDG;
      AutopilotStateMachine_HDG_entry();
    } else if (rtb_armed && rtb_AND8) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NAV;
      AutopilotStateMachine_NAV_entry();
    } else if (AutopilotStateMachine_P.Constant4_Value) {
      AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY;
      AutopilotStateMachine_RWY_entry();
    } else {
      if (AutopilotStateMachine_P.Constant5_Value) {
        AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_RWY_TRK;
        AutopilotStateMachine_RWY_TRK_entry(&BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      }
    }
  } else {
    AutopilotStateMachine_ON(&BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1,
      &BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1,
      &BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  }

  rtb_AND8 = ((AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode !=
               AutopilotStateMachine_P.CompareToConstant16_const) &&
              (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode !=
               AutopilotStateMachine_P.CompareToConstant27_const) &&
              (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode !=
               AutopilotStateMachine_P.CompareToConstant28_const) &&
              (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode !=
               AutopilotStateMachine_P.CompareToConstant29_const) &&
              (AutopilotStateMachine_DWork.Delay_DSTATE.data.flight_phase >=
               AutopilotStateMachine_P.CompareToConstant1_const_m) &&
              (AutopilotStateMachine_DWork.Delay_DSTATE.data.flight_phase <=
               AutopilotStateMachine_P.CompareToConstant2_const_o));
  if (AutopilotStateMachine_DWork.is_active_c7_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c7_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c7_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    rtb_armed_g = false;
  } else if (AutopilotStateMachine_DWork.is_c7_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    rtb_armed_g = false;
    if ((AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) &&
        AutopilotStateMachine_B.BusAssignment.input.APPR_push &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_GS_CPT) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_GS_TRACK) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_LAND) &&
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_FLARE)) {
      AutopilotStateMachine_DWork.is_c7_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      rtb_armed_g = true;
    }
  } else {
    rtb_armed_g = true;
    if (AutopilotStateMachine_B.BusAssignment.input.LOC_push || AutopilotStateMachine_B.BusAssignment.input.APPR_push ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode == vertical_mode_GS_CPT) ||
        (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode == vertical_mode_GS_TRACK)) {
      AutopilotStateMachine_DWork.is_c7_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
      rtb_armed_g = false;
    }
  }

  rtb_GainTheta = std::abs(AutopilotStateMachine_B.BusAssignment.input.H_fcu_ft -
    AutopilotStateMachine_B.BusAssignment.data.H_ind_ft);
  rtb_Compare_i = (rtb_GainTheta <= AutopilotStateMachine_P.CompareToConstant5_const);
  rtb_RelationalOperator = (rtb_GainTheta <= std::abs(AutopilotStateMachine_B.BusAssignment.data.H_dot_ft_min) /
    AutopilotStateMachine_P.Constant10_Value);
  rtb_GainTheta = std::abs(AutopilotStateMachine_B.BusAssignment.data.nav_gs_error_deg);
  rtb_AND5 = ((AutopilotStateMachine_B.BusAssignment.data.nav_gs_error_deg >=
               AutopilotStateMachine_P.CompareToConstant19_const) && (rtb_GainTheta <
    AutopilotStateMachine_P.CompareToConstant6_const) && AutopilotStateMachine_B.BusAssignment.data.nav_valid);
  rtb_AND3 = ((AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
               AutopilotStateMachine_P.CompareToConstant9_const) ||
              (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode ==
               AutopilotStateMachine_P.CompareToConstant10_const));
  if ((!rtb_AND3) || (rtb_GainTheta >= AutopilotStateMachine_P.CompareToConstant7_const) || rtb_AND10) {
    AutopilotStateMachine_B.in_i = AutopilotStateMachine_B.BusAssignment.time.simulation_time;
  }

  rtb_AND3 = ((AutopilotStateMachine_B.BusAssignment.time.simulation_time - AutopilotStateMachine_B.in_i >=
               AutopilotStateMachine_P.CompareToConstant8_const) && rtb_AND3);
  if (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) {
    AutopilotStateMachine_B.in_l = AutopilotStateMachine_B.BusAssignment.time.simulation_time;
  }

  AutopilotStateMachine_DWork.DelayInput1_DSTATE = (AutopilotStateMachine_B.BusAssignment.data.V2_kn >
    AutopilotStateMachine_P.CompareToConstant32_const);
  rtb_AND10 = ((AutopilotStateMachine_B.BusAssignment.time.simulation_time - AutopilotStateMachine_B.in_l >=
                AutopilotStateMachine_P.CompareToConstant33_const) &&
               (AutopilotStateMachine_DWork.Delay_DSTATE.data.flight_phase >=
                AutopilotStateMachine_P.CompareToConstant3_const) &&
               (AutopilotStateMachine_DWork.Delay_DSTATE.data.flight_phase <
                AutopilotStateMachine_P.CompareToConstant4_const) && AutopilotStateMachine_DWork.DelayInput1_DSTATE);
  rtb_RelationalOperator1 = (AutopilotStateMachine_B.BusAssignment.data.H_ind_ft >=
    AutopilotStateMachine_B.BusAssignment.data.thrust_reduction_altitude);
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push =
    AutopilotStateMachine_B.BusAssignment.input.ALT_push;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull =
    AutopilotStateMachine_B.BusAssignment.input.ALT_pull;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push =
    AutopilotStateMachine_B.BusAssignment.input.VS_push;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull =
    AutopilotStateMachine_B.BusAssignment.input.VS_pull;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push =
    AutopilotStateMachine_B.BusAssignment.input.LOC_push;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push =
    AutopilotStateMachine_B.BusAssignment.input.APPR_push;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft =
    AutopilotStateMachine_B.BusAssignment.input.H_fcu_ft;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_dot_fcu_fpm =
    AutopilotStateMachine_B.BusAssignment.input.H_dot_fcu_fpm;
  AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.FPA_fcu_deg =
    AutopilotStateMachine_B.BusAssignment.input.FPA_fcu_deg;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.ALT = rtb_AND8;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.ALT_CST = AutopilotStateMachine_P.Constant2_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.CLB = AutopilotStateMachine_P.Constant2_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.DES = AutopilotStateMachine_P.Constant2_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.GS = rtb_armed_g;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT = rtb_Compare_i;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CPT = rtb_RelationalOperator;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CST = AutopilotStateMachine_P.Constant3_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CST_CPT = AutopilotStateMachine_P.Constant3_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.CLB = AutopilotStateMachine_P.Constant3_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.DES = AutopilotStateMachine_P.Constant3_Value;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.GS_CPT = rtb_AND5;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.GS_TRACK = rtb_AND3;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.LAND = rtb_AND6;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.FLARE = rtb_AND9;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ROLL_OUT = rtb_AND7;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.SRS = rtb_AND10;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.THR_RED = rtb_RelationalOperator1;
  if (AutopilotStateMachine_DWork.is_active_c2_AutopilotStateMachine == 0U) {
    AutopilotStateMachine_DWork.is_active_c2_AutopilotStateMachine = 1U;
    AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_OFF;
    AutopilotStateMachine_OFF_entry_g();
  } else if (AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine == AutopilotStateMachine_IN_OFF) {
    if (AutopilotStateMachine_B.BusAssignment.data.ap_fd_active && (AutopilotStateMachine_B.BusAssignment.data.on_ground
         != 0.0) && rtb_AND10) {
      AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_SRS;
      AutopilotStateMachine_SRS_entry();
    } else if (AutopilotStateMachine_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull ||
               ((AutopilotStateMachine_B.BusAssignment.data.flight_phase >= 6.0) &&
                (AutopilotStateMachine_B.BusAssignment.data.on_ground == 0.0) &&
                AutopilotStateMachine_B.BusAssignment.data.ap_fd_active && (!AutopilotStateMachine_P.Constant2_Value)))
    {
      AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
      AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_VS;
      AutopilotStateMachine_VS_entry();
    } else {
      if (AutopilotStateMachine_P.Constant2_Value && AutopilotStateMachine_P.Constant3_Value &&
          (AutopilotStateMachine_B.BusAssignment.data.flight_phase == 6.0)) {
        AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_ON;
        AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_CLB;
        AutopilotStateMachine_CLB_entry();
      }
    }
  } else {
    AutopilotStateMachine_ON_n(&BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
      &BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  }

  AutopilotStateMachine_DWork.Delay_DSTATE = AutopilotStateMachine_B.BusAssignment;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.input.HDG_push = AutopilotStateMachine_B.BusAssignment.input.HDG_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.input.HDG_pull = AutopilotStateMachine_B.BusAssignment.input.HDG_pull;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.input.LOC_push = AutopilotStateMachine_B.BusAssignment.input.LOC_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.input.APPR_push =
    AutopilotStateMachine_B.BusAssignment.input.APPR_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.input.Psi_fcu_deg =
    AutopilotStateMachine_B.BusAssignment.input.Psi_fcu_deg;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.armed.NAV = rtb_armed;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.armed.LOC = rtb_armed_n;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.NAV =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_NAV;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.LOC_CPT =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.LOC_TRACK =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_TRACK;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.LAND = rtb_AND6;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.FLARE = rtb_AND9;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.ROLL_OUT = rtb_AND7;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.GA_TRACK = AutopilotStateMachine_P.Constant6_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.RWY = AutopilotStateMachine_P.Constant4_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.condition.RWY_TRACK = AutopilotStateMachine_P.Constant5_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output = AutopilotStateMachine_B.out_g;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.ALT_push =
    AutopilotStateMachine_B.BusAssignment.input.ALT_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.ALT_pull =
    AutopilotStateMachine_B.BusAssignment.input.ALT_pull;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.VS_push = AutopilotStateMachine_B.BusAssignment.input.VS_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.VS_pull = AutopilotStateMachine_B.BusAssignment.input.VS_pull;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.LOC_push =
    AutopilotStateMachine_B.BusAssignment.input.LOC_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.APPR_push =
    AutopilotStateMachine_B.BusAssignment.input.APPR_push;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.H_fcu_ft =
    AutopilotStateMachine_B.BusAssignment.input.H_fcu_ft;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.H_dot_fcu_fpm =
    AutopilotStateMachine_B.BusAssignment.input.H_dot_fcu_fpm;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.input.FPA_fcu_deg =
    AutopilotStateMachine_B.BusAssignment.input.FPA_fcu_deg;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.ALT = rtb_AND8;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.ALT_CST = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.CLB = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.DES = AutopilotStateMachine_P.Constant2_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.GS = rtb_armed_g;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.ALT = rtb_Compare_i;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.ALT_CPT = rtb_RelationalOperator;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.ALT_CST = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.ALT_CST_CPT = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.CLB = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.DES = AutopilotStateMachine_P.Constant3_Value;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.GS_CPT = rtb_AND5;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.GS_TRACK = rtb_AND3;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.LAND = rtb_AND6;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.FLARE = rtb_AND9;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.ROLL_OUT = rtb_AND7;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.SRS = rtb_AND10;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.condition.THR_RED = rtb_RelationalOperator1;
  AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output = AutopilotStateMachine_B.out;
  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_DWork.Delay_DSTATE.lateral.armed.NAV),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion5.chunks[0U], 2);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_DWork.Delay_DSTATE.lateral.armed.LOC),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion1.chunks[0U], 2);
  AutopilotStateMachine_BitShift(static_cast<real_T>(AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.ALT),
    &rtb_GainTheta);
  Double2MultiWord(std::floor(rtb_GainTheta), &rtb_DataTypeConversion.chunks[0U], 2);
  AutopilotStateMachine_BitShift1(static_cast<real_T>(AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.ALT_CST),
    &rtb_GainTheta);
  AutopilotStateMachine_Y.out = AutopilotStateMachine_DWork.Delay_DSTATE;
  AutopilotStateMachine_Y.out.output.lateral_law = static_cast<int32_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.law);
  AutopilotStateMachine_Y.out.output.lateral_mode = static_cast<int32_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.mode);
  MultiWordIor(&rtb_DataTypeConversion5.chunks[0U], &rtb_DataTypeConversion1.chunks[0U], &tmp_0.chunks[0U], 2);
  AutopilotStateMachine_Y.out.output.lateral_mode_armed = uMultiWord2Double(&tmp_0.chunks[0U], 2, 0);
  AutopilotStateMachine_Y.out.output.vertical_law = static_cast<int32_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.law);
  AutopilotStateMachine_Y.out.output.vertical_mode = static_cast<int32_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.mode);
  Double2MultiWord(std::floor(rtb_GainTheta), &tmp_5.chunks[0U], 2);
  MultiWordIor(&rtb_DataTypeConversion.chunks[0U], &tmp_5.chunks[0U], &tmp_4.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.CLB), 2))), &tmp_5.chunks[0U], 2);
  MultiWordIor(&tmp_4.chunks[0U], &tmp_5.chunks[0U], &tmp_3.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.DES), 3))), &tmp_4.chunks[0U], 2);
  MultiWordIor(&tmp_3.chunks[0U], &tmp_4.chunks[0U], &tmp_2.chunks[0U], 2);
  Double2MultiWord(static_cast<real_T>(static_cast<int32_T>(std::ldexp(static_cast<real_T>
    (AutopilotStateMachine_DWork.Delay_DSTATE.vertical.armed.GS), 4))), &tmp_3.chunks[0U], 2);
  MultiWordIor(&tmp_2.chunks[0U], &tmp_3.chunks[0U], &tmp_1.chunks[0U], 2);
  AutopilotStateMachine_Y.out.output.vertical_mode_armed = uMultiWord2Double(&tmp_1.chunks[0U], 2, 0);
  AutopilotStateMachine_Y.out.output.Psi_c_deg = AutopilotStateMachine_DWork.Delay_DSTATE.lateral.output.Psi_c_deg;
  AutopilotStateMachine_Y.out.output.H_c_ft = AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.H_c_ft;
  AutopilotStateMachine_Y.out.output.H_dot_c_fpm = AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.H_dot_c_fpm;
  AutopilotStateMachine_Y.out.output.FPA_c_deg = AutopilotStateMachine_DWork.Delay_DSTATE.vertical.output.FPA_c_deg;
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
  AutopilotStateMachine_DWork.is_active_c5_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c5_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c3_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c3_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c6_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c6_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c4_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c4_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_B.in = AutopilotStateMachine_P.out_Y0_d;
  AutopilotStateMachine_DWork.is_ON_c = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_LOC = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c1_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c1_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c7_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c7_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_B.in_i = AutopilotStateMachine_P.out_Y0_l;
  AutopilotStateMachine_B.in_l = AutopilotStateMachine_P.out_Y0;
  AutopilotStateMachine_DWork.is_ON = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_GS = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
  AutopilotStateMachine_DWork.is_active_c2_AutopilotStateMachine = 0U;
  AutopilotStateMachine_DWork.is_c2_AutopilotStateMachine = AutopilotStateMachine_IN_NO_ACTIVE_CHILD;
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
