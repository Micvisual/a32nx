#include "Autopilot.h"
#include "Autopilot_private.h"

const uint8_T Autopilot_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T Autopilot_IN_any = 1U;
const uint8_T Autopilot_IN_left = 2U;
const uint8_T Autopilot_IN_right = 3U;
const uint8_T Autopilot_IN_InAir = 1U;
const uint8_T Autopilot_IN_NO_ACTIVE_CHILD_n = 0U;
const uint8_T Autopilot_IN_OnGround = 2U;
const uint8_T Autopilot_IN_OFF = 1U;
const uint8_T Autopilot_IN_ON = 2U;
const uint8_T Autopilot_IN_FLARE = 1U;
const uint8_T Autopilot_IN_GA_TRK = 1U;
const uint8_T Autopilot_IN_HDG = 2U;
const uint8_T Autopilot_IN_LAND = 2U;
const uint8_T Autopilot_IN_LOC = 3U;
const uint8_T Autopilot_IN_LOC_CPT = 3U;
const uint8_T Autopilot_IN_LOC_TRACK = 4U;
const uint8_T Autopilot_IN_NAV = 4U;
const uint8_T Autopilot_IN_ROLL_OUT = 5U;
const uint8_T Autopilot_IN_RWY = 5U;
const uint8_T Autopilot_IN_RWY_TRK = 6U;
const uint8_T Autopilot_IN_ALT = 1U;
const uint8_T Autopilot_IN_ALT_CPT = 2U;
const uint8_T Autopilot_IN_ALT_CST = 3U;
const uint8_T Autopilot_IN_ALT_CST_CPT = 4U;
const uint8_T Autopilot_IN_CLB = 5U;
const uint8_T Autopilot_IN_DES = 6U;
const uint8_T Autopilot_IN_GS = 7U;
const uint8_T Autopilot_IN_GS_CPT = 2U;
const uint8_T Autopilot_IN_GS_TRACK = 3U;
const uint8_T Autopilot_IN_LAND_k = 4U;
const uint8_T Autopilot_IN_OP_CLB = 8U;
const uint8_T Autopilot_IN_OP_DES = 9U;
const uint8_T Autopilot_IN_SRS = 10U;
const uint8_T Autopilot_IN_VS = 11U;
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

const ap_input Autopilot_rtZap_input = { { 0.0, 0.0 }, { false, false, false, false, false, false, false, false, false,
    false, false, 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

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
  localDW->is_active_c8_Autopilot = 0U;
  localDW->is_c8_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD;
}

void AutopilotModelClass::Autopilot_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T *rty_out,
  rtDW_Chart_Autopilot_T *localDW)
{
  real_T tmp;
  real_T tmp_0;
  if (localDW->is_active_c8_Autopilot == 0U) {
    localDW->is_active_c8_Autopilot = 1U;
    localDW->is_c8_Autopilot = Autopilot_IN_any;
    if (std::abs(rtu_left) < std::abs(rtu_right)) {
      *rty_out = rtu_left;
    } else {
      *rty_out = rtu_right;
    }
  } else {
    switch (localDW->is_c8_Autopilot) {
     case Autopilot_IN_any:
      tmp = std::abs(rtu_right);
      tmp_0 = std::abs(rtu_left);
      if ((rtu_use_short_path == 0.0) && (tmp < tmp_0) && (tmp >= 10.0) && (tmp <= 20.0)) {
        localDW->is_c8_Autopilot = Autopilot_IN_right;
        *rty_out = rtu_right;
      } else if ((rtu_use_short_path == 0.0) && (tmp_0 < tmp) && (tmp_0 >= 10.0) && (tmp_0 <= 20.0)) {
        localDW->is_c8_Autopilot = Autopilot_IN_left;
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
        localDW->is_c8_Autopilot = Autopilot_IN_any;
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
        localDW->is_c8_Autopilot = Autopilot_IN_any;
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

void AutopilotModelClass::Autopilot_NAV_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_NAV;
  Autopilot_B.out_g.law = lateral_law_HPATH;
}

void AutopilotModelClass::Autopilot_HDG_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_HDG;
  Autopilot_B.out_g.law = lateral_law_HDG;
}

void AutopilotModelClass::Autopilot_HDG_during(const base_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  Autopilot_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
}

void AutopilotModelClass::Autopilot_LOC_CPT_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_LOC_CPT;
  Autopilot_B.out_g.law = lateral_law_LOC_CPT;
}

void AutopilotModelClass::Autopilot_OFF_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_NONE;
  Autopilot_B.out_g.law = lateral_law_NONE;
}

void AutopilotModelClass::Autopilot_ROLL_OUT_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_ROLL_OUT;
  Autopilot_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotModelClass::Autopilot_FLARE_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_FLARE;
  Autopilot_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotModelClass::Autopilot_LOC_TRACK_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_LOC_TRACK;
  Autopilot_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotModelClass::Autopilot_LAND_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_LAND;
  Autopilot_B.out_g.law = lateral_law_LOC_TRACK;
}

void AutopilotModelClass::Autopilot_GA_TRK_entry(const base_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  Autopilot_B.out_g.mode = lateral_mode_GA_TRACK;
  Autopilot_B.out_g.law = lateral_law_TRACK;
  Autopilot_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
}

void AutopilotModelClass::Autopilot_RWY_TRK_entry(const base_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  Autopilot_B.out_g.mode = lateral_mode_RWY_TRACK;
  Autopilot_B.out_g.law = lateral_law_TRACK;
  Autopilot_B.out_g.Psi_c_deg = BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->Psi_fcu_deg;
}

void AutopilotModelClass::Autopilot_ON(const base_lateral_armed
  *BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1, const base_lateral_condition
  *BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1, const base_lateral_input
  *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1)
{
  boolean_T guard1 = false;
  if ((!Autopilot_B.BusAssignment.data.ap_fd_active) || (Autopilot_B.BusAssignment.data.flight_phase >= 9.0) ||
      (Autopilot_B.BusAssignment.data.flight_phase <= 2.0)) {
    Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
    Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
    Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
    Autopilot_OFF_entry();
  } else {
    guard1 = false;
    switch (Autopilot_DWork.is_ON_c) {
     case Autopilot_IN_GA_TRK:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
        Autopilot_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          Autopilot_DWork.is_ON_c = Autopilot_IN_NAV;
          Autopilot_NAV_entry();
        }
      }
      break;

     case Autopilot_IN_HDG:
      if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
           BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
          (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
           BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_NAV;
        Autopilot_NAV_entry();
      } else if (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->LOC &&
                 BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_LOC;
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
        Autopilot_LOC_CPT_entry();
      } else {
        Autopilot_HDG_during(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      }
      break;

     case Autopilot_IN_LOC:
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->GA_TRACK) {
        Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
        Autopilot_DWork.is_ON_c = Autopilot_IN_GA_TRK;
        Autopilot_GA_TRK_entry(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      } else {
        switch (Autopilot_DWork.is_LOC) {
         case Autopilot_IN_FLARE:
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->ROLL_OUT) {
            Autopilot_DWork.is_LOC = Autopilot_IN_ROLL_OUT;
            Autopilot_ROLL_OUT_entry();
          }
          break;

         case Autopilot_IN_LAND:
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->FLARE) {
            Autopilot_DWork.is_LOC = Autopilot_IN_FLARE;
            Autopilot_FLARE_entry();
          } else {
            if (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LAND) {
              if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
                Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
                Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
                Autopilot_HDG_entry();
              } else {
                if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
                  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
                  Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
                  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
                  Autopilot_OFF_entry();
                }
              }
            }
          }
          break;

         case Autopilot_IN_LOC_CPT:
          if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->LOC_push ||
              BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->APPR_push ||
              (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT)) {
            if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
              Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
              Autopilot_HDG_entry();
            } else if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
              Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
              Autopilot_OFF_entry();
            } else {
              guard1 = true;
            }
          } else {
            guard1 = true;
          }
          break;

         case Autopilot_IN_LOC_TRACK:
          if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LAND) {
            Autopilot_DWork.is_LOC = Autopilot_IN_LAND;
            Autopilot_LAND_entry();
          } else {
            if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->LOC_push ||
                BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->APPR_push ||
                (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_TRACK)) {
              if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
                Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
                Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
                Autopilot_HDG_entry();
              } else {
                if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
                  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
                  Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
                  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
                  Autopilot_OFF_entry();
                }
              }
            }
          }
          break;

         default:
          if (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->ROLL_OUT) {
            if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
              Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
              Autopilot_HDG_entry();
            } else {
              if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
                Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
                Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
                Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
                Autopilot_OFF_entry();
              }
            }
          }
          break;
        }
      }
      break;

     case Autopilot_IN_NAV:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull ||
          (!BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
        Autopilot_HDG_entry();
      } else {
        if (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->LOC &&
            BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_CPT) {
          Autopilot_DWork.is_ON_c = Autopilot_IN_LOC;
          Autopilot_DWork.is_LOC = Autopilot_IN_LOC_CPT;
          Autopilot_LOC_CPT_entry();
        }
      }
      break;

     case Autopilot_IN_RWY:
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->RWY_TRACK) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_RWY_TRK;
        Autopilot_RWY_TRK_entry(BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      } else if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
        Autopilot_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          Autopilot_DWork.is_ON_c = Autopilot_IN_NAV;
          Autopilot_NAV_entry();
        }
      }
      break;

     default:
      if (BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_pull) {
        Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
        Autopilot_HDG_entry();
      } else {
        if ((BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1->HDG_push &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV) ||
            (BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1->NAV &&
             BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->NAV)) {
          Autopilot_DWork.is_ON_c = Autopilot_IN_NAV;
          Autopilot_NAV_entry();
        }
      }
      break;
    }

    if (guard1) {
      if (BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1->LOC_TRACK) {
        Autopilot_DWork.is_LOC = Autopilot_IN_LOC_TRACK;
        Autopilot_LOC_TRACK_entry();
      }
    }
  }
}

void AutopilotModelClass::Autopilot_RWY_entry(void)
{
  Autopilot_B.out_g.mode = lateral_mode_RWY;
  Autopilot_B.out_g.law = lateral_law_ROLL_OUT;
}

void AutopilotModelClass::Autopilot_VS_during(void)
{
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  Autopilot_B.out.H_dot_c_fpm = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_dot_fcu_fpm;
}

void AutopilotModelClass::Autopilot_ALT_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_ALT;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_ALT_HOLD;
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotModelClass::Autopilot_DES_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_DES;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_VS;
}

void AutopilotModelClass::Autopilot_CLB_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_CLB;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_VS;
}

void AutopilotModelClass::Autopilot_OP_CLB_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_OP_CLB;
  if (std::abs(Autopilot_B.BusAssignment.data.H_ind_ft -
               Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) <= 1200.0) {
    Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
    Autopilot_B.out.law = vertical_law_VS;
    Autopilot_B.out.H_dot_c_fpm = 1000.0;
  } else {
    Autopilot_B.out.mode_autothrust = athr_mode_THRUST_CLB;
    Autopilot_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotModelClass::Autopilot_OP_DES_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_OP_DES;
  if (std::abs(Autopilot_B.BusAssignment.data.H_ind_ft -
               Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) <= 1200.0) {
    Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
    Autopilot_B.out.law = vertical_law_VS;
    Autopilot_B.out.H_dot_c_fpm = -1000.0;
  } else {
    Autopilot_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
    Autopilot_B.out.law = vertical_law_SPD_MACH;
  }
}

void AutopilotModelClass::Autopilot_GS_CPT_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_GS_CPT;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_GS;
}

void AutopilotModelClass::Autopilot_ALT_CPT_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_ALT_CPT;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_ALT_ACQ;
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotModelClass::Autopilot_ALT(const base_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
      BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_GS;
    Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
    Autopilot_GS_CPT_entry();
  } else {
    tmp = std::abs(Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   Autopilot_B.BusAssignment.data.H_ind_ft);
    if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
        (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
         Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_DES;
      Autopilot_OP_DES_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_CLB;
      Autopilot_OP_CLB_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_CLB;
      Autopilot_CLB_entry();
    } else {
      if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
          (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
           Autopilot_B.BusAssignment.data.H_ind_ft)) {
        Autopilot_DWork.is_ON = Autopilot_IN_DES;
        Autopilot_DES_entry();
      }
    }
  }
}

void AutopilotModelClass::Autopilot_VS_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_VS;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_VS;
}

void AutopilotModelClass::Autopilot_ALT_CPT(const base_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
      BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_GS;
    Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
    Autopilot_GS_CPT_entry();
  } else {
    tmp = std::abs(Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   Autopilot_B.BusAssignment.data.H_ind_ft);
    if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
        (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
         Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_DES;
      Autopilot_OP_DES_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_CLB;
      Autopilot_OP_CLB_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_CLB;
      Autopilot_CLB_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_DES;
      Autopilot_DES_entry();
    } else if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
      Autopilot_DWork.is_ON = Autopilot_IN_VS;
      Autopilot_VS_entry();
    } else {
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT;
        Autopilot_ALT_entry();
      }
    }
  }
}

void AutopilotModelClass::Autopilot_ALT_CST_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_ALT_CST;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_ALT_HOLD;
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotModelClass::Autopilot_ALT_CST_CPT(const base_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST) {
    Autopilot_DWork.is_ON = Autopilot_IN_ALT_CST;
    Autopilot_ALT_CST_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_GS;
    Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
    Autopilot_GS_CPT_entry();
  } else {
    tmp = std::abs(Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                   Autopilot_B.BusAssignment.data.H_ind_ft);
    if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
        (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
         Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_DES;
      Autopilot_OP_DES_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_CLB;
      Autopilot_OP_CLB_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
               (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                Autopilot_B.BusAssignment.data.H_ind_ft)) {
      Autopilot_DWork.is_ON = Autopilot_IN_CLB;
      Autopilot_CLB_entry();
    } else {
      if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0) &&
          (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
           Autopilot_B.BusAssignment.data.H_ind_ft)) {
        Autopilot_DWork.is_ON = Autopilot_IN_DES;
        Autopilot_DES_entry();
      }
    }
  }
}

void AutopilotModelClass::Autopilot_CLB_during(void)
{
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  Autopilot_B.out.H_dot_c_fpm = 1500.0;
}

void AutopilotModelClass::Autopilot_ALT_CST_CPT_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_ALT_CST_CPT;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_ALT_ACQ;
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
}

void AutopilotModelClass::Autopilot_CLB(const base_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_ALT_CST_CPT;
    Autopilot_ALT_CST_CPT_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_ALT_CPT;
    Autopilot_ALT_CPT_entry();
  } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
             Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
    Autopilot_DWork.is_ON = Autopilot_IN_VS;
    Autopilot_VS_entry();
  } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
             BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
    Autopilot_DWork.is_ON = Autopilot_IN_GS;
    Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
    Autopilot_GS_CPT_entry();
  } else {
    Autopilot_CLB_during();
  }
}

void AutopilotModelClass::Autopilot_DES_during(void)
{
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  Autopilot_B.out.H_dot_c_fpm = -1500.0;
}

void AutopilotModelClass::Autopilot_OFF_entry_g(void)
{
  Autopilot_B.out.mode = vertical_mode_NONE;
  Autopilot_B.out.law = vertical_law_NONE;
  Autopilot_B.out.mode_autothrust = athr_mode_NONE;
}

void AutopilotModelClass::Autopilot_ROLL_OUT_entry_b(void)
{
  Autopilot_B.out.mode = vertical_mode_ROLL_OUT;
  Autopilot_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
  Autopilot_B.out.law = vertical_law_FLARE;
}

void AutopilotModelClass::Autopilot_GS_TRACK_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_GS_TRACK;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_GS;
}

void AutopilotModelClass::Autopilot_LAND_entry_m(void)
{
  Autopilot_B.out.mode = vertical_mode_LAND;
  Autopilot_B.out.mode_autothrust = athr_mode_SPEED;
  Autopilot_B.out.law = vertical_law_GS;
}

void AutopilotModelClass::Autopilot_FLARE_entry_b(void)
{
  Autopilot_B.out.mode = vertical_mode_FLARE;
  Autopilot_B.out.mode_autothrust = athr_mode_THRUST_IDLE;
  Autopilot_B.out.law = vertical_law_FLARE;
}

void AutopilotModelClass::Autopilot_SRS_entry(void)
{
  Autopilot_B.out.mode = vertical_mode_SRS;
  Autopilot_B.out.law = vertical_law_SRS;
  Autopilot_B.out.mode_autothrust = athr_mode_NONE;
}

void AutopilotModelClass::Autopilot_GS(const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  boolean_T guard1 = false;
  if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->SRS) {
    Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
    Autopilot_DWork.is_ON = Autopilot_IN_SRS;
    Autopilot_SRS_entry();
  } else {
    guard1 = false;
    switch (Autopilot_DWork.is_GS) {
     case Autopilot_IN_FLARE:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ROLL_OUT) {
        Autopilot_DWork.is_GS = Autopilot_IN_ROLL_OUT;
        Autopilot_ROLL_OUT_entry_b();
      }
      break;

     case Autopilot_IN_GS_CPT:
      if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push ||
          Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push ||
          (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT)) {
        if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
          Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
          Autopilot_DWork.is_ON = Autopilot_IN_VS;
          Autopilot_VS_entry();
        } else if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
          Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
          Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
          Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
          Autopilot_OFF_entry_g();
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      break;

     case Autopilot_IN_GS_TRACK:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->LAND) {
        Autopilot_DWork.is_GS = Autopilot_IN_LAND_k;
        Autopilot_LAND_entry_m();
      } else {
        if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push ||
            Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push ||
            (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_TRACK)) {
          if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
            Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
            Autopilot_DWork.is_ON = Autopilot_IN_VS;
            Autopilot_VS_entry();
          } else {
            if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
              Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
              Autopilot_OFF_entry_g();
            }
          }
        }
      }
      break;

     case Autopilot_IN_LAND_k:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->FLARE) {
        Autopilot_DWork.is_GS = Autopilot_IN_FLARE;
        Autopilot_FLARE_entry_b();
      } else {
        if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->LAND) {
          if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
            Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
            Autopilot_DWork.is_ON = Autopilot_IN_VS;
            Autopilot_VS_entry();
          } else {
            if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
              Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
              Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
              Autopilot_OFF_entry_g();
            }
          }
        }
      }
      break;

     default:
      if (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ROLL_OUT) {
        if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
          Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
          Autopilot_DWork.is_ON = Autopilot_IN_VS;
          Autopilot_VS_entry();
        } else {
          if (Autopilot_B.BusAssignment.data.on_ground != 0.0) {
            Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
            Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
            Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
            Autopilot_OFF_entry_g();
          }
        }
      }
      break;
    }

    if (guard1) {
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_TRACK) {
        Autopilot_DWork.is_GS = Autopilot_IN_GS_TRACK;
        Autopilot_GS_TRACK_entry();
      }
    }
  }
}

void AutopilotModelClass::Autopilot_OP_CLB_during(void)
{
  Autopilot_B.out.H_c_ft = Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft;
  if (std::abs(Autopilot_B.BusAssignment.data.H_ind_ft -
               Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft) > 1200.0) {
    Autopilot_B.out.mode_autothrust = athr_mode_THRUST_CLB;
    Autopilot_B.out.law = vertical_law_SPD_MACH;
    Autopilot_B.out.H_dot_c_fpm = 0.0;
  }
}

void AutopilotModelClass::Autopilot_exit_internal_ON(void)
{
  Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
}

void AutopilotModelClass::Autopilot_ON_n(const base_vertical_armed
  *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const base_vertical_condition
  *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1)
{
  real_T tmp;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  if ((!Autopilot_B.BusAssignment.data.ap_fd_active) || (Autopilot_B.BusAssignment.data.flight_phase >= 9.0) ||
      (Autopilot_B.BusAssignment.data.flight_phase <= 2.0)) {
    Autopilot_exit_internal_ON();
    Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
    Autopilot_OFF_entry_g();
  } else {
    guard1 = false;
    guard2 = false;
    switch (Autopilot_DWork.is_ON) {
     case Autopilot_IN_ALT:
      Autopilot_ALT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
                    BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case Autopilot_IN_ALT_CPT:
      Autopilot_ALT_CPT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
                        BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case Autopilot_IN_ALT_CST:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      }
      break;

     case Autopilot_IN_ALT_CST_CPT:
      Autopilot_ALT_CST_CPT(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
                            BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case Autopilot_IN_CLB:
      Autopilot_CLB(BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
                    BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case Autopilot_IN_DES:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CST_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT_CST_CPT;
        Autopilot_ALT_CST_CPT_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT_CPT;
        Autopilot_ALT_CPT_entry();
      } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        Autopilot_DWork.is_ON = Autopilot_IN_VS;
        Autopilot_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      } else {
        Autopilot_DES_during();
      }
      break;

     case Autopilot_IN_GS:
      Autopilot_GS(BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
      break;

     case Autopilot_IN_OP_CLB:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT_CPT;
        Autopilot_ALT_CPT_entry();
      } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        Autopilot_DWork.is_ON = Autopilot_IN_VS;
        Autopilot_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      } else {
        Autopilot_OP_CLB_during();
      }
      break;

     case Autopilot_IN_OP_DES:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT_CPT;
        Autopilot_ALT_CPT_entry();
      } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push ||
                 Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull) {
        Autopilot_DWork.is_ON = Autopilot_IN_VS;
        Autopilot_VS_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      } else {
        Autopilot_OP_CLB_during();
      }
      break;

     case Autopilot_IN_SRS:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->CLB &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->CLB &&
          (Autopilot_B.BusAssignment.data.flight_phase == 6.0)) {
        guard1 = true;
      } else if ((Autopilot_B.BusAssignment.data.on_ground != 0.0) &&
                 (!BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->SRS)) {
        Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
        Autopilot_OFF_entry_g();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      } else {
        tmp = std::abs(Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                       Autopilot_B.BusAssignment.data.H_ind_ft);
        if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
            (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
             Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_OP_DES;
          Autopilot_OP_DES_entry();
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          guard2 = true;
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          guard1 = true;
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_DES;
          Autopilot_DES_entry();
        } else {
          if ((!BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->CLB) &&
              (Autopilot_B.BusAssignment.data.flight_phase == 6.0)) {
            guard2 = true;
          }
        }
      }
      break;

     default:
      if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->ALT &&
          BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_ALT_CPT;
        Autopilot_ALT_CPT_entry();
      } else if (BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1->GS &&
                 BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->GS_CPT) {
        Autopilot_DWork.is_ON = Autopilot_IN_GS;
        Autopilot_DWork.is_GS = Autopilot_IN_GS_CPT;
        Autopilot_GS_CPT_entry();
      } else {
        tmp = std::abs(Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft -
                       Autopilot_B.BusAssignment.data.H_ind_ft);
        if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0) &&
            (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
             Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_OP_DES;
          Autopilot_OP_DES_entry();
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_OP_CLB;
          Autopilot_OP_CLB_entry();
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft >
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_CLB;
          Autopilot_CLB_entry();
        } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push && (tmp >= 100.0)
                   && (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft <
                       Autopilot_B.BusAssignment.data.H_ind_ft)) {
          Autopilot_DWork.is_ON = Autopilot_IN_DES;
          Autopilot_DES_entry();
        } else if (BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1->ALT) {
          Autopilot_DWork.is_ON = Autopilot_IN_ALT;
          Autopilot_ALT_entry();
        } else {
          Autopilot_VS_during();
        }
      }
      break;
    }

    if (guard2) {
      Autopilot_DWork.is_ON = Autopilot_IN_OP_CLB;
      Autopilot_OP_CLB_entry();
    }

    if (guard1) {
      Autopilot_DWork.is_ON = Autopilot_IN_CLB;
      Autopilot_CLB_entry();
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
  real_T rtb_Minup;
  real_T rtb_Sum_m5;
  boolean_T rtb_RelationalOperator1;
  boolean_T rtb_AND7;
  boolean_T rtb_AND10;
  boolean_T rtb_AND3;
  boolean_T rtb_AND5;
  boolean_T rtb_AND9;
  boolean_T rtb_AND6;
  boolean_T rtb_RelationalOperator_l;
  boolean_T rtb_Compare_c3;
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
  real_T rtb_Switch;
  real_T rtb_ROLLLIM1;
  real_T rtb_Switch_mx;
  real_T rtb_Sum_fl;
  real_T rtb_out_ik;
  real_T rtb_out_nk;
  real_T rtb_out_me;
  real_T rtb_out_i;
  real_T rtb_out_g;
  real_T rtb_Sum1_l5;
  base_lateral_armed BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1;
  base_lateral_condition BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1;
  base_lateral_input BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1;
  base_vertical_armed BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1;
  base_vertical_condition BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1;
  real_T tmp[9];
  Autopilot_DWork.DelayInput1_DSTATE = (static_cast<int32_T>(Autopilot_U.in.input.AP_1_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE));
  rtb_AND7 = (static_cast<int32_T>(Autopilot_U.in.input.AP_DISCONNECT_push) > static_cast<int32_T>
              (Autopilot_DWork.DelayInput1_DSTATE_d));
  Autopilot_DWork.DelayInput1_DSTATE_h = (static_cast<int32_T>(Autopilot_U.in.input.APPR_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_h));
  rtb_GainTheta = Autopilot_P.GainTheta_Gain * Autopilot_U.in.data.Theta_deg;
  rtb_GainTheta1 = Autopilot_P.GainTheta1_Gain * Autopilot_U.in.data.Phi_deg;
  rtb_Saturation = 0.017453292519943295 * rtb_GainTheta;
  rtb_ROLLLIM1 = 0.017453292519943295 * rtb_GainTheta1;
  rtb_Switch = std::tan(rtb_Saturation);
  rtb_Saturation1 = std::sin(rtb_ROLLLIM1);
  rtb_ROLLLIM1 = std::cos(rtb_ROLLLIM1);
  tmp[0] = 1.0;
  tmp[3] = rtb_Saturation1 * rtb_Switch;
  tmp[6] = rtb_ROLLLIM1 * rtb_Switch;
  tmp[1] = 0.0;
  tmp[4] = rtb_ROLLLIM1;
  tmp[7] = -rtb_Saturation1;
  tmp[2] = 0.0;
  rtb_Saturation = 1.0 / std::cos(rtb_Saturation);
  tmp[5] = rtb_Saturation * rtb_Saturation1;
  tmp[8] = rtb_Saturation * rtb_ROLLLIM1;
  rtb_Saturation = Autopilot_P.Gain_Gain_kc * Autopilot_U.in.data.p_rad_s * Autopilot_P.Gainpk_Gain;
  rtb_Saturation1 = Autopilot_P.Gain_Gain_lv * Autopilot_U.in.data.q_rad_s * Autopilot_P.Gainqk_Gain;
  rtb_Switch = Autopilot_P.Gain_Gain_aq * Autopilot_U.in.data.r_rad_s;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_Switch + (tmp[rtb_on_ground + 3] * rtb_Saturation1 +
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

  Autopilot_B.BusAssignment = Autopilot_P.ap_output_MATLABStruct;
  Autopilot_B.BusAssignment.time = Autopilot_U.in.time;
  Autopilot_B.BusAssignment.mode = Autopilot_U.in.mode;
  Autopilot_B.BusAssignment.input.AP_1_push = Autopilot_DWork.DelayInput1_DSTATE;
  Autopilot_B.BusAssignment.input.AP_2_push = (static_cast<int32_T>(Autopilot_U.in.input.AP_2_push) >
    static_cast<int32_T>(Autopilot_DWork.DelayInput1_DSTATE_b));
  Autopilot_B.BusAssignment.input.AP_DISCONNECT_push = rtb_AND7;
  Autopilot_B.BusAssignment.input.HDG_push = (static_cast<int32_T>(Autopilot_U.in.input.HDG_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_e));
  Autopilot_B.BusAssignment.input.HDG_pull = (static_cast<int32_T>(Autopilot_U.in.input.HDG_pull) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_g));
  Autopilot_B.BusAssignment.input.ALT_push = (static_cast<int32_T>(Autopilot_U.in.input.ALT_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_f));
  Autopilot_B.BusAssignment.input.ALT_pull = (static_cast<int32_T>(Autopilot_U.in.input.ALT_pull) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_i));
  Autopilot_B.BusAssignment.input.VS_push = (static_cast<int32_T>(Autopilot_U.in.input.VS_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_bd));
  Autopilot_B.BusAssignment.input.VS_pull = (static_cast<int32_T>(Autopilot_U.in.input.VS_pull) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_a));
  Autopilot_B.BusAssignment.input.LOC_push = (static_cast<int32_T>(Autopilot_U.in.input.LOC_push) > static_cast<int32_T>
    (Autopilot_DWork.DelayInput1_DSTATE_fn));
  Autopilot_B.BusAssignment.input.APPR_push = Autopilot_DWork.DelayInput1_DSTATE_h;
  Autopilot_B.BusAssignment.input.Psi_fcu_deg = Autopilot_U.in.input.Psi_fcu_deg;
  Autopilot_B.BusAssignment.input.H_fcu_ft = Autopilot_U.in.input.H_fcu_ft;
  Autopilot_B.BusAssignment.input.H_dot_fcu_fpm = Autopilot_U.in.input.H_dot_fcu_fpm;
  Autopilot_B.BusAssignment.input.FPA_fcu_deg = Autopilot_U.in.input.FPA_fcu_deg;
  Autopilot_B.BusAssignment.data.Theta_deg = rtb_GainTheta;
  Autopilot_B.BusAssignment.data.Phi_deg = rtb_GainTheta1;
  Autopilot_B.BusAssignment.data.qk_deg_s = result[1];
  Autopilot_B.BusAssignment.data.rk_deg_s = result[2];
  Autopilot_B.BusAssignment.data.pk_deg_s = result[0];
  Autopilot_B.BusAssignment.data.V_ias_kn = Autopilot_U.in.data.V_ias_kn;
  Autopilot_B.BusAssignment.data.V_tas_kn = Autopilot_U.in.data.V_tas_kn;
  Autopilot_B.BusAssignment.data.V_mach = Autopilot_U.in.data.V_mach;
  Autopilot_B.BusAssignment.data.V_gnd_kn = Autopilot_U.in.data.V_gnd_kn;
  Autopilot_B.BusAssignment.data.alpha_deg = Autopilot_U.in.data.alpha_deg;
  Autopilot_B.BusAssignment.data.H_ft = Autopilot_U.in.data.H_ft;
  Autopilot_B.BusAssignment.data.H_ind_ft = Autopilot_U.in.data.H_ind_ft;
  Autopilot_B.BusAssignment.data.H_radio_ft = Autopilot_U.in.data.H_radio_ft;
  Autopilot_B.BusAssignment.data.H_dot_ft_min = Autopilot_U.in.data.H_dot_ft_min;
  Autopilot_B.BusAssignment.data.Psi_magnetic_deg = Autopilot_U.in.data.Psi_magnetic_deg;
  Autopilot_B.BusAssignment.data.Psi_magnetic_track_deg = Autopilot_U.in.data.Psi_magnetic_track_deg;
  Autopilot_B.BusAssignment.data.Psi_true_deg = Autopilot_U.in.data.Psi_true_deg;
  Autopilot_B.BusAssignment.data.bx_m_s2 = Autopilot_U.in.data.bx_m_s2;
  Autopilot_B.BusAssignment.data.by_m_s2 = Autopilot_U.in.data.by_m_s2;
  Autopilot_B.BusAssignment.data.bz_m_s2 = Autopilot_U.in.data.bz_m_s2;
  Autopilot_B.BusAssignment.data.ap_fd_active = Autopilot_U.in.data.ap_fd_active;
  Autopilot_B.BusAssignment.data.ap_V_c_kn = Autopilot_U.in.data.ap_V_c_kn;
  Autopilot_B.BusAssignment.data.ap_H_c_ft = Autopilot_U.in.data.ap_H_c_ft;
  Autopilot_B.BusAssignment.data.ap_Psi_c_deg = Autopilot_U.in.data.ap_Psi_c_deg;
  Autopilot_B.BusAssignment.data.ap_H_dot_c_ft_min = Autopilot_U.in.data.ap_H_dot_c_ft_min;
  Autopilot_B.BusAssignment.data.ap_FPA_c_deg = Autopilot_U.in.data.ap_FPA_c_deg;
  Autopilot_B.BusAssignment.data.nav_valid = Autopilot_U.in.data.nav_valid;
  Autopilot_B.BusAssignment.data.nav_loc_deg = Autopilot_U.in.data.nav_loc_deg;
  Autopilot_B.BusAssignment.data.nav_radial_error_deg = Autopilot_U.in.data.nav_radial_error_deg;
  Autopilot_B.BusAssignment.data.nav_dme_nmi = Autopilot_U.in.data.nav_dme_nmi;
  Autopilot_B.BusAssignment.data.nav_gs_error_deg = Autopilot_U.in.data.nav_gs_error_deg;
  Autopilot_B.BusAssignment.data.flight_guidance_xtk_nmi = Autopilot_U.in.data.flight_guidance_xtk_nmi;
  Autopilot_B.BusAssignment.data.flight_guidance_tae_deg = Autopilot_U.in.data.flight_guidance_tae_deg;
  Autopilot_B.BusAssignment.data.flight_phase = Autopilot_U.in.data.flight_phase;
  Autopilot_B.BusAssignment.data.V2_kn = Autopilot_U.in.data.V2_kn;
  Autopilot_B.BusAssignment.data.is_flight_plan_available = Autopilot_U.in.data.is_flight_plan_available;
  Autopilot_B.BusAssignment.data.thrust_reduction_altitude = Autopilot_U.in.data.thrust_reduction_altitude;
  Autopilot_B.BusAssignment.data.thrust_reduction_altitude_go_around =
    Autopilot_U.in.data.thrust_reduction_altitude_go_around;
  Autopilot_B.BusAssignment.data.on_ground = rtb_on_ground;
  Autopilot_B.BusAssignment.data.zeta_deg = Autopilot_P.Gain2_Gain * Autopilot_U.in.data.zeta_pos;
  Autopilot_B.BusAssignment.data.throttle_lever_1_pos = Autopilot_U.in.data.throttle_lever_1_pos;
  Autopilot_B.BusAssignment.data.throttle_lever_2_pos = Autopilot_U.in.data.throttle_lever_2_pos;
  if (Autopilot_DWork.is_active_c3_Autopilot == 0U) {
    Autopilot_DWork.is_active_c3_Autopilot = 1U;
    Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
    Autopilot_B.BusAssignment.output.ap_on = 0.0;
  } else if (Autopilot_DWork.is_c3_Autopilot == Autopilot_IN_OFF) {
    if (Autopilot_DWork.DelayInput1_DSTATE) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_ON;
      Autopilot_B.BusAssignment.output.ap_on = 1.0;
    } else {
      Autopilot_B.BusAssignment.output.ap_on = 0.0;
    }
  } else {
    if (Autopilot_DWork.DelayInput1_DSTATE || rtb_AND7) {
      Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_OFF;
      Autopilot_B.BusAssignment.output.ap_on = 0.0;
    } else {
      Autopilot_B.BusAssignment.output.ap_on = 1.0;
    }
  }

  if (Autopilot_DWork.is_active_c6_Autopilot == 0U) {
    Autopilot_DWork.is_active_c6_Autopilot = 1U;
    Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_OFF;
    rtb_armed = false;
  } else if (Autopilot_DWork.is_c6_Autopilot == Autopilot_IN_OFF) {
    rtb_armed = false;
    if (Autopilot_P.Constant1_Value_en && Autopilot_B.BusAssignment.input.HDG_push &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_NAV) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_CPT) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_TRACK) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LAND) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_FLARE)) {
      Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_ON;
      rtb_armed = true;
    }
  } else {
    rtb_armed = true;
    if (Autopilot_B.BusAssignment.input.HDG_pull || (Autopilot_DWork.Delay_DSTATE.lateral.output.mode ==
         lateral_mode_NAV)) {
      Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_OFF;
      rtb_armed = false;
    }
  }

  if (Autopilot_DWork.is_active_c4_Autopilot == 0U) {
    Autopilot_DWork.is_active_c4_Autopilot = 1U;
    Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OFF;
    rtb_armed_n = false;
  } else if (Autopilot_DWork.is_c4_Autopilot == Autopilot_IN_OFF) {
    rtb_armed_n = false;
    if ((Autopilot_B.BusAssignment.data.on_ground == 0.0) && (Autopilot_B.BusAssignment.input.LOC_push ||
         Autopilot_B.BusAssignment.input.APPR_push) && (Autopilot_DWork.Delay_DSTATE.lateral.output.mode !=
         lateral_mode_LOC_CPT) && (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LOC_TRACK) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_LAND) &&
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode != lateral_mode_FLARE)) {
      Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_ON;
      rtb_armed_n = true;
    }
  } else {
    rtb_armed_n = true;
    if ((Autopilot_B.BusAssignment.input.LOC_push && (!Autopilot_DWork.Delay_DSTATE.vertical.armed.GS)) ||
        (Autopilot_B.BusAssignment.input.APPR_push && Autopilot_DWork.Delay_DSTATE.vertical.armed.GS) ||
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode == lateral_mode_LOC_CPT) ||
        (Autopilot_DWork.Delay_DSTATE.lateral.output.mode == lateral_mode_LOC_TRACK)) {
      Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_OFF;
      rtb_armed_n = false;
    }
  }

  Autopilot_DWork.DelayInput1_DSTATE_h = (Autopilot_B.BusAssignment.data.flight_guidance_xtk_nmi <=
    Autopilot_P.CompareToConstant35_const);
  Autopilot_DWork.DelayInput1_DSTATE_h = (Autopilot_DWork.DelayInput1_DSTATE_h &&
    (Autopilot_B.BusAssignment.data.on_ground == 0.0) && (Autopilot_B.BusAssignment.data.H_radio_ft >=
    Autopilot_P.CompareToConstant36_const));
  rtb_Minup = std::abs(Autopilot_B.BusAssignment.data.nav_radial_error_deg);
  rtb_Compare_c3 = ((rtb_Minup < Autopilot_P.CompareToConstant_const) && Autopilot_B.BusAssignment.data.nav_valid);
  rtb_AND3 = !Autopilot_B.BusAssignment.data.nav_valid;
  if ((rtb_Minup >= Autopilot_P.CompareToConstant1_const) || rtb_AND3 || (!rtb_Compare_c3)) {
    Autopilot_B.in = Autopilot_B.BusAssignment.time.simulation_time;
  }

  rtb_RelationalOperator_l = ((Autopilot_B.BusAssignment.time.simulation_time - Autopilot_B.in >=
    Autopilot_P.CompareToConstant2_const) && ((Autopilot_DWork.Delay_DSTATE.lateral.output.mode ==
    Autopilot_P.CompareToConstant3_const_f) || (Autopilot_DWork.Delay_DSTATE.lateral.output.mode ==
    Autopilot_P.CompareToConstant4_const_k)));
  rtb_AND6 = (((Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant12_const) ||
               (Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant20_const)) &&
              ((Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant11_const) ||
               (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant21_const)) &&
              (Autopilot_B.BusAssignment.data.H_radio_ft <= Autopilot_P.CompareToConstant13_const));
  rtb_AND9 = (((Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant23_const) ||
               (Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant25_const)) &&
              ((Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant22_const) ||
               (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant26_const)) &&
              (Autopilot_B.BusAssignment.data.H_radio_ft <= Autopilot_P.CompareToConstant24_const));
  rtb_AND7 = (((Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant15_const) ||
               (Autopilot_DWork.Delay_DSTATE.lateral.output.mode == Autopilot_P.CompareToConstant17_const)) &&
              ((Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant14_const) ||
               (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant18_const)) &&
              (Autopilot_B.BusAssignment.data.on_ground != 0.0) && (Autopilot_B.BusAssignment.data.V_gnd_kn >
    Autopilot_P.CompareToConstant34_const));
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_NAV = Autopilot_DWork.DelayInput1_DSTATE_h;
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT = rtb_Compare_c3;
  rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_TRACK = rtb_RelationalOperator_l;
  BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1.NAV = rtb_armed;
  BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1.LOC = rtb_armed_n;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.NAV = Autopilot_DWork.DelayInput1_DSTATE_h;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LOC_CPT = rtb_Compare_c3;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LOC_TRACK = rtb_RelationalOperator_l;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.LAND = rtb_AND6;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.FLARE = rtb_AND9;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.ROLL_OUT = rtb_AND7;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.GA_TRACK = Autopilot_P.Constant6_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.RWY = Autopilot_P.Constant4_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1.RWY_TRACK = Autopilot_P.Constant5_Value;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.HDG_push = Autopilot_B.BusAssignment.input.HDG_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.HDG_pull = Autopilot_B.BusAssignment.input.HDG_pull;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.LOC_push = Autopilot_B.BusAssignment.input.LOC_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.APPR_push = Autopilot_B.BusAssignment.input.APPR_push;
  BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1.Psi_fcu_deg =
    Autopilot_B.BusAssignment.input.Psi_fcu_deg;
  if (Autopilot_DWork.is_active_c1_Autopilot == 0U) {
    Autopilot_DWork.is_active_c1_Autopilot = 1U;
    Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_OFF;
    Autopilot_OFF_entry();
  } else if (Autopilot_DWork.is_c1_Autopilot == Autopilot_IN_OFF) {
    if (((Autopilot_B.BusAssignment.data.flight_phase >= 6.0) && Autopilot_B.BusAssignment.input.HDG_pull) ||
        ((Autopilot_B.BusAssignment.data.flight_phase >= 6.0) && Autopilot_B.BusAssignment.data.ap_fd_active &&
         (!rtb_armed))) {
      Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ON;
      Autopilot_DWork.is_ON_c = Autopilot_IN_HDG;
      Autopilot_HDG_entry();
    } else if (rtb_armed && Autopilot_DWork.DelayInput1_DSTATE_h) {
      Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ON;
      Autopilot_DWork.is_ON_c = Autopilot_IN_NAV;
      Autopilot_NAV_entry();
    } else if (Autopilot_P.Constant4_Value) {
      Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ON;
      Autopilot_DWork.is_ON_c = Autopilot_IN_RWY;
      Autopilot_RWY_entry();
    } else {
      if (Autopilot_P.Constant5_Value) {
        Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_ON;
        Autopilot_DWork.is_ON_c = Autopilot_IN_RWY_TRK;
        Autopilot_RWY_TRK_entry(&BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
      }
    }
  } else {
    Autopilot_ON(&BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1,
                 &BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1,
                 &BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  }

  Autopilot_DWork.DelayInput1_DSTATE_h = (Autopilot_DWork.Delay_DSTATE.vertical.output.mode !=
    Autopilot_P.CompareToConstant16_const);
  Autopilot_DWork.DelayInput1_DSTATE_h = (Autopilot_DWork.DelayInput1_DSTATE_h &&
    (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != Autopilot_P.CompareToConstant27_const) &&
    (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != Autopilot_P.CompareToConstant28_const) &&
    (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != Autopilot_P.CompareToConstant29_const) &&
    (Autopilot_DWork.Delay_DSTATE.data.flight_phase >= Autopilot_P.CompareToConstant1_const_m) &&
    (Autopilot_DWork.Delay_DSTATE.data.flight_phase <= Autopilot_P.CompareToConstant2_const_o));
  if (Autopilot_DWork.is_active_c7_Autopilot == 0U) {
    Autopilot_DWork.is_active_c7_Autopilot = 1U;
    Autopilot_DWork.is_c7_Autopilot = Autopilot_IN_OFF;
    rtb_armed_g = false;
  } else if (Autopilot_DWork.is_c7_Autopilot == Autopilot_IN_OFF) {
    rtb_armed_g = false;
    if ((Autopilot_B.BusAssignment.data.on_ground == 0.0) && Autopilot_B.BusAssignment.input.APPR_push &&
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_GS_CPT) &&
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_GS_TRACK) &&
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_LAND) &&
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode != vertical_mode_FLARE)) {
      Autopilot_DWork.is_c7_Autopilot = Autopilot_IN_ON;
      rtb_armed_g = true;
    }
  } else {
    rtb_armed_g = true;
    if (Autopilot_B.BusAssignment.input.LOC_push || Autopilot_B.BusAssignment.input.APPR_push ||
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == vertical_mode_GS_CPT) ||
        (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == vertical_mode_GS_TRACK)) {
      Autopilot_DWork.is_c7_Autopilot = Autopilot_IN_OFF;
      rtb_armed_g = false;
    }
  }

  rtb_Minup = std::abs(Autopilot_B.BusAssignment.input.H_fcu_ft - Autopilot_B.BusAssignment.data.H_ind_ft);
  rtb_Compare_c3 = (rtb_Minup <= Autopilot_P.CompareToConstant5_const);
  rtb_RelationalOperator_l = (rtb_Minup <= std::abs(Autopilot_B.BusAssignment.data.H_dot_ft_min) /
    Autopilot_P.Constant10_Value);
  rtb_Saturation1 = std::abs(Autopilot_B.BusAssignment.data.nav_gs_error_deg);
  rtb_AND5 = ((Autopilot_B.BusAssignment.data.nav_gs_error_deg >= Autopilot_P.CompareToConstant19_const) &&
              (rtb_Saturation1 < Autopilot_P.CompareToConstant6_const) && Autopilot_B.BusAssignment.data.nav_valid);
  if ((rtb_Saturation1 >= Autopilot_P.CompareToConstant7_const) || rtb_AND3 || (!rtb_AND5)) {
    Autopilot_B.in_i = Autopilot_B.BusAssignment.time.simulation_time;
  }

  rtb_AND3 = ((Autopilot_B.BusAssignment.time.simulation_time - Autopilot_B.in_i >= Autopilot_P.CompareToConstant8_const)
              && ((Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant9_const) ||
                  (Autopilot_DWork.Delay_DSTATE.vertical.output.mode == Autopilot_P.CompareToConstant10_const)));
  if (Autopilot_B.BusAssignment.data.on_ground == 0.0) {
    Autopilot_B.in_l = Autopilot_B.BusAssignment.time.simulation_time;
  }

  Autopilot_DWork.DelayInput1_DSTATE = (Autopilot_B.BusAssignment.data.V2_kn > Autopilot_P.CompareToConstant32_const);
  rtb_AND10 = ((Autopilot_B.BusAssignment.time.simulation_time - Autopilot_B.in_l >=
                Autopilot_P.CompareToConstant33_const) && (Autopilot_DWork.Delay_DSTATE.data.flight_phase >=
    Autopilot_P.CompareToConstant3_const) && (Autopilot_DWork.Delay_DSTATE.data.flight_phase <
    Autopilot_P.CompareToConstant4_const) && Autopilot_DWork.DelayInput1_DSTATE);
  rtb_RelationalOperator1 = (Autopilot_B.BusAssignment.data.H_ind_ft >=
    Autopilot_B.BusAssignment.data.thrust_reduction_altitude);
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_push =
    Autopilot_B.BusAssignment.input.ALT_push;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.ALT_pull =
    Autopilot_B.BusAssignment.input.ALT_pull;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_push =
    Autopilot_B.BusAssignment.input.VS_push;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull =
    Autopilot_B.BusAssignment.input.VS_pull;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.LOC_push =
    Autopilot_B.BusAssignment.input.LOC_push;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.APPR_push =
    Autopilot_B.BusAssignment.input.APPR_push;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_fcu_ft =
    Autopilot_B.BusAssignment.input.H_fcu_ft;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.H_dot_fcu_fpm =
    Autopilot_B.BusAssignment.input.H_dot_fcu_fpm;
  Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.FPA_fcu_deg =
    Autopilot_B.BusAssignment.input.FPA_fcu_deg;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.ALT = Autopilot_DWork.DelayInput1_DSTATE_h;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.ALT_CST = Autopilot_P.Constant2_Value_e;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.CLB = Autopilot_P.Constant2_Value_e;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.DES = Autopilot_P.Constant2_Value_e;
  BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1.GS = rtb_armed_g;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT = rtb_Compare_c3;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CPT = rtb_RelationalOperator_l;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CST = Autopilot_P.Constant3_Value_i1;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ALT_CST_CPT = Autopilot_P.Constant3_Value_i1;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.CLB = Autopilot_P.Constant3_Value_i1;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.DES = Autopilot_P.Constant3_Value_i1;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.GS_CPT = rtb_AND5;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.GS_TRACK = rtb_AND3;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.LAND = rtb_AND6;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.FLARE = rtb_AND9;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.ROLL_OUT = rtb_AND7;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.SRS = rtb_AND10;
  BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1.THR_RED = rtb_RelationalOperator1;
  if (Autopilot_DWork.is_active_c2_Autopilot == 0U) {
    Autopilot_DWork.is_active_c2_Autopilot = 1U;
    Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_OFF;
    Autopilot_OFF_entry_g();
  } else if (Autopilot_DWork.is_c2_Autopilot == Autopilot_IN_OFF) {
    if (Autopilot_B.BusAssignment.data.ap_fd_active && (Autopilot_B.BusAssignment.data.on_ground != 0.0) && rtb_AND10) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_ON;
      Autopilot_DWork.is_ON = Autopilot_IN_SRS;
      Autopilot_SRS_entry();
    } else if (Autopilot_B.BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1.VS_pull ||
               ((Autopilot_B.BusAssignment.data.flight_phase >= 6.0) && (Autopilot_B.BusAssignment.data.on_ground == 0.0)
                && Autopilot_B.BusAssignment.data.ap_fd_active && (!Autopilot_P.Constant2_Value_e))) {
      Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_ON;
      Autopilot_DWork.is_ON = Autopilot_IN_VS;
      Autopilot_VS_entry();
    } else {
      if (Autopilot_P.Constant2_Value_e && Autopilot_P.Constant3_Value_i1 &&
          (Autopilot_B.BusAssignment.data.flight_phase == 6.0)) {
        Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_ON;
        Autopilot_DWork.is_ON = Autopilot_IN_CLB;
        Autopilot_CLB_entry();
      }
    }
  } else {
    Autopilot_ON_n(&BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
                   &BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  }

  Autopilot_DWork.Delay_DSTATE = Autopilot_B.BusAssignment;
  Autopilot_DWork.Delay_DSTATE.lateral.input.HDG_push = Autopilot_B.BusAssignment.input.HDG_push;
  Autopilot_DWork.Delay_DSTATE.lateral.input.HDG_pull = Autopilot_B.BusAssignment.input.HDG_pull;
  Autopilot_DWork.Delay_DSTATE.lateral.input.LOC_push = Autopilot_B.BusAssignment.input.LOC_push;
  Autopilot_DWork.Delay_DSTATE.lateral.input.APPR_push = Autopilot_B.BusAssignment.input.APPR_push;
  Autopilot_DWork.Delay_DSTATE.lateral.input.Psi_fcu_deg = Autopilot_B.BusAssignment.input.Psi_fcu_deg;
  Autopilot_DWork.Delay_DSTATE.lateral.armed.NAV = rtb_armed;
  Autopilot_DWork.Delay_DSTATE.lateral.armed.LOC = rtb_armed_n;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.NAV =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_NAV;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.LOC_CPT =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_CPT;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.LOC_TRACK =
    rtb_BusConversion_InsertedFor_BusAssignment_at_inport_3_BusCreator1_o_LOC_TRACK;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.LAND = rtb_AND6;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.FLARE = rtb_AND9;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.ROLL_OUT = rtb_AND7;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.GA_TRACK = Autopilot_P.Constant6_Value;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.RWY = Autopilot_P.Constant4_Value;
  Autopilot_DWork.Delay_DSTATE.lateral.condition.RWY_TRACK = Autopilot_P.Constant5_Value;
  Autopilot_DWork.Delay_DSTATE.lateral.output = Autopilot_B.out_g;
  Autopilot_DWork.Delay_DSTATE.vertical.input.ALT_push = Autopilot_B.BusAssignment.input.ALT_push;
  Autopilot_DWork.Delay_DSTATE.vertical.input.ALT_pull = Autopilot_B.BusAssignment.input.ALT_pull;
  Autopilot_DWork.Delay_DSTATE.vertical.input.VS_push = Autopilot_B.BusAssignment.input.VS_push;
  Autopilot_DWork.Delay_DSTATE.vertical.input.VS_pull = Autopilot_B.BusAssignment.input.VS_pull;
  Autopilot_DWork.Delay_DSTATE.vertical.input.LOC_push = Autopilot_B.BusAssignment.input.LOC_push;
  Autopilot_DWork.Delay_DSTATE.vertical.input.APPR_push = Autopilot_B.BusAssignment.input.APPR_push;
  Autopilot_DWork.Delay_DSTATE.vertical.input.H_fcu_ft = Autopilot_B.BusAssignment.input.H_fcu_ft;
  Autopilot_DWork.Delay_DSTATE.vertical.input.H_dot_fcu_fpm = Autopilot_B.BusAssignment.input.H_dot_fcu_fpm;
  Autopilot_DWork.Delay_DSTATE.vertical.input.FPA_fcu_deg = Autopilot_B.BusAssignment.input.FPA_fcu_deg;
  Autopilot_DWork.Delay_DSTATE.vertical.armed.ALT = Autopilot_DWork.DelayInput1_DSTATE_h;
  Autopilot_DWork.Delay_DSTATE.vertical.armed.ALT_CST = Autopilot_P.Constant2_Value_e;
  Autopilot_DWork.Delay_DSTATE.vertical.armed.CLB = Autopilot_P.Constant2_Value_e;
  Autopilot_DWork.Delay_DSTATE.vertical.armed.DES = Autopilot_P.Constant2_Value_e;
  Autopilot_DWork.Delay_DSTATE.vertical.armed.GS = rtb_armed_g;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.ALT = rtb_Compare_c3;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.ALT_CPT = rtb_RelationalOperator_l;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.ALT_CST = Autopilot_P.Constant3_Value_i1;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.ALT_CST_CPT = Autopilot_P.Constant3_Value_i1;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.CLB = Autopilot_P.Constant3_Value_i1;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.DES = Autopilot_P.Constant3_Value_i1;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.GS_CPT = rtb_AND5;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.GS_TRACK = rtb_AND3;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.LAND = rtb_AND6;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.FLARE = rtb_AND9;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.ROLL_OUT = rtb_AND7;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.SRS = rtb_AND10;
  Autopilot_DWork.Delay_DSTATE.vertical.condition.THR_RED = rtb_RelationalOperator1;
  Autopilot_DWork.Delay_DSTATE.vertical.output = Autopilot_B.out;
  if (Autopilot_P.ManualSwitch_CurrentSetting == 1) {
    rtb_Saturation1 = Autopilot_P.Constant_Value;
  } else {
    rtb_Saturation1 = Autopilot_DWork.Delay_DSTATE.mode.lateral_mode;
  }

  if (rtb_Saturation1 > Autopilot_P.Switch_Threshold_o) {
    rtb_Switch = rtb_Saturation1;
  } else {
    rtb_Switch = static_cast<int32_T>(Autopilot_DWork.Delay_DSTATE.lateral.output.law);
  }

  if (rtb_Switch != Autopilot_P.CompareToConstant_const_k) {
    Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  }

  if (Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi > Autopilot_P.Saturation_UpperSat_o) {
    rtb_Saturation = Autopilot_P.Saturation_UpperSat_o;
  } else if (Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi < Autopilot_P.Saturation_LowerSat_o) {
    rtb_Saturation = Autopilot_P.Saturation_LowerSat_o;
  } else {
    rtb_Saturation = Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi;
  }

  rtb_Saturation1 = std::sin(Autopilot_P.Gain1_Gain_cu * Autopilot_DWork.Delay_DSTATE.data.nav_radial_error_deg) *
    rtb_Saturation * Autopilot_P.Gain2_Gain_g;
  if (rtb_Saturation1 > Autopilot_P.Saturation1_UpperSat_g) {
    rtb_Saturation1 = Autopilot_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_Saturation1 < Autopilot_P.Saturation1_LowerSat_k) {
      rtb_Saturation1 = Autopilot_P.Saturation1_LowerSat_k;
    }
  }

  Autopilot_DWork.Delay_DSTATE_h += Autopilot_P.Gain6_Gain * rtb_Saturation1 *
    Autopilot_P.DiscreteTimeIntegratorVariableTs_Gain * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (Autopilot_DWork.Delay_DSTATE_h > Autopilot_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else {
    if (Autopilot_DWork.Delay_DSTATE_h < Autopilot_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
      Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
    }
  }

  rtb_Saturation = Autopilot_DWork.Delay_DSTATE.data.nav_radial_error_deg +
    Autopilot_DWork.Delay_DSTATE.data.nav_loc_deg;
  rtb_GainTheta = rt_modd((Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_deg - (rt_modd(rt_modd(rtb_Saturation,
    Autopilot_P.Constant3_Value_l) + Autopilot_P.Constant3_Value_l, Autopilot_P.Constant3_Value_l) +
    Autopilot_P.Constant3_Value_c)) + Autopilot_P.Constant3_Value_c, Autopilot_P.Constant3_Value_c);
  rtb_GainTheta1 = rt_modd(Autopilot_P.Constant3_Value_c - rtb_GainTheta, Autopilot_P.Constant3_Value_c);
  if (rtb_GainTheta < rtb_GainTheta1) {
    rtb_GainTheta *= Autopilot_P.Gain1_Gain_k;
  } else {
    rtb_GainTheta = Autopilot_P.Gain_Gain_p * rtb_GainTheta1;
  }

  rtb_GainTheta = rt_modd((rt_modd(rt_modd(((rtb_Saturation1 * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_gnd_kn,
    Autopilot_P.ScheduledGain_BreakpointsForDimension1, Autopilot_P.ScheduledGain_Table, 2U) +
    Autopilot_DWork.Delay_DSTATE_h) + Autopilot_P.Gain1_Gain_fq * rtb_GainTheta) +
    Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_deg, Autopilot_P.Constant3_Value_lz) + Autopilot_P.Constant3_Value_lz,
    Autopilot_P.Constant3_Value_lz) - (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_deg +
    Autopilot_P.Constant3_Value_i)) + Autopilot_P.Constant3_Value_i, Autopilot_P.Constant3_Value_i);
  rtb_GainTheta1 = rt_modd((Autopilot_DWork.Delay_DSTATE.data.nav_loc_deg -
    (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_deg + Autopilot_P.Constant3_Value_l5)) +
    Autopilot_P.Constant3_Value_l5, Autopilot_P.Constant3_Value_l5);
  Autopilot_Chart(rtb_GainTheta1, Autopilot_P.Gain_Gain_ec * rt_modd(Autopilot_P.Constant3_Value_l5 - rtb_GainTheta1,
    Autopilot_P.Constant3_Value_l5), Autopilot_P.Constant2_Value_l, &rtb_Saturation1, &Autopilot_DWork.sf_Chart_i);
  Autopilot_Chart(rtb_GainTheta, Autopilot_P.Gain_Gain_ka * rt_modd(Autopilot_P.Constant3_Value_i - rtb_GainTheta,
    Autopilot_P.Constant3_Value_i), Autopilot_P.Constant1_Value_e, &rtb_GainTheta1, &Autopilot_DWork.sf_Chart_g);
  Autopilot_DWork.DelayInput1_DSTATE_h = (Autopilot_DWork.Delay_DSTATE.data.H_radio_ft <=
    Autopilot_P.CompareToConstant_const_d);
  if (Autopilot_DWork.DelayInput1_DSTATE_h) {
    rtb_GainTheta = Autopilot_P.Gain_Gain_a * rtb_Saturation1;
  } else {
    rtb_GainTheta = Autopilot_P.Constant1_Value;
  }

  rtb_Saturation1 = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter1_C1;
  rtb_Minup = rtb_Saturation1 + Autopilot_P.Constant_Value_d;
  Autopilot_DWork.Delay1_DSTATE = 1.0 / rtb_Minup * (Autopilot_P.Constant_Value_d - rtb_Saturation1) *
    Autopilot_DWork.Delay1_DSTATE + (rtb_GainTheta + Autopilot_DWork.Delay_DSTATE_ho) * (rtb_Saturation1 / rtb_Minup);
  switch (static_cast<int32_T>(rtb_Switch)) {
   case 0:
    rtb_Saturation1 = Autopilot_P.beta_Value_ed;
    break;

   case 1:
    rtb_Saturation1 = Autopilot_P.beta_Value_e;
    break;

   case 2:
    rtb_Saturation1 = Autopilot_P.beta_Value_b;
    break;

   case 3:
    rtb_Saturation1 = Autopilot_P.beta_Value_m;
    break;

   case 4:
    rtb_Saturation1 = Autopilot_P.beta_Value;
    break;

   case 5:
    rtb_Saturation1 = Autopilot_DWork.Delay1_DSTATE;
    break;

   default:
    rtb_Saturation1 = (Autopilot_P.Gain5_Gain * rtb_GainTheta1 + Autopilot_P.Gain_Gain_b *
                       Autopilot_DWork.Delay_DSTATE.data.rk_deg_s) + Autopilot_DWork.Delay_DSTATE.data.zeta_deg;
    break;
  }

  rtb_ROLLLIM1 = look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn, Autopilot_P.ROLLLIM1_bp01Data,
    Autopilot_P.ROLLLIM1_tableData, 4U);
  rtb_GainTheta1 = rt_modd((Autopilot_DWork.Delay_DSTATE.lateral.output.Psi_c_deg -
    (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_deg + Autopilot_P.Constant3_Value_p)) +
    Autopilot_P.Constant3_Value_p, Autopilot_P.Constant3_Value_p);
  Autopilot_Chart(rtb_GainTheta1, Autopilot_P.Gain_Gain_ke * rt_modd(Autopilot_P.Constant3_Value_p - rtb_GainTheta1,
    Autopilot_P.Constant3_Value_p), Autopilot_P.Constant_Value_c, &rtb_out_ik, &Autopilot_DWork.sf_Chart_b);
  rtb_GainTheta1 = rt_modd((Autopilot_DWork.Delay_DSTATE.lateral.output.Psi_c_deg -
    (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_a)) +
    Autopilot_P.Constant3_Value_a, Autopilot_P.Constant3_Value_a);
  Autopilot_Chart(rtb_GainTheta1, Autopilot_P.Gain_Gain_kg * rt_modd(Autopilot_P.Constant3_Value_a - rtb_GainTheta1,
    Autopilot_P.Constant3_Value_a), Autopilot_P.Constant_Value_i, &rtb_out_nk, &Autopilot_DWork.sf_Chart_n);
  rtb_out_i = Autopilot_P.Gain_Gain_n * Autopilot_DWork.Delay_DSTATE.data.flight_guidance_xtk_nmi;
  if (rtb_out_i > Autopilot_P.Saturation_UpperSat_kr) {
    rtb_out_i = Autopilot_P.Saturation_UpperSat_kr;
  } else {
    if (rtb_out_i < Autopilot_P.Saturation_LowerSat_p) {
      rtb_out_i = Autopilot_P.Saturation_LowerSat_p;
    }
  }

  rtb_GainTheta1 = rt_modd((rt_modd(rt_modd((Autopilot_P.Gain2_Gain_f *
    Autopilot_DWork.Delay_DSTATE.data.flight_guidance_tae_deg + rtb_out_i) * Autopilot_P.Gain1_Gain_n +
    Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg, Autopilot_P.Constant3_Value_e) +
    Autopilot_P.Constant3_Value_e, Autopilot_P.Constant3_Value_e) -
    (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_nk)) +
    Autopilot_P.Constant3_Value_nk, Autopilot_P.Constant3_Value_nk);
  Autopilot_Chart(rtb_GainTheta1, Autopilot_P.Gain_Gain_nf * rt_modd(Autopilot_P.Constant3_Value_nk - rtb_GainTheta1,
    Autopilot_P.Constant3_Value_nk), Autopilot_P.Constant_Value_cw, &rtb_out_me, &Autopilot_DWork.sf_Chart);
  rtb_GainTheta1 = rt_modd((Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg - (rt_modd(rt_modd(rtb_Saturation,
    Autopilot_P.Constant3_Value_ik) + Autopilot_P.Constant3_Value_ik, Autopilot_P.Constant3_Value_ik) +
    Autopilot_P.Constant3_Value_j)) + Autopilot_P.Constant3_Value_j, Autopilot_P.Constant3_Value_j);
  rtb_Minup = rt_modd(Autopilot_P.Constant3_Value_j - rtb_GainTheta1, Autopilot_P.Constant3_Value_j);
  if (Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi > Autopilot_P.Saturation_UpperSat_m) {
    rtb_Saturation = Autopilot_P.Saturation_UpperSat_m;
  } else if (Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi < Autopilot_P.Saturation_LowerSat_k) {
    rtb_Saturation = Autopilot_P.Saturation_LowerSat_k;
  } else {
    rtb_Saturation = Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi;
  }

  rtb_out_i = std::sin(Autopilot_P.Gain1_Gain_h * Autopilot_DWork.Delay_DSTATE.data.nav_radial_error_deg) *
    rtb_Saturation * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.nav_dme_nmi,
    Autopilot_P.ScheduledGain_BreakpointsForDimension1_a, Autopilot_P.ScheduledGain_Table_pq, 4U);
  if (rtb_out_i > Autopilot_P.Saturation1_UpperSat_i) {
    rtb_out_i = Autopilot_P.Saturation1_UpperSat_i;
  } else {
    if (rtb_out_i < Autopilot_P.Saturation1_LowerSat_g) {
      rtb_out_i = Autopilot_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_GainTheta1 < rtb_Minup) {
    rtb_GainTheta1 *= Autopilot_P.Gain1_Gain;
  } else {
    rtb_GainTheta1 = Autopilot_P.Gain_Gain * rtb_Minup;
  }

  rtb_Saturation = rt_modd((rt_modd(rt_modd((rtb_out_i + rtb_GainTheta1) * Autopilot_P.Gain3_Gain +
    Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg, Autopilot_P.Constant3_Value_g) +
    Autopilot_P.Constant3_Value_g, Autopilot_P.Constant3_Value_g) -
    (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_ic)) +
    Autopilot_P.Constant3_Value_ic, Autopilot_P.Constant3_Value_ic);
  Autopilot_Chart(rtb_Saturation, Autopilot_P.Gain_Gain_i * rt_modd(Autopilot_P.Constant3_Value_ic - rtb_Saturation,
    Autopilot_P.Constant3_Value_ic), Autopilot_P.Constant_Value_e, &rtb_out_i, &Autopilot_DWork.sf_Chart_c);
  rtb_Saturation = Autopilot_P.DiscreteDerivativeVariableTs_Gain *
    Autopilot_DWork.Delay_DSTATE.data.nav_radial_error_deg;
  rtb_GainTheta1 = (rtb_Saturation - Autopilot_DWork.Delay_DSTATE_a) / Autopilot_DWork.Delay_DSTATE.time.dt *
    Autopilot_P.Gain3_Gain_i + Autopilot_DWork.Delay_DSTATE.data.nav_radial_error_deg;
  rtb_out_g = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter_C1;
  rtb_Minup = rtb_out_g + Autopilot_P.Constant_Value_b2;
  Autopilot_DWork.Delay1_DSTATE_g = 1.0 / rtb_Minup * (Autopilot_P.Constant_Value_b2 - rtb_out_g) *
    Autopilot_DWork.Delay1_DSTATE_g + (rtb_GainTheta1 + Autopilot_DWork.Delay_DSTATE_am) * (rtb_out_g / rtb_Minup);
  rtb_Minup = rt_modd((rt_modd(rt_modd(Autopilot_DWork.Delay1_DSTATE_g * look1_binlxpw
    (Autopilot_DWork.Delay_DSTATE.data.H_radio_ft, Autopilot_P.ScheduledGain_BreakpointsForDimension1_e,
     Autopilot_P.ScheduledGain_Table_pf, 4U) + Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg,
    Autopilot_P.Constant3_Value_f) + Autopilot_P.Constant3_Value_f, Autopilot_P.Constant3_Value_f) -
                       (Autopilot_DWork.Delay_DSTATE.data.Psi_magnetic_track_deg + Autopilot_P.Constant3_Value_k)) +
                      Autopilot_P.Constant3_Value_k, Autopilot_P.Constant3_Value_k);
  Autopilot_Chart(rtb_Minup, Autopilot_P.Gain_Gain_cz * rt_modd(Autopilot_P.Constant3_Value_k - rtb_Minup,
    Autopilot_P.Constant3_Value_k), Autopilot_P.Constant_Value_p1, &rtb_out_g, &Autopilot_DWork.sf_Chart_l);
  switch (static_cast<int32_T>(rtb_Switch)) {
   case 0:
    rtb_out_g = Autopilot_DWork.Delay_DSTATE.data.Phi_deg;
    break;

   case 1:
    rtb_out_g = rtb_out_ik * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_b, Autopilot_P.ScheduledGain_Table_p, 3U);
    break;

   case 2:
    rtb_out_g = rtb_out_nk * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_c, Autopilot_P.ScheduledGain_Table_b, 3U);
    break;

   case 3:
    rtb_out_g = rtb_out_me * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_m, Autopilot_P.ScheduledGain_Table_a, 3U);
    break;

   case 4:
    rtb_out_g = rtb_out_i * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_f, Autopilot_P.ScheduledGain_Table_n, 3U);
    break;

   case 5:
    rtb_out_g *= look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.V_tas_kn,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_j, Autopilot_P.ScheduledGain_Table_m, 3U);
    break;

   default:
    rtb_out_g = Autopilot_P.Constant3_Value;
    break;
  }

  if (rtb_out_g > rtb_ROLLLIM1) {
    rtb_out_g = rtb_ROLLLIM1;
  } else {
    rtb_Switch = Autopilot_P.Gain1_Gain_l * rtb_ROLLLIM1;
    if (rtb_out_g < rtb_Switch) {
      rtb_out_g = rtb_Switch;
    }
  }

  if (Autopilot_DWork.Delay_DSTATE.output.ap_on == 0.0) {
    Autopilot_DWork.icLoad = 1U;
  }

  if (Autopilot_DWork.icLoad != 0) {
    Autopilot_DWork.Delay_DSTATE_hc = Autopilot_DWork.Delay_DSTATE.data.Phi_deg;
  }

  rtb_out_i = rtb_out_g - Autopilot_DWork.Delay_DSTATE_hc;
  rtb_Switch_mx = Autopilot_P.Constant2_Value_h * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_out_i < rtb_Switch_mx) {
    rtb_Switch_mx = rtb_out_i;
  }

  rtb_out_i = Autopilot_P.Gain1_Gain_kf * Autopilot_P.Constant2_Value_h * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_Switch_mx > rtb_out_i) {
    rtb_out_i = rtb_Switch_mx;
  }

  Autopilot_DWork.Delay_DSTATE_hc += rtb_out_i;
  rtb_Sum_m5 = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter_C1_l;
  rtb_Minup = rtb_Sum_m5 + Autopilot_P.Constant_Value_o;
  Autopilot_DWork.Delay1_DSTATE_k = 1.0 / rtb_Minup * (Autopilot_P.Constant_Value_o - rtb_Sum_m5) *
    Autopilot_DWork.Delay1_DSTATE_k + (Autopilot_DWork.Delay_DSTATE_hc + Autopilot_DWork.Delay_DSTATE_e) * (rtb_Sum_m5 /
    rtb_Minup);
  rtb_out_i = Autopilot_DWork.Delay_DSTATE.output.ap_on - Autopilot_DWork.Delay_DSTATE_b;
  rtb_Switch_mx = Autopilot_P.RateLimiterVariableTs_up * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_out_i < rtb_Switch_mx) {
    rtb_Switch_mx = rtb_out_i;
  }

  rtb_out_i = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.RateLimiterVariableTs_lo;
  if (rtb_Switch_mx > rtb_out_i) {
    rtb_out_i = rtb_Switch_mx;
  }

  Autopilot_DWork.Delay_DSTATE_b += rtb_out_i;
  if (Autopilot_DWork.Delay_DSTATE_b > Autopilot_P.Saturation_UpperSat_d1) {
    rtb_Sum_fl = Autopilot_P.Saturation_UpperSat_d1;
  } else if (Autopilot_DWork.Delay_DSTATE_b < Autopilot_P.Saturation_LowerSat_d4) {
    rtb_Sum_fl = Autopilot_P.Saturation_LowerSat_d4;
  } else {
    rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE_b;
  }

  rtb_Switch = Autopilot_DWork.Delay1_DSTATE_k * rtb_Sum_fl;
  rtb_Sum_fl = Autopilot_P.Constant_Value_k - rtb_Sum_fl;
  rtb_Sum_fl *= Autopilot_DWork.Delay_DSTATE.data.Phi_deg;
  Autopilot_Y.out.output.autopilot.Phi_c_deg = rtb_Switch + rtb_Sum_fl;
  Autopilot_Y.out.output.flight_director.Beta_c_deg = rtb_Saturation1;
  Autopilot_Y.out.output.autopilot.Beta_c_deg = rtb_Saturation1;
  Autopilot_Y.out.output.flight_director.Phi_c_deg = rtb_out_g;
  if (Autopilot_P.ManualSwitch_CurrentSetting_b == 1) {
    rtb_Sum_fl = Autopilot_P.Constant_Value_m;
  } else {
    rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE.mode.vertical_mode;
  }

  if (rtb_Sum_fl > Autopilot_P.Switch_Threshold_d) {
    rtb_Switch_mx = rtb_Sum_fl;
  } else {
    rtb_Switch_mx = static_cast<int32_T>(Autopilot_DWork.Delay_DSTATE.vertical.output.law);
  }

  if (rtb_Switch_mx != Autopilot_P.CompareToConstant5_const_e) {
    Autopilot_B.u_n = (Autopilot_DWork.Delay_DSTATE.vertical.output.H_c_ft + Autopilot_DWork.Delay_DSTATE.data.H_ft) -
      Autopilot_DWork.Delay_DSTATE.data.H_ind_ft;
  }

  rtb_Switch = Autopilot_P.DiscreteDerivativeVariableTs_Gain_m * Autopilot_DWork.Delay_DSTATE.data.V_ias_kn;
  rtb_ROLLLIM1 = (rtb_Switch - Autopilot_DWork.Delay_DSTATE_ex) / Autopilot_DWork.Delay_DSTATE.time.dt *
    Autopilot_P.Gain3_Gain_f + Autopilot_DWork.Delay_DSTATE.data.V_ias_kn;
  rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter1_C1_a;
  rtb_Sum_m5 = rtb_Sum_fl + Autopilot_P.Constant_Value_j;
  Autopilot_DWork.Delay1_DSTATE_f = 1.0 / rtb_Sum_m5 * (Autopilot_P.Constant_Value_j - rtb_Sum_fl) *
    Autopilot_DWork.Delay1_DSTATE_f + (rtb_ROLLLIM1 + Autopilot_DWork.Delay_DSTATE_bx) * (rtb_Sum_fl / rtb_Sum_m5);
  rtb_out_i = Autopilot_DWork.Delay1_DSTATE_f - Autopilot_DWork.Delay_DSTATE.data.ap_V_c_kn;
  if (rtb_out_i > Autopilot_P.Saturation_UpperSat_h) {
    rtb_out_i = Autopilot_P.Saturation_UpperSat_h;
  } else {
    if (rtb_out_i < Autopilot_P.Saturation_LowerSat_c) {
      rtb_out_i = Autopilot_P.Saturation_LowerSat_c;
    }
  }

  rtb_Sum_fl = Autopilot_P.Gain1_Gain_bz * rtb_out_i;
  rtb_out_g = Autopilot_P.Gain1_Gain_ch * Autopilot_DWork.Delay_DSTATE.data.alpha_deg;
  rtb_Saturation1 = Autopilot_DWork.Delay_DSTATE.data.bz_m_s2 * std::sin(rtb_out_g);
  rtb_out_g = std::cos(rtb_out_g);
  rtb_out_g *= Autopilot_DWork.Delay_DSTATE.data.bx_m_s2;
  rtb_out_g = (rtb_Saturation1 + rtb_out_g) * Autopilot_P.Gain_Gain_pc * Autopilot_P.Gain_Gain_k1;
  rtb_out_i = Autopilot_P.Gain1_Gain_k0 * std::abs(Autopilot_P.Constant_Value_em) + std::abs(rtb_out_g);
  if (rtb_out_i <= Autopilot_P.Constant1_Value_ef) {
    rtb_out_i = Autopilot_P.Constant1_Value_ef;
  }

  rtb_Saturation1 = std::abs(rtb_Sum_fl);
  if (rtb_Saturation1 < rtb_out_i) {
    rtb_out_i = rtb_Saturation1;
  }

  if (rtb_out_g < 0.0) {
    rtb_out_g = -1.0;
  } else {
    if (rtb_out_g > 0.0) {
      rtb_out_g = 1.0;
    }
  }

  rtb_Sum1_l5 = rtb_Sum_fl - rtb_out_i * rtb_out_g * Autopilot_P.Gain_Gain_h;
  rtb_Sum_fl = Autopilot_P.kntoms_Gain_i * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
  rtb_Saturation1 = std::sin((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_nd *
    Autopilot_DWork.Delay_DSTATE.data.Phi_deg) * Autopilot_DWork.Delay_DSTATE.data.alpha_deg) *
    Autopilot_P.Gain1_Gain_m5) * rtb_Sum_fl * Autopilot_P.msftmin_Gain_h;
  if (rtb_Sum_fl > Autopilot_P.Saturation_UpperSat_j) {
    rtb_Sum_fl = Autopilot_P.Saturation_UpperSat_j;
  } else {
    if (rtb_Sum_fl < Autopilot_P.Saturation_LowerSat_m3) {
      rtb_Sum_fl = Autopilot_P.Saturation_LowerSat_m3;
    }
  }

  rtb_out_i = (Autopilot_P.Constant_Value_ke - rtb_Saturation1) * Autopilot_P.ftmintoms_Gain_g / rtb_Sum_fl;
  rtb_out_ik = Autopilot_P.DiscreteDerivativeVariableTs_Gain_l * Autopilot_DWork.Delay_DSTATE.data.nav_gs_error_deg;
  rtb_out_nk = (rtb_out_ik - Autopilot_DWork.Delay_DSTATE_e3) / Autopilot_DWork.Delay_DSTATE.time.dt *
    Autopilot_P.Gain3_Gain_o + Autopilot_P.Gain1_Gain_j * Autopilot_DWork.Delay_DSTATE.data.nav_gs_error_deg;
  rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter_C1_n;
  rtb_Sum_m5 = rtb_Sum_fl + Autopilot_P.Constant_Value_n;
  Autopilot_DWork.Delay1_DSTATE_j = 1.0 / rtb_Sum_m5 * (Autopilot_P.Constant_Value_n - rtb_Sum_fl) *
    Autopilot_DWork.Delay1_DSTATE_j + (rtb_out_nk + Autopilot_DWork.Delay_DSTATE_f) * (rtb_Sum_fl / rtb_Sum_m5);
  if (Autopilot_DWork.Delay_DSTATE.data.H_radio_ft <= Autopilot_P.CompareToConstant_const_j) {
    Autopilot_B.u = Autopilot_DWork.Delay_DSTATE.data.H_dot_ft_min;
  }

  rtb_out_me = Autopilot_P.DiscreteDerivativeVariableTs_Gain_lf * Autopilot_DWork.Delay_DSTATE.data.V_ias_kn;
  rtb_Minup = (rtb_out_me - Autopilot_DWork.Delay_DSTATE_k) / Autopilot_DWork.Delay_DSTATE.time.dt *
    Autopilot_P.Gain3_Gain_fd + Autopilot_DWork.Delay_DSTATE.data.V_ias_kn;
  rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter1_C1_p;
  rtb_Sum_m5 = rtb_Sum_fl + Autopilot_P.Constant_Value_mm;
  Autopilot_DWork.Delay1_DSTATE_b = 1.0 / rtb_Sum_m5 * (Autopilot_P.Constant_Value_mm - rtb_Sum_fl) *
    Autopilot_DWork.Delay1_DSTATE_b + (rtb_Minup + Autopilot_DWork.Delay_DSTATE_b5) * (rtb_Sum_fl / rtb_Sum_m5);
  rtb_Saturation1 = Autopilot_DWork.Delay1_DSTATE_b - (Autopilot_P.Constant2_Value_m +
    Autopilot_DWork.Delay_DSTATE.data.V2_kn);
  if (rtb_Saturation1 > Autopilot_P.Saturation_UpperSat_nk) {
    rtb_Saturation1 = Autopilot_P.Saturation_UpperSat_nk;
  } else {
    if (rtb_Saturation1 < Autopilot_P.Saturation_LowerSat_h) {
      rtb_Saturation1 = Autopilot_P.Saturation_LowerSat_h;
    }
  }

  rtb_Sum_fl = Autopilot_P.Gain1_Gain_pc * rtb_Saturation1;
  rtb_out_g = Autopilot_P.Gain1_Gain_db * Autopilot_DWork.Delay_DSTATE.data.alpha_deg;
  rtb_Saturation1 = Autopilot_DWork.Delay_DSTATE.data.bz_m_s2 * std::sin(rtb_out_g);
  rtb_out_g = std::cos(rtb_out_g);
  rtb_out_g *= Autopilot_DWork.Delay_DSTATE.data.bx_m_s2;
  rtb_out_g = (rtb_Saturation1 + rtb_out_g) * Autopilot_P.Gain_Gain_pn * Autopilot_P.Gain_Gain_he;
  rtb_Saturation1 = Autopilot_P.Gain1_Gain_k4 * std::abs(Autopilot_P.Constant_Value_jb) + std::abs(rtb_out_g);
  if (rtb_Saturation1 <= Autopilot_P.Constant1_Value_l) {
    rtb_Saturation1 = Autopilot_P.Constant1_Value_l;
  }

  rtb_Sum_m5 = std::abs(rtb_Sum_fl);
  if (rtb_Sum_m5 < rtb_Saturation1) {
    rtb_Saturation1 = rtb_Sum_m5;
  }

  if (rtb_out_g < 0.0) {
    rtb_out_g = -1.0;
  } else {
    if (rtb_out_g > 0.0) {
      rtb_out_g = 1.0;
    }
  }

  rtb_out_g = rtb_Sum_fl - rtb_Saturation1 * rtb_out_g * Autopilot_P.Gain_Gain_hk;
  rtb_Sum_fl = Autopilot_P.kntoms_Gain_f * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
  rtb_Saturation1 = std::sin((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_a *
    Autopilot_DWork.Delay_DSTATE.data.Phi_deg) * Autopilot_DWork.Delay_DSTATE.data.alpha_deg) *
    Autopilot_P.Gain1_Gain_n4) * rtb_Sum_fl;
  if (rtb_Sum_fl > Autopilot_P.Saturation_UpperSat_j3) {
    rtb_Sum_fl = Autopilot_P.Saturation_UpperSat_j3;
  } else {
    if (rtb_Sum_fl < Autopilot_P.Saturation_LowerSat_e) {
      rtb_Sum_fl = Autopilot_P.Saturation_LowerSat_e;
    }
  }

  result[0] = Autopilot_P.Constant1_Value_d - Autopilot_DWork.Delay_DSTATE.data.Theta_deg;
  result[1] = rtb_out_g;
  rtb_Saturation1 = (Autopilot_P.Constant_Value_in - Autopilot_P.msftmin_Gain_d * rtb_Saturation1) *
    Autopilot_P.ftmintoms_Gain_c / rtb_Sum_fl;
  if (rtb_Saturation1 > 1.0) {
    rtb_Saturation1 = 1.0;
  } else {
    if (rtb_Saturation1 < -1.0) {
      rtb_Saturation1 = -1.0;
    }
  }

  result[2] = Autopilot_P.Gain_Gain_kcp * std::asin(rtb_Saturation1) * Autopilot_P.Gain_Gain_e5;
  switch (static_cast<int32_T>(rtb_Switch_mx)) {
   case 0:
    rtb_Sum_fl = Autopilot_DWork.Delay_DSTATE.data.Theta_deg;
    break;

   case 1:
    rtb_Saturation1 = Autopilot_P.kntoms_Gain_n * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
    rtb_out_i = (Autopilot_B.u_n - Autopilot_DWork.Delay_DSTATE.data.H_ft) * Autopilot_P.Gain_Gain_ft;
    if (rtb_out_i > Autopilot_P.Saturation_UpperSat_n) {
      rtb_out_i = Autopilot_P.Saturation_UpperSat_n;
    } else {
      if (rtb_out_i < Autopilot_P.Saturation_LowerSat_d) {
        rtb_out_i = Autopilot_P.Saturation_LowerSat_d;
      }
    }

    if (rtb_Saturation1 > Autopilot_P.Saturation_UpperSat_g) {
      rtb_out_g = Autopilot_P.Saturation_UpperSat_g;
    } else if (rtb_Saturation1 < Autopilot_P.Saturation_LowerSat_g) {
      rtb_out_g = Autopilot_P.Saturation_LowerSat_g;
    } else {
      rtb_out_g = rtb_Saturation1;
    }

    rtb_out_i = (rtb_out_i - std::sin((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_m *
      Autopilot_DWork.Delay_DSTATE.data.Phi_deg) * Autopilot_DWork.Delay_DSTATE.data.alpha_deg) *
      Autopilot_P.Gain1_Gain_c) * rtb_Saturation1 * Autopilot_P.msftmin_Gain_ff) * Autopilot_P.ftmintoms_Gain_j /
      rtb_out_g;
    if (rtb_out_i > 1.0) {
      rtb_out_i = 1.0;
    } else {
      if (rtb_out_i < -1.0) {
        rtb_out_i = -1.0;
      }
    }

    rtb_Sum_fl = Autopilot_P.Gain_Gain_k * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_bb;
    break;

   case 2:
    rtb_Saturation1 = Autopilot_P.kntoms_Gain_e * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
    rtb_out_i = Autopilot_DWork.Delay_DSTATE.vertical.output.H_c_ft - Autopilot_DWork.Delay_DSTATE.data.H_ind_ft;
    if (rtb_out_i < 0.0) {
      rtb_out_g = -1.0;
    } else if (rtb_out_i > 0.0) {
      rtb_out_g = 1.0;
    } else {
      rtb_out_g = rtb_out_i;
    }

    if (rtb_Saturation1 > Autopilot_P.Saturation_UpperSat_d) {
      rtb_Sum_fl = Autopilot_P.Saturation_UpperSat_d;
    } else if (rtb_Saturation1 < Autopilot_P.Saturation_LowerSat_n) {
      rtb_Sum_fl = Autopilot_P.Saturation_LowerSat_n;
    } else {
      rtb_Sum_fl = rtb_Saturation1;
    }

    rtb_out_i = ((Autopilot_P.Constant_Value_b * rtb_out_g + rtb_out_i) * Autopilot_P.Gain_Gain_e - std::sin
                 ((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_b1 *
      Autopilot_DWork.Delay_DSTATE.data.Phi_deg) * Autopilot_DWork.Delay_DSTATE.data.alpha_deg) *
                  Autopilot_P.Gain1_Gain_p) * rtb_Saturation1 * Autopilot_P.msftmin_Gain_f) *
      Autopilot_P.ftmintoms_Gain_le / rtb_Sum_fl;
    if (rtb_out_i > 1.0) {
      rtb_out_i = 1.0;
    } else {
      if (rtb_out_i < -1.0) {
        rtb_out_i = -1.0;
      }
    }

    rtb_Sum_fl = Autopilot_P.Gain_Gain_f * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_lg;
    break;

   case 3:
    if (Autopilot_DWork.Delay_DSTATE.vertical.output.H_c_ft - Autopilot_DWork.Delay_DSTATE.data.H_ind_ft >
        Autopilot_P.Switch_Threshold_k) {
      if (rtb_out_i > 1.0) {
        rtb_out_i = 1.0;
      } else {
        if (rtb_out_i < -1.0) {
          rtb_out_i = -1.0;
        }
      }

      rtb_Sum_fl = Autopilot_P.Gain_Gain_ly * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_d4;
      if (rtb_Sum1_l5 > rtb_Sum_fl) {
        rtb_Sum_fl = rtb_Sum1_l5;
      }
    } else {
      if (rtb_out_i > 1.0) {
        rtb_out_i = 1.0;
      } else {
        if (rtb_out_i < -1.0) {
          rtb_out_i = -1.0;
        }
      }

      rtb_Sum_fl = Autopilot_P.Gain_Gain_ly * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_d4;
      if (rtb_Sum1_l5 < rtb_Sum_fl) {
        rtb_Sum_fl = rtb_Sum1_l5;
      }
    }
    break;

   case 4:
    rtb_Saturation1 = Autopilot_P.kntoms_Gain_a * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
    if (rtb_Saturation1 > Autopilot_P.Saturation_UpperSat_e) {
      rtb_out_i = Autopilot_P.Saturation_UpperSat_e;
    } else if (rtb_Saturation1 < Autopilot_P.Saturation_LowerSat_a) {
      rtb_out_i = Autopilot_P.Saturation_LowerSat_a;
    } else {
      rtb_out_i = rtb_Saturation1;
    }

    rtb_out_i = (Autopilot_DWork.Delay_DSTATE.vertical.output.H_dot_c_fpm - std::sin
                 ((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos(Autopilot_P.Gain1_Gain_g *
      Autopilot_DWork.Delay_DSTATE.data.Phi_deg) * Autopilot_DWork.Delay_DSTATE.data.alpha_deg) *
                  Autopilot_P.Gain1_Gain_dz) * rtb_Saturation1 * Autopilot_P.msftmin_Gain_l) *
      Autopilot_P.ftmintoms_Gain_l / rtb_out_i;
    if (rtb_out_i > 1.0) {
      rtb_out_i = 1.0;
    } else {
      if (rtb_out_i < -1.0) {
        rtb_out_i = -1.0;
      }
    }

    rtb_Sum_fl = Autopilot_P.Gain_Gain_d * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_a3;
    break;

   case 5:
    rtb_Sum_fl = (Autopilot_DWork.Delay_DSTATE.vertical.output.FPA_c_deg - (Autopilot_DWork.Delay_DSTATE.data.Theta_deg
      - std::cos(Autopilot_P.Gain1_Gain_d * Autopilot_DWork.Delay_DSTATE.data.Phi_deg) *
      Autopilot_DWork.Delay_DSTATE.data.alpha_deg)) * Autopilot_P.Gain_Gain_c;
    break;

   case 6:
    rtb_Sum_fl = Autopilot_DWork.Delay1_DSTATE_j * look1_binlxpw(Autopilot_DWork.Delay_DSTATE.data.H_radio_ft,
      Autopilot_P.ScheduledGain_BreakpointsForDimension1_h, Autopilot_P.ScheduledGain_Table_i, 4U);
    break;

   case 7:
    if (Autopilot_DWork.Delay_DSTATE.data.on_ground > Autopilot_P.Switch_Threshold) {
      rtb_Sum_fl = (Autopilot_DWork.Delay_DSTATE.data.Theta_deg - Autopilot_P.Constant2_Value) * Autopilot_P.Gain4_Gain;
    } else {
      rtb_Saturation1 = Autopilot_P.kntoms_Gain * Autopilot_DWork.Delay_DSTATE.data.V_tas_kn;
      rtb_Switch_mx = Autopilot_B.u / Autopilot_P.Constant_Value_p * Autopilot_DWork.Delay_DSTATE.data.H_radio_ft;
      if (Autopilot_P.Constant3_Value_n < rtb_Switch_mx) {
        rtb_Switch_mx = Autopilot_P.Constant3_Value_n;
      }

      if (rtb_Saturation1 > Autopilot_P.Saturation_UpperSat) {
        rtb_out_i = Autopilot_P.Saturation_UpperSat;
      } else if (rtb_Saturation1 < Autopilot_P.Saturation_LowerSat) {
        rtb_out_i = Autopilot_P.Saturation_LowerSat;
      } else {
        rtb_out_i = rtb_Saturation1;
      }

      rtb_out_i = (rtb_Switch_mx - std::sin((Autopilot_DWork.Delay_DSTATE.data.Theta_deg - std::cos
        (Autopilot_P.Gain1_Gain_f * Autopilot_DWork.Delay_DSTATE.data.Phi_deg) *
        Autopilot_DWork.Delay_DSTATE.data.alpha_deg) * Autopilot_P.Gain1_Gain_b) * rtb_Saturation1 *
                   Autopilot_P.msftmin_Gain) * Autopilot_P.ftmintoms_Gain / rtb_out_i;
      if (rtb_out_i > 1.0) {
        rtb_out_i = 1.0;
      } else {
        if (rtb_out_i < -1.0) {
          rtb_out_i = -1.0;
        }
      }

      rtb_Sum_fl = Autopilot_P.Gain_Gain_l * std::asin(rtb_out_i) * Autopilot_P.Gain_Gain_m;
    }
    break;

   default:
    if (result[0] < rtb_out_g) {
      if (rtb_out_g < result[2]) {
        rtb_on_ground = 1;
      } else if (result[0] < result[2]) {
        rtb_on_ground = 2;
      } else {
        rtb_on_ground = 0;
      }
    } else if (result[0] < result[2]) {
      rtb_on_ground = 0;
    } else if (rtb_out_g < result[2]) {
      rtb_on_ground = 2;
    } else {
      rtb_on_ground = 1;
    }

    rtb_Sum_fl = result[rtb_on_ground];
    break;
  }

  rtb_Sum_fl += Autopilot_DWork.Delay_DSTATE.data.Theta_deg;
  if (rtb_Sum_fl > Autopilot_P.Constant1_Value_i) {
    rtb_Sum_fl = Autopilot_P.Constant1_Value_i;
  } else {
    rtb_Saturation1 = Autopilot_P.Gain1_Gain_i * Autopilot_P.Constant1_Value_i;
    if (rtb_Sum_fl < rtb_Saturation1) {
      rtb_Sum_fl = rtb_Saturation1;
    }
  }

  if (Autopilot_DWork.Delay_DSTATE.output.ap_on == 0.0) {
    Autopilot_DWork.icLoad_b = 1U;
  }

  if (Autopilot_DWork.icLoad_b != 0) {
    Autopilot_DWork.Delay_DSTATE_g = Autopilot_DWork.Delay_DSTATE.data.Theta_deg;
  }

  rtb_out_i = rtb_Sum_fl - Autopilot_DWork.Delay_DSTATE_g;
  rtb_Switch_mx = Autopilot_P.Constant2_Value_h1 * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_out_i < rtb_Switch_mx) {
    rtb_Switch_mx = rtb_out_i;
  }

  rtb_out_i = Autopilot_P.Gain1_Gain_ds * Autopilot_P.Constant2_Value_h1 * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_Switch_mx > rtb_out_i) {
    rtb_out_i = rtb_Switch_mx;
  }

  Autopilot_DWork.Delay_DSTATE_g += rtb_out_i;
  rtb_Sum_m5 = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.LagFilter_C1_f;
  rtb_out_g = rtb_Sum_m5 + Autopilot_P.Constant_Value_cc;
  Autopilot_DWork.Delay1_DSTATE_n = 1.0 / rtb_out_g * (Autopilot_P.Constant_Value_cc - rtb_Sum_m5) *
    Autopilot_DWork.Delay1_DSTATE_n + (Autopilot_DWork.Delay_DSTATE_g + Autopilot_DWork.Delay_DSTATE_j) * (rtb_Sum_m5 /
    rtb_out_g);
  rtb_out_i = Autopilot_DWork.Delay_DSTATE.output.ap_on - Autopilot_DWork.Delay_DSTATE_gx;
  rtb_Switch_mx = Autopilot_P.RateLimiterVariableTs_up_i * Autopilot_DWork.Delay_DSTATE.time.dt;
  if (rtb_out_i < rtb_Switch_mx) {
    rtb_Switch_mx = rtb_out_i;
  }

  rtb_out_i = Autopilot_DWork.Delay_DSTATE.time.dt * Autopilot_P.RateLimiterVariableTs_lo_o;
  if (rtb_Switch_mx > rtb_out_i) {
    rtb_out_i = rtb_Switch_mx;
  }

  Autopilot_DWork.Delay_DSTATE_gx += rtb_out_i;
  if (Autopilot_DWork.Delay_DSTATE_gx > Autopilot_P.Saturation_UpperSat_nn) {
    rtb_Sum_m5 = Autopilot_P.Saturation_UpperSat_nn;
  } else if (Autopilot_DWork.Delay_DSTATE_gx < Autopilot_P.Saturation_LowerSat_l) {
    rtb_Sum_m5 = Autopilot_P.Saturation_LowerSat_l;
  } else {
    rtb_Sum_m5 = Autopilot_DWork.Delay_DSTATE_gx;
  }

  rtb_Saturation1 = Autopilot_DWork.Delay1_DSTATE_n * rtb_Sum_m5;
  rtb_Sum_m5 = Autopilot_P.Constant_Value_px - rtb_Sum_m5;
  rtb_Sum_m5 *= Autopilot_DWork.Delay_DSTATE.data.Theta_deg;
  Autopilot_Y.out.output.autopilot.Theta_c_deg = rtb_Saturation1 + rtb_Sum_m5;
  Autopilot_Y.out.time = Autopilot_DWork.Delay_DSTATE.time;
  Autopilot_Y.out.mode = Autopilot_DWork.Delay_DSTATE.mode;
  Autopilot_Y.out.input = Autopilot_DWork.Delay_DSTATE.input;
  Autopilot_Y.out.data = Autopilot_DWork.Delay_DSTATE.data;
  Autopilot_Y.out.lateral = Autopilot_DWork.Delay_DSTATE.lateral;
  Autopilot_Y.out.vertical = Autopilot_DWork.Delay_DSTATE.vertical;
  Autopilot_Y.out.output.ap_on = Autopilot_DWork.Delay_DSTATE.output.ap_on;
  Autopilot_Y.out.output.flight_director.Theta_c_deg = rtb_Sum_fl;
  Autopilot_DWork.DelayInput1_DSTATE = Autopilot_U.in.input.AP_1_push;
  Autopilot_DWork.DelayInput1_DSTATE_b = Autopilot_U.in.input.AP_2_push;
  Autopilot_DWork.DelayInput1_DSTATE_d = Autopilot_U.in.input.AP_DISCONNECT_push;
  Autopilot_DWork.DelayInput1_DSTATE_e = Autopilot_U.in.input.HDG_push;
  Autopilot_DWork.DelayInput1_DSTATE_g = Autopilot_U.in.input.HDG_pull;
  Autopilot_DWork.DelayInput1_DSTATE_f = Autopilot_U.in.input.ALT_push;
  Autopilot_DWork.DelayInput1_DSTATE_i = Autopilot_U.in.input.ALT_pull;
  Autopilot_DWork.DelayInput1_DSTATE_bd = Autopilot_U.in.input.VS_push;
  Autopilot_DWork.DelayInput1_DSTATE_a = Autopilot_U.in.input.VS_pull;
  Autopilot_DWork.DelayInput1_DSTATE_fn = Autopilot_U.in.input.LOC_push;
  Autopilot_DWork.DelayInput1_DSTATE_h = Autopilot_U.in.input.APPR_push;
  Autopilot_DWork.Delay_DSTATE_ho = rtb_GainTheta;
  Autopilot_DWork.Delay_DSTATE_a = rtb_Saturation;
  Autopilot_DWork.Delay_DSTATE_am = rtb_GainTheta1;
  Autopilot_DWork.icLoad = 0U;
  Autopilot_DWork.Delay_DSTATE_e = Autopilot_DWork.Delay_DSTATE_hc;
  Autopilot_DWork.Delay_DSTATE_ex = rtb_Switch;
  Autopilot_DWork.Delay_DSTATE_bx = rtb_ROLLLIM1;
  Autopilot_DWork.Delay_DSTATE_e3 = rtb_out_ik;
  Autopilot_DWork.Delay_DSTATE_f = rtb_out_nk;
  Autopilot_DWork.Delay_DSTATE_k = rtb_out_me;
  Autopilot_DWork.Delay_DSTATE_b5 = rtb_Minup;
  Autopilot_DWork.icLoad_b = 0U;
  Autopilot_DWork.Delay_DSTATE_j = Autopilot_DWork.Delay_DSTATE_g;
}

void AutopilotModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&Autopilot_B)), 0,
                     sizeof(BlockIO_Autopilot_T));
  (void) std::memset(static_cast<void *>(&Autopilot_DWork), 0,
                     sizeof(D_Work_Autopilot_T));
  Autopilot_U.in = Autopilot_rtZap_input;
  Autopilot_Y.out = Autopilot_rtZap_output;
  Autopilot_DWork.DelayInput1_DSTATE = Autopilot_P.DetectIncrease_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_b = Autopilot_P.DetectIncrease1_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_d = Autopilot_P.DetectIncrease2_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_e = Autopilot_P.DetectIncrease3_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_g = Autopilot_P.DetectIncrease4_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_f = Autopilot_P.DetectIncrease5_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_i = Autopilot_P.DetectIncrease6_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_bd = Autopilot_P.DetectIncrease7_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_a = Autopilot_P.DetectIncrease8_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_fn = Autopilot_P.DetectIncrease9_vinit;
  Autopilot_DWork.DelayInput1_DSTATE_h = Autopilot_P.DetectIncrease10_vinit;
  Autopilot_DWork.Delay_DSTATE = Autopilot_P.Delay_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_h = Autopilot_P.DiscreteTimeIntegratorVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_ho = Autopilot_P.Delay_InitialCondition_a;
  Autopilot_DWork.Delay1_DSTATE = Autopilot_P.Delay1_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_a = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_am = Autopilot_P.Delay_InitialCondition_k;
  Autopilot_DWork.Delay1_DSTATE_g = Autopilot_P.Delay1_InitialCondition_f;
  Autopilot_DWork.icLoad = 1U;
  Autopilot_DWork.Delay_DSTATE_e = Autopilot_P.Delay_InitialCondition_g;
  Autopilot_DWork.Delay1_DSTATE_k = Autopilot_P.Delay1_InitialCondition_d;
  Autopilot_DWork.Delay_DSTATE_b = Autopilot_P.RateLimiterVariableTs_InitialCondition;
  Autopilot_DWork.Delay_DSTATE_ex = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_e;
  Autopilot_DWork.Delay_DSTATE_bx = Autopilot_P.Delay_InitialCondition_f;
  Autopilot_DWork.Delay1_DSTATE_f = Autopilot_P.Delay1_InitialCondition_h;
  Autopilot_DWork.Delay_DSTATE_e3 = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_h;
  Autopilot_DWork.Delay_DSTATE_f = Autopilot_P.Delay_InitialCondition_h;
  Autopilot_DWork.Delay1_DSTATE_j = Autopilot_P.Delay1_InitialCondition_e;
  Autopilot_DWork.Delay_DSTATE_k = Autopilot_P.DiscreteDerivativeVariableTs_InitialCondition_b;
  Autopilot_DWork.Delay_DSTATE_b5 = Autopilot_P.Delay_InitialCondition_b;
  Autopilot_DWork.Delay1_DSTATE_b = Autopilot_P.Delay1_InitialCondition_l;
  Autopilot_DWork.icLoad_b = 1U;
  Autopilot_DWork.Delay_DSTATE_j = Autopilot_P.Delay_InitialCondition_b5;
  Autopilot_DWork.Delay1_DSTATE_n = Autopilot_P.Delay1_InitialCondition_h0;
  Autopilot_DWork.Delay_DSTATE_gx = Autopilot_P.RateLimiterVariableTs_InitialCondition_p;
  Autopilot_DWork.is_active_c5_Autopilot = 0U;
  Autopilot_DWork.is_c5_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c3_Autopilot = 0U;
  Autopilot_DWork.is_c3_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c6_Autopilot = 0U;
  Autopilot_DWork.is_c6_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c4_Autopilot = 0U;
  Autopilot_DWork.is_c4_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_B.in = Autopilot_P.out_Y0_d;
  Autopilot_DWork.is_ON_c = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_LOC = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c1_Autopilot = 0U;
  Autopilot_DWork.is_c1_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c7_Autopilot = 0U;
  Autopilot_DWork.is_c7_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_B.in_i = Autopilot_P.out_Y0;
  Autopilot_B.in_l = Autopilot_P.out_Y0_i;
  Autopilot_DWork.is_ON = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_GS = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_DWork.is_active_c2_Autopilot = 0U;
  Autopilot_DWork.is_c2_Autopilot = Autopilot_IN_NO_ACTIVE_CHILD_n;
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_i);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_g);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_b);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_n);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_c);
  Autopilot_Chart_Init(&Autopilot_DWork.sf_Chart_l);
  Autopilot_B.u_n = Autopilot_P.Y_Y0;
  Autopilot_B.u = Autopilot_P.Y_Y0_n;
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
