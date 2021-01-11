#include "AutopilotStateMachine.h"
#include "AutopilotStateMachine_private.h"

Parameters_AutopilotStateMachine_T AutopilotStateMachineModelClass::AutopilotStateMachine_P = {

  {
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
      0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      {
        0,
        0,
        0,
        0,
        0.0
      },

      {
        0,
        0
      },

      {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
      },

      {
        lateral_mode_NONE,
        lateral_law_NONE,
        0.0
      }
    },

    {
      {
        0,
        0,
        0,
        0,
        0,
        0,
        0.0,
        0.0,
        0.0
      },

      {
        0,
        0,
        0,
        0,
        0
      },

      {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
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
  },

  1.0,

  30.0,

  4.0,

  0.2,

  10.0,

  400.0,

  50.0,

  50.0,

  5.0,

  7.0,

  20.0,

  -0.2,

  1.0,

  0.4,

  15.0,

  30.0,

  3.0,

  6.0,

  100.0,

  lateral_mode_LOC_CPT,

  lateral_mode_LOC_CPT,

  lateral_mode_LOC_TRACK,

  lateral_mode_LOC_TRACK,

  lateral_mode_LAND,

  lateral_mode_LAND,

  lateral_mode_FLARE,

  lateral_mode_FLARE,

  lateral_mode_ROLL_OUT,

  vertical_mode_GS_TRACK,

  vertical_mode_LAND,

  vertical_mode_LAND,

  vertical_mode_FLARE,

  vertical_mode_FLARE,

  vertical_mode_ROLL_OUT,

  vertical_mode_ALT_CPT,

  vertical_mode_ALT,

  vertical_mode_GS_CPT,

  vertical_mode_GS_TRACK,

  vertical_mode_GS_CPT,

  vertical_mode_GS_CPT,

  vertical_mode_GS_TRACK,

  0,

  0,

  0,

  0,

  0,

  0,

  0,

  0,

  0,

  0,

  0,


  {
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
  },

  0.0,

  0.0,

  0.0,

  -1.0,

  -1.0,

  57.295779513082323,

  -1.0,

  57.295779513082323,

  57.295779513082323,

  -1.0,

  2.0,

  1.0,

  1.0,

  0.0,

  2.0,

  1.0,

  0.0,

  -25.0,

  8.0,

  1,

  0,

  0,

  0,

  0,

  0
};
