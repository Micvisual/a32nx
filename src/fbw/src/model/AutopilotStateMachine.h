#ifndef RTW_HEADER_AutopilotStateMachine_h_
#define RTW_HEADER_AutopilotStateMachine_h_
#include <cmath>
#include <cstring>
#ifndef AutopilotStateMachine_COMMON_INCLUDES_
# define AutopilotStateMachine_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "AutopilotStateMachine_types.h"

#include "multiword_types.h"

typedef struct {
  ap_sm_output BusAssignment;
  ap_vertical_output out;
  ap_vertical_input BusConversion_InsertedFor_VerticalMode_at_inport_3_BusCreator1;
  ap_lateral_output out_g;
  real_T in;
  real_T in_i;
  real_T in_l;
} BlockIO_AutopilotStateMachine_T;

typedef struct {
  ap_sm_output Delay_DSTATE;
  boolean_T DelayInput1_DSTATE;
  boolean_T DelayInput1_DSTATE_b;
  boolean_T DelayInput1_DSTATE_d;
  boolean_T DelayInput1_DSTATE_e;
  boolean_T DelayInput1_DSTATE_g;
  boolean_T DelayInput1_DSTATE_f;
  boolean_T DelayInput1_DSTATE_i;
  boolean_T DelayInput1_DSTATE_bd;
  boolean_T DelayInput1_DSTATE_a;
  boolean_T DelayInput1_DSTATE_fn;
  boolean_T DelayInput1_DSTATE_h;
  uint8_T is_active_c2_AutopilotStateMachine;
  uint8_T is_c2_AutopilotStateMachine;
  uint8_T is_ON;
  uint8_T is_GS;
  uint8_T is_active_c6_AutopilotStateMachine;
  uint8_T is_c6_AutopilotStateMachine;
  uint8_T is_active_c7_AutopilotStateMachine;
  uint8_T is_c7_AutopilotStateMachine;
  uint8_T is_active_c4_AutopilotStateMachine;
  uint8_T is_c4_AutopilotStateMachine;
  uint8_T is_active_c1_AutopilotStateMachine;
  uint8_T is_c1_AutopilotStateMachine;
  uint8_T is_ON_c;
  uint8_T is_LOC;
  uint8_T is_active_c3_AutopilotStateMachine;
  uint8_T is_c3_AutopilotStateMachine;
  uint8_T is_active_c5_AutopilotStateMachine;
  uint8_T is_c5_AutopilotStateMachine;
} D_Work_AutopilotStateMachine_T;

typedef struct {
  ap_sm_input in;
} ExternalInputs_AutopilotStateMachine_T;

typedef struct {
  ap_sm_output out;
} ExternalOutputs_AutopilotStateMachine_T;

struct Parameters_AutopilotStateMachine_T_ {
  ap_sm_output ap_sm_output_MATLABStruct;
  real_T CompareToConstant35_const;
  real_T CompareToConstant36_const;
  real_T CompareToConstant_const;
  real_T CompareToConstant1_const;
  real_T CompareToConstant2_const;
  real_T CompareToConstant13_const;
  real_T CompareToConstant24_const;
  real_T CompareToConstant34_const;
  real_T CompareToConstant1_const_m;
  real_T CompareToConstant2_const_o;
  real_T CompareToConstant5_const;
  real_T CompareToConstant19_const;
  real_T CompareToConstant6_const;
  real_T CompareToConstant7_const;
  real_T CompareToConstant8_const;
  real_T CompareToConstant33_const;
  real_T CompareToConstant3_const;
  real_T CompareToConstant4_const;
  real_T CompareToConstant32_const;
  lateral_mode CompareToConstant5_const_o;
  lateral_mode CompareToConstant3_const_f;
  lateral_mode CompareToConstant4_const_k;
  lateral_mode CompareToConstant12_const;
  lateral_mode CompareToConstant20_const;
  lateral_mode CompareToConstant23_const;
  lateral_mode CompareToConstant25_const;
  lateral_mode CompareToConstant15_const;
  lateral_mode CompareToConstant17_const;
  vertical_mode CompareToConstant11_const;
  vertical_mode CompareToConstant21_const;
  vertical_mode CompareToConstant22_const;
  vertical_mode CompareToConstant26_const;
  vertical_mode CompareToConstant14_const;
  vertical_mode CompareToConstant18_const;
  vertical_mode CompareToConstant16_const;
  vertical_mode CompareToConstant27_const;
  vertical_mode CompareToConstant28_const;
  vertical_mode CompareToConstant29_const;
  vertical_mode CompareToConstant30_const;
  vertical_mode CompareToConstant9_const;
  vertical_mode CompareToConstant10_const;
  boolean_T DetectIncrease_vinit;
  boolean_T DetectIncrease1_vinit;
  boolean_T DetectIncrease2_vinit;
  boolean_T DetectIncrease3_vinit;
  boolean_T DetectIncrease4_vinit;
  boolean_T DetectIncrease5_vinit;
  boolean_T DetectIncrease6_vinit;
  boolean_T DetectIncrease7_vinit;
  boolean_T DetectIncrease8_vinit;
  boolean_T DetectIncrease9_vinit;
  boolean_T DetectIncrease10_vinit;
  ap_sm_output Delay_InitialCondition;
  real_T out_Y0;
  real_T out_Y0_l;
  real_T out_Y0_d;
  real_T GainTheta_Gain;
  real_T GainTheta1_Gain;
  real_T Gain_Gain;
  real_T Gainqk_Gain;
  real_T Gain_Gain_a;
  real_T Gain_Gain_k;
  real_T Gainpk_Gain;
  real_T Gain_Gain_af;
  real_T Constant1_Value;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Gain2_Gain;
  real_T Constant10_Value;
  boolean_T Constant1_Value_e;
  boolean_T Constant6_Value;
  boolean_T Constant4_Value;
  boolean_T Constant5_Value;
  boolean_T Constant2_Value;
  boolean_T Constant3_Value;
};

extern const ap_sm_input AutopilotStateMachine_rtZap_sm_input;
extern const ap_sm_output AutopilotStateMachine_rtZap_sm_output;
class AutopilotStateMachineModelClass {
 public:
  ExternalInputs_AutopilotStateMachine_T AutopilotStateMachine_U;
  ExternalOutputs_AutopilotStateMachine_T AutopilotStateMachine_Y;
  void initialize();
  void step();
  void terminate();
  AutopilotStateMachineModelClass();
  ~AutopilotStateMachineModelClass();
 private:
  static Parameters_AutopilotStateMachine_T AutopilotStateMachine_P;
  BlockIO_AutopilotStateMachine_T AutopilotStateMachine_B;
  D_Work_AutopilotStateMachine_T AutopilotStateMachine_DWork;
  void AutopilotStateMachine_BitShift(real_T rtu_u, real_T *rty_y);
  void AutopilotStateMachine_BitShift1(real_T rtu_u, real_T *rty_y);
  void AutopilotStateMachine_NAV_entry(void);
  void AutopilotStateMachine_HDG_entry(void);
  void AutopilotStateMachine_HDG_during(const ap_lateral_input
    *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  void AutopilotStateMachine_LOC_CPT_entry(void);
  void AutopilotStateMachine_OFF_entry(void);
  void AutopilotStateMachine_ROLL_OUT_entry(void);
  void AutopilotStateMachine_FLARE_entry(void);
  void AutopilotStateMachine_LOC_TRACK_entry(void);
  void AutopilotStateMachine_LAND_entry(void);
  void AutopilotStateMachine_GA_TRK_entry(const ap_lateral_input
    *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  void AutopilotStateMachine_RWY_TRK_entry(const ap_lateral_input
    *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  void AutopilotStateMachine_ON(const ap_lateral_armed *BusConversion_InsertedFor_LateralMode_at_inport_2_BusCreator1,
    const ap_lateral_condition *BusConversion_InsertedFor_LateralMode_at_inport_3_BusCreator1, const ap_lateral_input
    *BusConversion_InsertedFor_LateralMode_at_inport_4_BusCreator1);
  void AutopilotStateMachine_RWY_entry(void);
  void AutopilotStateMachine_VS_during(void);
  void AutopilotStateMachine_ALT_entry(void);
  void AutopilotStateMachine_DES_entry(void);
  void AutopilotStateMachine_CLB_entry(void);
  void AutopilotStateMachine_OP_CLB_entry(void);
  void AutopilotStateMachine_OP_DES_entry(void);
  void AutopilotStateMachine_GS_CPT_entry(void);
  void AutopilotStateMachine_ALT_CPT_entry(void);
  void AutopilotStateMachine_ALT(const ap_vertical_armed *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
    const ap_vertical_condition *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  void AutopilotStateMachine_VS_entry(void);
  void AutopilotStateMachine_ALT_CPT(const ap_vertical_armed
    *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
    *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  void AutopilotStateMachine_ALT_CST_entry(void);
  void AutopilotStateMachine_ALT_CST_CPT(const ap_vertical_armed
    *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
    *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  void AutopilotStateMachine_CLB_during(void);
  void AutopilotStateMachine_ALT_CST_CPT_entry(void);
  void AutopilotStateMachine_CLB(const ap_vertical_armed *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1,
    const ap_vertical_condition *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  void AutopilotStateMachine_DES_during(void);
  void AutopilotStateMachine_OFF_entry_g(void);
  void AutopilotStateMachine_ROLL_OUT_entry_b(void);
  void AutopilotStateMachine_GS_TRACK_entry(void);
  void AutopilotStateMachine_LAND_entry_m(void);
  void AutopilotStateMachine_FLARE_entry_b(void);
  void AutopilotStateMachine_SRS_entry(void);
  void AutopilotStateMachine_GS(const ap_vertical_condition
    *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
  void AutopilotStateMachine_OP_CLB_during(void);
  void AutopilotStateMachine_exit_internal_ON(void);
  void AutopilotStateMachine_ON_n(const ap_vertical_armed
    *BusConversion_InsertedFor_VerticalMode_at_inport_4_BusCreator1, const ap_vertical_condition
    *BusConversion_InsertedFor_VerticalMode_at_inport_5_BusCreator1);
};

#endif

