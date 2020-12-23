#ifndef RTW_HEADER_Autopilot_h_
#define RTW_HEADER_Autopilot_h_
#include <cfloat>
#include <cmath>
#include <cstring>
#ifndef Autopilot_COMMON_INCLUDES_
# define Autopilot_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "Autopilot_types.h"

typedef struct {
  uint8_T is_active_c5_Autopilot;
  uint8_T is_c5_Autopilot;
} rtDW_Chart_Autopilot_T;

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_l;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_a;
  real_T Delay_DSTATE_g;
  real_T Delay1_DSTATE_b;
  real_T Delay_DSTATE_p;
  real_T Delay_DSTATE_d;
  real_T Delay1_DSTATE_g;
  real_T Delay_DSTATE_i;
  real_T Delay_DSTATE_n;
  real_T Delay1_DSTATE_e;
  real_T Delay_DSTATE_e;
  real_T Delay_DSTATE_pw;
  real_T Delay1_DSTATE_n;
  real_T Delay_DSTATE_py;
  real_T Delay_DSTATE_b;
  real_T Delay1_DSTATE_e1;
  real_T Delay_DSTATE_m;
  real_T Delay_DSTATE_k;
  real_T Delay1_DSTATE_o;
  real_T Delay_DSTATE_ip;
  real_T Delay_DSTATE_o;
  real_T Delay1_DSTATE_l;
  uint8_T icLoad;
  uint8_T icLoad_a;
  uint8_T icLoad_k;
  uint8_T icLoad_l;
  uint8_T icLoad_aw;
  uint8_T icLoad_m;
  uint8_T icLoad_kc;
  uint8_T is_active_c4_Autopilot;
  uint8_T is_c4_Autopilot;
  uint8_T is_active_c1_Autopilot;
  uint8_T is_c1_Autopilot;
  uint8_T is_active_c2_Autopilot;
  uint8_T is_c2_Autopilot;
  uint8_T is_active_c3_Autopilot;
  uint8_T is_c3_Autopilot;
  boolean_T IC_FirstOutputTime;
  boolean_T IC_FirstOutputTime_p;
  boolean_T IC_FirstOutputTime_c;
  boolean_T IC_FirstOutputTime_cl;
  rtDW_Chart_Autopilot_T sf_Chart_m;
  rtDW_Chart_Autopilot_T sf_Chart_h;
  rtDW_Chart_Autopilot_T sf_Chart_i;
} D_Work_Autopilot_T;

typedef struct {
  ap_input in;
} ExternalInputs_Autopilot_T;

typedef struct {
  ap_output out;
} ExternalOutputs_Autopilot_T;

struct Parameters_Autopilot_T_ {
  ap_output ap_output_MATLABStruct;
  real_T LagFilter_C1;
  real_T LagFilter_C1_o;
  real_T LagFilter_C1_e;
  real_T LagFilter1_C1;
  real_T LagFilter_C1_c;
  real_T LagFilter_C1_k;
  real_T LagFilter_C1_ku;
  real_T LagFilter_C1_l;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T Gain1_Gain;
  real_T Gain1_Gain_a;
  real_T Gain1_Gain_f;
  real_T Gain1_Gain_n;
  real_T Constant1_Value;
  real_T Gain1_Gain_e;
  real_T Gain1_Gain_h;
  real_T msftmin_Gain;
  real_T Gain_Gain;
  real_T ftmintoms_Gain;
  real_T Gain_Gain_j;
  real_T Gain1_Gain_d;
  real_T Constant1_Value_g;
  real_T Gain1_Gain_f3;
  real_T Gain1_Gain_fr;
  real_T msftmin_Gain_f;
  real_T Gain_Gain_m;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T ftmintoms_Gain_b;
  real_T Gain_Gain_d;
  real_T Gain1_Gain_ea;
  real_T Gain2_Gain;
  real_T Gain1_Gain_b;
  real_T msftmin_Gain_p;
  real_T ftmintoms_Gain_c;
  real_T Gain_Gain_l;
  real_T Saturation_UpperSat_a;
  real_T Saturation_LowerSat_d;
  real_T Gain1_Gain_g;
  real_T Gain1_Gain_nh;
  real_T Constant_Value;
  real_T Switch_Threshold;
  real_T Gain1_Gain_j;
  real_T Constant1_Value_e;
  real_T Gain1_Gain_o;
  real_T Gain1_Gain_i;
  real_T msftmin_Gain_o;
  real_T ftmintoms_Gain_f;
  real_T Gain_Gain_ds;
  real_T Gain_Gain_jt;
  real_T Gain2_Gain_j;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Constant3_Value;
  real_T Gain_Gain_i;
  real_T Constant4_Value;
  real_T Gain_Gain_e;
  real_T Constant1_Value_j;
  real_T Gain1_Gain_p;
  real_T Delay_InitialCondition;
  real_T Constant_Value_c;
  real_T Delay1_InitialCondition;
  real_T Gain2_Gain_c;
  real_T ROLLLIM1_tableData_j[5];
  real_T ROLLLIM1_bp01Data_a[5];
  real_T Constant3_Value_m;
  real_T Gain_Gain_d3;
  real_T Constant4_Value_k;
  real_T Gain_Gain_en;
  real_T Constant1_Value_h;
  real_T Gain1_Gain_nw;
  real_T Delay_InitialCondition_h;
  real_T Constant_Value_j;
  real_T Delay1_InitialCondition_a;
  real_T Gain2_Gain_cw;
  real_T ROLLLIM1_tableData_c[5];
  real_T ROLLLIM1_bp01Data_j[5];
  real_T Constant3_Value_j;
  real_T Constant_Value_a;
  real_T Gain_Gain_a;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_g;
  real_T Gain1_Gain_pa;
  real_T Gain_Gain_i0;
  real_T Constant4_Value_d;
  real_T Gain_Gain_l0;
  real_T Constant1_Value_m;
  real_T Gain1_Gain_ee;
  real_T Delay_InitialCondition_m;
  real_T Constant_Value_b;
  real_T Delay1_InitialCondition_g;
  real_T Gain2_Gain_k;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T Gain2_Gain_k1;
  real_T kntoms_Gain_e;
  real_T IC_Value_b;
  real_T Gain1_Gain_gk;
  real_T Gain3_Gain;
  real_T Delay_InitialCondition_f;
  real_T Constant_Value_l;
  real_T Delay1_InitialCondition_f;
  real_T kntoms_Gain_a;
  real_T IC_Value_e;
  real_T Gain2_Gain_k2;
  real_T kntoms_Gain_p;
  real_T IC_Value_h;
  real_T Constant2_Value;
  real_T Gain1_Gain_ab;
  real_T Delay_InitialCondition_g;
  real_T Constant_Value_f;
  real_T Delay1_InitialCondition_l;
  real_T Constant2_Value_k;
  real_T Gain1_Gain_e0;
  real_T Delay_InitialCondition_gs;
  real_T Constant_Value_p;
  real_T Delay1_InitialCondition_o;
  real_T Constant2_Value_p;
  real_T Gain1_Gain_m;
  real_T Delay_InitialCondition_i;
  real_T Constant_Value_d;
  real_T Delay1_InitialCondition_h;
  real_T Constant2_Value_f;
  real_T Gain1_Gain_a2;
  real_T Delay_InitialCondition_o;
  real_T Constant_Value_o;
  real_T Delay1_InitialCondition_p;
};

extern const ap_input Autopilot_rtZap_input;
extern const ap_output Autopilot_rtZap_output;
class AutopilotModelClass {
 public:
  ExternalInputs_Autopilot_T Autopilot_U;
  ExternalOutputs_Autopilot_T Autopilot_Y;
  void initialize();
  void step();
  void terminate();
  AutopilotModelClass();
  ~AutopilotModelClass();
 private:
  static Parameters_Autopilot_T Autopilot_P;
  D_Work_Autopilot_T Autopilot_DWork;
  void Autopilot_Chart_Init(rtDW_Chart_Autopilot_T *localDW);
  void Autopilot_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T *rty_out,
                       rtDW_Chart_Autopilot_T *localDW);
};

#endif

