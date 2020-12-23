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
  real_T Delay_DSTATE_n;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_k;
  real_T Delay_DSTATE_nk;
  real_T Delay_DSTATE_h;
  real_T Delay1_DSTATE_b;
  real_T Delay_DSTATE_n4;
  real_T Delay_DSTATE_l;
  real_T Delay1_DSTATE_k;
  real_T Delay_DSTATE_f;
  uint8_T icLoad;
  uint8_T icLoad_i;
  uint8_T is_active_c4_Autopilot;
  uint8_T is_c4_Autopilot;
  uint8_T is_active_c1_Autopilot;
  uint8_T is_c1_Autopilot;
  uint8_T is_active_c2_Autopilot;
  uint8_T is_c2_Autopilot;
  uint8_T is_active_c3_Autopilot;
  uint8_T is_c3_Autopilot;
  boolean_T IC_FirstOutputTime;
  boolean_T IC_FirstOutputTime_h;
  boolean_T IC_FirstOutputTime_p;
  boolean_T IC_FirstOutputTime_k;
  rtDW_Chart_Autopilot_T sf_Chart_f;
  rtDW_Chart_Autopilot_T sf_Chart_b;
  rtDW_Chart_Autopilot_T sf_Chart_o;
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
  real_T LagFilter1_C1;
  real_T LagFilter_C1_p;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T RateLimiterVariableTs_InitialCondition;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T RateLimiterVariableTs_InitialCondition_i;
  real_T RateLimiterVariableTs_lo;
  real_T RateLimiterVariableTs_lo_h;
  real_T RateLimiterVariableTs_up;
  real_T RateLimiterVariableTs_up_k;
  real_T Gain_Gain;
  real_T Gain_Gain_b;
  real_T Gain_Gain_by;
  real_T Gain1_Gain;
  real_T Gain1_Gain_b;
  real_T Gain2_Gain;
  real_T Gain1_Gain_k;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T Gain_Gain_a;
  real_T Gain1_Gain_c;
  real_T Gain2_Gain_k;
  real_T Gain1_Gain_h;
  real_T msftmin_Gain_p;
  real_T ftmintoms_Gain_c;
  real_T Gain_Gain_o;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_g;
  real_T Switch_Threshold;
  real_T Gain1_Gain_kd;
  real_T Gain2_Gain_k1;
  real_T Gain1_Gain_o;
  real_T msftmin_Gain_d;
  real_T Gain_Gain_m;
  real_T ftmintoms_Gain_m;
  real_T Gain_Gain_j;
  real_T Gain1_Gain_a;
  real_T Gain2_Gain_kp;
  real_T Gain1_Gain_i;
  real_T msftmin_Gain_e;
  real_T Gain_Gain_ml;
  real_T Saturation_UpperSat_i;
  real_T Saturation_LowerSat_f;
  real_T ftmintoms_Gain_j;
  real_T Gain_Gain_n;
  real_T Gain1_Gain_cl;
  real_T Gain_Gain_jt;
  real_T Constant3_Value;
  real_T Gain_Gain_h;
  real_T Constant_Value;
  real_T Constant3_Value_e;
  real_T Gain_Gain_l;
  real_T Constant_Value_k;
  real_T Constant3_Value_a;
  real_T Constant_Value_j;
  real_T Gain_Gain_aa;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_g;
  real_T Gain1_Gain_p;
  real_T Gain_Gain_bp;
  real_T Constant_Value_g;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Gain_Gain_byk;
  real_T Constant2_Value;
  real_T Gain1_Gain_ih;
  real_T Delay_InitialCondition;
  real_T Constant_Value_m;
  real_T Delay1_InitialCondition;
  real_T Saturation_UpperSat_c;
  real_T Saturation_LowerSat_fa;
  real_T Constant_Value_d;
  real_T Constant1_Value;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T kntoms_Gain_e;
  real_T IC_Value_p;
  real_T Gain3_Gain;
  real_T Delay_InitialCondition_c;
  real_T Constant_Value_i;
  real_T Delay1_InitialCondition_e;
  real_T kntoms_Gain_a;
  real_T IC_Value_n;
  real_T kntoms_Gain_p;
  real_T IC_Value_pe;
  real_T Gain_Gain_nw;
  real_T Constant2_Value_e;
  real_T Gain1_Gain_d;
  real_T Delay_InitialCondition_o;
  real_T Constant_Value_ig;
  real_T Delay1_InitialCondition_n;
  real_T Saturation_UpperSat_a;
  real_T Saturation_LowerSat_j;
  real_T Constant_Value_p;
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

