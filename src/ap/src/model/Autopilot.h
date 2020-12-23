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
  real_T Delay_DSTATE_b;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_k;
  real_T Delay_DSTATE_n;
  real_T Delay_DSTATE_c;
  real_T Delay1_DSTATE_b;
  real_T Delay_DSTATE_o;
  real_T Delay_DSTATE_j;
  real_T Delay1_DSTATE_p;
  real_T Delay_DSTATE_a;
  uint8_T icLoad;
  uint8_T icLoad_n;
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
  boolean_T IC_FirstOutputTime_n;
  boolean_T IC_FirstOutputTime_e;
  rtDW_Chart_Autopilot_T sf_Chart_da;
  rtDW_Chart_Autopilot_T sf_Chart_j;
  rtDW_Chart_Autopilot_T sf_Chart_f;
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
  real_T LagFilter_C1_h;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T RateLimiterVariableTs_InitialCondition;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T RateLimiterVariableTs_InitialCondition_i;
  real_T RateLimiterVariableTs_lo;
  real_T RateLimiterVariableTs_lo_h;
  real_T RateLimiterVariableTs_up;
  real_T RateLimiterVariableTs_up_k;
  real_T Gain_Gain;
  real_T Gain_Gain_h;
  real_T Gain_Gain_p;
  real_T Gain1_Gain;
  real_T Gain1_Gain_k;
  real_T Gain2_Gain;
  real_T Gain1_Gain_kx;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T Gain_Gain_hr;
  real_T Gain1_Gain_h;
  real_T Gain2_Gain_k;
  real_T Gain1_Gain_hy;
  real_T msftmin_Gain_l;
  real_T ftmintoms_Gain_k;
  real_T Gain_Gain_o;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_g;
  real_T Switch_Threshold;
  real_T Gain1_Gain_o;
  real_T Gain2_Gain_k1;
  real_T Gain1_Gain_ol;
  real_T msftmin_Gain_ln;
  real_T Gain_Gain_m;
  real_T ftmintoms_Gain_p;
  real_T Gain_Gain_b;
  real_T Gain1_Gain_i;
  real_T Gain2_Gain_kp;
  real_T Gain1_Gain_d;
  real_T msftmin_Gain_f;
  real_T Gain_Gain_ml;
  real_T Saturation_UpperSat_i;
  real_T Saturation_LowerSat_f;
  real_T ftmintoms_Gain_pe;
  real_T Gain_Gain_c;
  real_T Gain1_Gain_e;
  real_T Gain_Gain_j;
  real_T Constant3_Value;
  real_T Gain_Gain_n;
  real_T Constant_Value;
  real_T Constant3_Value_c;
  real_T Gain_Gain_d;
  real_T Constant_Value_k;
  real_T Constant3_Value_g;
  real_T Constant_Value_f;
  real_T Gain_Gain_a;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_g;
  real_T Gain1_Gain_p;
  real_T Gain_Gain_l;
  real_T Constant_Value_g;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Gain_Gain_by;
  real_T Constant2_Value;
  real_T Gain1_Gain_l;
  real_T Delay_InitialCondition;
  real_T Constant_Value_c;
  real_T Delay1_InitialCondition;
  real_T Saturation_UpperSat_h;
  real_T Saturation_LowerSat_p;
  real_T Constant_Value_e;
  real_T Constant1_Value;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T kntoms_Gain_e;
  real_T IC_Value_o;
  real_T Gain3_Gain;
  real_T Delay_InitialCondition_e;
  real_T Constant_Value_j;
  real_T Delay1_InitialCondition_j;
  real_T kntoms_Gain_a;
  real_T IC_Value_i;
  real_T kntoms_Gain_p;
  real_T IC_Value_g;
  real_T Gain_Gain_nw;
  real_T Constant2_Value_e;
  real_T Gain1_Gain_iq;
  real_T Delay_InitialCondition_n;
  real_T Constant_Value_a;
  real_T Delay1_InitialCondition_e;
  real_T Saturation_UpperSat_il;
  real_T Saturation_LowerSat_fy;
  real_T Constant_Value_i;
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

