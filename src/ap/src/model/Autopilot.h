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
  real_T LAW;
} BlockIO_Autopilot_T;

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_g;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_f;
  real_T Delay_DSTATE_h;
  real_T Delay1_DSTATE_h;
  real_T Delay_DSTATE_k;
  real_T Delay_DSTATE_m;
  real_T Delay_DSTATE_a;
  real_T Delay1_DSTATE_e;
  real_T Delay_DSTATE_at;
  real_T Delay_DSTATE_gp;
  real_T Delay1_DSTATE_j;
  real_T Delay_DSTATE_d;
  real_T Delay_DSTATE_o;
  real_T Delay1_DSTATE_m;
  real_T Delay_DSTATE_l;
  real_T loc_trk_time;
  uint8_T icLoad;
  uint8_T icLoad_b;
  uint8_T is_active_c4_Autopilot;
  uint8_T is_c4_Autopilot;
  uint8_T is_active_c1_Autopilot;
  uint8_T is_c1_Autopilot;
  uint8_T is_active_c2_Autopilot;
  uint8_T is_c2_Autopilot;
  uint8_T is_LOC;
  uint8_T is_active_c3_Autopilot;
  uint8_T is_c3_Autopilot;
  boolean_T IC_FirstOutputTime;
  boolean_T IC_FirstOutputTime_j;
  boolean_T IC_FirstOutputTime_h;
  boolean_T IC_FirstOutputTime_b;
  rtDW_Chart_Autopilot_T sf_Chart_h;
  rtDW_Chart_Autopilot_T sf_Chart_k;
  rtDW_Chart_Autopilot_T sf_Chart_dz;
  rtDW_Chart_Autopilot_T sf_Chart_i;
  rtDW_Chart_Autopilot_T sf_Chart_b;
} D_Work_Autopilot_T;

typedef struct {
  ap_input in;
} ExternalInputs_Autopilot_T;

typedef struct {
  ap_output out;
} ExternalOutputs_Autopilot_T;

struct Parameters_Autopilot_T_ {
  ap_output ap_output_MATLABStruct;
  real_T ScheduledGain_BreakpointsForDimension1[4];
  real_T ScheduledGain_BreakpointsForDimension1_n[4];
  real_T ScheduledGain_BreakpointsForDimension1_nx[4];
  real_T ScheduledGain_BreakpointsForDimension1_e[4];
  real_T ScheduledGain_BreakpointsForDimension1_g[4];
  real_T LagFilter_C1;
  real_T LagFilter_C1_o;
  real_T LagFilter1_C1;
  real_T LagFilter_C1_m;
  real_T LagFilter_C1_n;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteDerivativeVariableTs_Gain_e;
  real_T DiscreteDerivativeVariableTs_Gain_o;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T RateLimiterVariableTs_InitialCondition;
  real_T DiscreteDerivativeVariableTs_InitialCondition_m;
  real_T DiscreteDerivativeVariableTs_InitialCondition_p;
  real_T RateLimiterVariableTs_InitialCondition_i;
  real_T ScheduledGain_Table[4];
  real_T ScheduledGain_Table_n[4];
  real_T ScheduledGain_Table_g[4];
  real_T ScheduledGain_Table_c[4];
  real_T ScheduledGain_Table_m[4];
  real_T RateLimiterVariableTs_lo;
  real_T RateLimiterVariableTs_lo_h;
  real_T RateLimiterVariableTs_up;
  real_T RateLimiterVariableTs_up_k;
  real_T Gain1_Gain;
  real_T Gain4_Gain;
  real_T Gain1_Gain_c;
  real_T Gain2_Gain;
  real_T Gain1_Gain_k;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T Gain_Gain;
  real_T Gain1_Gain_h;
  real_T Gain2_Gain_k;
  real_T Gain1_Gain_cz;
  real_T msftmin_Gain_h;
  real_T ftmintoms_Gain_j;
  real_T Gain_Gain_j;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_g;
  real_T Switch_Threshold;
  real_T Gain1_Gain_ca;
  real_T Gain2_Gain_k1;
  real_T Gain1_Gain_f;
  real_T msftmin_Gain_c;
  real_T Constant_Value;
  real_T Gain_Gain_m;
  real_T ftmintoms_Gain_k;
  real_T Gain_Gain_g;
  real_T Gain1_Gain_i;
  real_T Gain2_Gain_kp;
  real_T Gain1_Gain_k3;
  real_T msftmin_Gain_n;
  real_T Gain_Gain_ml;
  real_T Saturation_UpperSat_i;
  real_T Saturation_LowerSat_f;
  real_T ftmintoms_Gain_m;
  real_T Gain_Gain_p;
  real_T Gain1_Gain_im;
  real_T Gain_Gain_jt;
  real_T Constant3_Value;
  real_T Gain_Gain_b;
  real_T Constant_Value_e;
  real_T Constant3_Value_c;
  real_T Gain_Gain_mr;
  real_T Constant_Value_k;
  real_T Constant3_Value_cl;
  real_T Constant_Value_m;
  real_T Gain_Gain_a;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_g;
  real_T Gain1_Gain_p;
  real_T Gain_Gain_f;
  real_T Constant_Value_g;
  real_T Constant3_Value_k;
  real_T Constant_Value_o;
  real_T Gain1_Gain_o;
  real_T Gain2_Gain_p;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Gain3_Gain;
  real_T Gain_Gain_az;
  real_T Constant_Value_b;
  real_T Constant3_Value_cq;
  real_T Constant_Value_h;
  real_T Gain3_Gain_o;
  real_T Delay_InitialCondition;
  real_T Constant_Value_by;
  real_T Delay1_InitialCondition;
  real_T Gain4_Gain_f;
  real_T Gain_Gain_n;
  real_T Constant_Value_k5;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Gain_Gain_by;
  real_T Constant2_Value;
  real_T Gain1_Gain_d;
  real_T Delay_InitialCondition_p;
  real_T Constant_Value_d;
  real_T Delay1_InitialCondition_k;
  real_T Saturation_UpperSat_d;
  real_T Saturation_LowerSat_d;
  real_T Constant_Value_dc;
  real_T Constant1_Value;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T kntoms_Gain_e;
  real_T IC_Value_l;
  real_T Gain3_Gain_h;
  real_T Delay_InitialCondition_i;
  real_T Constant_Value_n;
  real_T Delay1_InitialCondition_o;
  real_T kntoms_Gain_a;
  real_T IC_Value_a;
  real_T kntoms_Gain_p;
  real_T IC_Value_k;
  real_T Gain1_Gain_kq;
  real_T Gain3_Gain_a;
  real_T Delay_InitialCondition_b;
  real_T Constant_Value_i;
  real_T Delay1_InitialCondition_l;
  real_T Gain_Gain_nw;
  real_T Constant2_Value_e;
  real_T Gain1_Gain_m;
  real_T Delay_InitialCondition_m;
  real_T Constant_Value_l;
  real_T Delay1_InitialCondition_e;
  real_T Saturation_UpperSat_h;
  real_T Saturation_LowerSat_fk;
  real_T Constant_Value_na;
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
  BlockIO_Autopilot_T Autopilot_B;
  D_Work_Autopilot_T Autopilot_DWork;
  void Autopilot_Chart_Init(rtDW_Chart_Autopilot_T *localDW);
  void Autopilot_Chart(real_T rtu_right, real_T rtu_left, real_T rtu_use_short_path, real_T *rty_out,
                       rtDW_Chart_Autopilot_T *localDW);
};

#endif

