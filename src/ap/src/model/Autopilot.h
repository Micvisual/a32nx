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
  uint8_T is_active_c4_Autopilot;
  uint8_T is_c4_Autopilot;
} rtDW_Chart_Autopilot_T;

typedef struct {
  real_T LAW;
} BlockIO_Autopilot_T;

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_b;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_i;
  real_T Delay_DSTATE_iq;
  real_T Delay1_DSTATE_b;
  real_T Delay_DSTATE_m;
  real_T Delay_DSTATE_bk;
  real_T Delay_DSTATE_o;
  real_T Delay1_DSTATE_c;
  real_T Delay_DSTATE_n;
  real_T Delay_DSTATE_j;
  real_T Delay1_DSTATE_l;
  real_T Delay_DSTATE_g;
  real_T Delay_DSTATE_e;
  real_T Delay1_DSTATE_h;
  real_T Delay_DSTATE_jx;
  real_T Delay_DSTATE_d;
  real_T Delay1_DSTATE_m;
  real_T Delay_DSTATE_c;
  real_T loc_trk_time;
  uint8_T icLoad;
  uint8_T icLoad_d;
  uint8_T is_active_c1_Autopilot;
  uint8_T is_c1_Autopilot;
  uint8_T is_active_c2_Autopilot;
  uint8_T is_c2_Autopilot;
  uint8_T is_LOC;
  uint8_T is_active_c3_Autopilot;
  uint8_T is_c3_Autopilot;
  boolean_T IC_FirstOutputTime;
  boolean_T IC_FirstOutputTime_f;
  boolean_T IC_FirstOutputTime_a;
  boolean_T IC_FirstOutputTime_i;
  boolean_T IC_FirstOutputTime_c;
  rtDW_Chart_Autopilot_T sf_Chart_f;
  rtDW_Chart_Autopilot_T sf_Chart_p;
  rtDW_Chart_Autopilot_T sf_Chart_a;
  rtDW_Chart_Autopilot_T sf_Chart_e2;
  rtDW_Chart_Autopilot_T sf_Chart_e;
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
  real_T ScheduledGain_BreakpointsForDimension1_h[4];
  real_T ScheduledGain_BreakpointsForDimension1_i[4];
  real_T ScheduledGain_BreakpointsForDimension1_g[4];
  real_T ScheduledGain_BreakpointsForDimension1_f[4];
  real_T LagFilter_C1;
  real_T LagFilter_C1_a;
  real_T LagFilter1_C1;
  real_T LagFilter_C1_m;
  real_T LagFilter1_C1_e;
  real_T LagFilter_C1_ac;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteDerivativeVariableTs_Gain_p;
  real_T DiscreteDerivativeVariableTs_Gain_o;
  real_T DiscreteDerivativeVariableTs_Gain_j;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T RateLimiterVariableTs_InitialCondition;
  real_T DiscreteDerivativeVariableTs_InitialCondition_j;
  real_T DiscreteDerivativeVariableTs_InitialCondition_p;
  real_T DiscreteDerivativeVariableTs_InitialCondition_n;
  real_T RateLimiterVariableTs_InitialCondition_i;
  real_T ScheduledGain_Table[4];
  real_T ScheduledGain_Table_c[4];
  real_T ScheduledGain_Table_m[4];
  real_T ScheduledGain_Table_i[4];
  real_T ScheduledGain_Table_o[4];
  real_T RateLimiterVariableTs_lo;
  real_T RateLimiterVariableTs_lo_h;
  real_T RateLimiterVariableTs_up;
  real_T RateLimiterVariableTs_up_k;
  real_T Constant_Value;
  real_T Gain1_Gain;
  real_T Gain1_Gain_m;
  real_T Gain2_Gain;
  real_T Gain_Gain;
  real_T Gain4_Gain;
  real_T Gain1_Gain_m3;
  real_T Gain2_Gain_k;
  real_T Gain1_Gain_j;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T Gain_Gain_c;
  real_T Gain_Gain_n;
  real_T Switch_Threshold;
  real_T Gain1_Gain_c;
  real_T Gain2_Gain_k1;
  real_T Gain1_Gain_k;
  real_T msftmin_Gain_j;
  real_T Constant_Value_j;
  real_T Gain_Gain_m;
  real_T ftmintoms_Gain_n;
  real_T Gain_Gain_i;
  real_T Gain_Gain_p;
  real_T Gain1_Gain_l;
  real_T Gain2_Gain_kp;
  real_T Gain1_Gain_kz;
  real_T msftmin_Gain_p;
  real_T Gain_Gain_ml;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T ftmintoms_Gain_k;
  real_T Gain_Gain_f;
  real_T Gain_Gain_p1;
  real_T Constant_Value_o;
  real_T Gain1_Gain_kt;
  real_T Constant3_Value;
  real_T Gain_Gain_b;
  real_T Constant_Value_e;
  real_T Constant3_Value_g;
  real_T Gain_Gain_a;
  real_T Constant_Value_k;
  real_T Constant3_Value_o;
  real_T Constant_Value_a;
  real_T Gain_Gain_aa;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_g;
  real_T Gain2_Gain_a;
  real_T Gain1_Gain_p;
  real_T Gain_Gain_l;
  real_T Constant_Value_g;
  real_T Constant3_Value_h;
  real_T Constant_Value_l;
  real_T Gain1_Gain_o;
  real_T Gain2_Gain_p;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Gain3_Gain;
  real_T Gain_Gain_l2;
  real_T Constant_Value_b;
  real_T Constant3_Value_b;
  real_T Constant_Value_c;
  real_T Gain3_Gain_o;
  real_T Delay_InitialCondition;
  real_T Constant_Value_bb;
  real_T Delay1_InitialCondition;
  real_T Gain4_Gain_f;
  real_T Gain_Gain_i3;
  real_T Constant_Value_k5;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Gain_Gain_by;
  real_T Constant2_Value;
  real_T Gain1_Gain_me;
  real_T Delay_InitialCondition_c;
  real_T Constant_Value_f;
  real_T Delay1_InitialCondition_e;
  real_T Saturation_UpperSat_h;
  real_T Saturation_LowerSat_h;
  real_T Constant_Value_lk;
  real_T Constant1_Value;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T kntoms_Gain_e;
  real_T IC_Value_h;
  real_T Gain3_Gain_a;
  real_T Delay_InitialCondition_k;
  real_T Constant_Value_e3;
  real_T Delay1_InitialCondition_i;
  real_T Saturation_UpperSat_l;
  real_T Saturation_LowerSat_e;
  real_T Gain1_Gain_a;
  real_T Constant_Value_h;
  real_T Gain1_Gain_b;
  real_T Gain1_Gain_i;
  real_T Gain_Gain_e;
  real_T Gain_Gain_o;
  real_T Constant1_Value_e;
  real_T Gain_Gain_fk;
  real_T Constant_Value_ea;
  real_T Gain2_Gain_d;
  real_T Gain1_Gain_d;
  real_T Gain1_Gain_g;
  real_T kntoms_Gain_c;
  real_T msftmin_Gain_b;
  real_T ftmintoms_Gain_d;
  real_T IC_Value_k;
  real_T Gain_Gain_d;
  real_T Gain_Gain_fh;
  real_T kntoms_Gain_p;
  real_T IC_Value_n;
  real_T Gain1_Gain_kq;
  real_T Gain3_Gain_ad;
  real_T Delay_InitialCondition_o;
  real_T Constant_Value_br;
  real_T Delay1_InitialCondition_g;
  real_T Constant1_Value_i;
  real_T Gain1_Gain_ji;
  real_T Gain3_Gain_j;
  real_T Delay_InitialCondition_ke;
  real_T Constant_Value_ax;
  real_T Delay1_InitialCondition_n;
  real_T Saturation_UpperSat_f;
  real_T Saturation_LowerSat_b;
  real_T Gain1_Gain_f;
  real_T Constant_Value_he;
  real_T Gain1_Gain_ig;
  real_T Gain1_Gain_do;
  real_T Gain_Gain_h;
  real_T Gain_Gain_iq;
  real_T Constant1_Value_o;
  real_T Gain_Gain_bs;
  real_T Constant_Value_i;
  real_T Gain2_Gain_f;
  real_T Gain1_Gain_ad;
  real_T Gain1_Gain_a3;
  real_T kntoms_Gain_d;
  real_T msftmin_Gain_d;
  real_T ftmintoms_Gain_f;
  real_T IC_Value_d;
  real_T Gain_Gain_d5;
  real_T Gain_Gain_a5;
  real_T Gain_Gain_nw;
  real_T Constant2_Value_e;
  real_T Gain1_Gain_dw;
  real_T Delay_InitialCondition_f;
  real_T Constant_Value_hy;
  real_T Delay1_InitialCondition_k;
  real_T Saturation_UpperSat_n;
  real_T Saturation_LowerSat_a;
  real_T Constant_Value_o3;
  real_T Constant_Value_f3;
  uint8_T ManualSwitch_CurrentSetting;
  uint8_T ManualSwitch_CurrentSetting_h;
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

