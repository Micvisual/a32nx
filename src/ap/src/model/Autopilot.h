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
  uint8_T is_active_c1_Autopilot;
  uint8_T is_c1_Autopilot;
} rtDW_Chart_Autopilot_T;

typedef struct {
  real_T u;
  real_T LAW;
} BlockIO_Autopilot_T;

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_m;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_h;
  real_T Delay_DSTATE_g;
  real_T Delay1_DSTATE_b;
  real_T Delay_DSTATE_i;
  real_T Delay_DSTATE_l;
  real_T Delay_DSTATE_a;
  real_T Delay1_DSTATE_a;
  real_T Delay_DSTATE_f;
  real_T Delay_DSTATE_hl;
  real_T Delay1_DSTATE_j;
  real_T Delay_DSTATE_l2;
  real_T Delay_DSTATE_k;
  real_T Delay1_DSTATE_f;
  real_T Delay_DSTATE_p;
  real_T Delay_DSTATE_h4;
  real_T Delay1_DSTATE_je;
  real_T Delay_DSTATE_d;
  real_T loc_trk_time;
  uint8_T icLoad;
  uint8_T icLoad_n;
  uint8_T is_active_c10_Autopilot;
  uint8_T is_c10_Autopilot;
  uint8_T is_active_c3_Autopilot;
  uint8_T is_c3_Autopilot;
  uint8_T is_active_c5_Autopilot;
  uint8_T is_c5_Autopilot;
  uint8_T is_active_c6_Autopilot;
  uint8_T is_c6_Autopilot;
  uint8_T is_LOC;
  boolean_T IC_FirstOutputTime;
  boolean_T IC_FirstOutputTime_o;
  boolean_T IC_FirstOutputTime_g;
  boolean_T IC_FirstOutputTime_f;
  boolean_T IC_FirstOutputTime_l;
  boolean_T IC_FirstOutputTime_c;
  rtDW_Chart_Autopilot_T sf_Chart_i;
  rtDW_Chart_Autopilot_T sf_Chart_jt;
  rtDW_Chart_Autopilot_T sf_Chart_h;
  rtDW_Chart_Autopilot_T sf_Chart_m;
  rtDW_Chart_Autopilot_T sf_Chart_j;
  rtDW_Chart_Autopilot_T sf_Chart_d;
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
  real_T ScheduledGain_BreakpointsForDimension1_i[4];
  real_T ScheduledGain_BreakpointsForDimension1_o[4];
  real_T ScheduledGain_BreakpointsForDimension1_d[4];
  real_T ScheduledGain_BreakpointsForDimension1_e[5];
  real_T ScheduledGain_BreakpointsForDimension1_f[4];
  real_T ScheduledGain_BreakpointsForDimension1_h[5];
  real_T LagFilter_C1;
  real_T LagFilter_C1_l;
  real_T LagFilter1_C1;
  real_T LagFilter_C1_n;
  real_T LagFilter1_C1_p;
  real_T LagFilter_C1_m;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteDerivativeVariableTs_Gain_m;
  real_T DiscreteDerivativeVariableTs_Gain_l;
  real_T DiscreteDerivativeVariableTs_Gain_lf;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T RateLimiterVariableTs_InitialCondition;
  real_T DiscreteDerivativeVariableTs_InitialCondition_e;
  real_T DiscreteDerivativeVariableTs_InitialCondition_h;
  real_T DiscreteDerivativeVariableTs_InitialCondition_b;
  real_T RateLimiterVariableTs_InitialCondition_p;
  real_T ScheduledGain_Table[4];
  real_T ScheduledGain_Table_f[4];
  real_T ScheduledGain_Table_o[4];
  real_T ScheduledGain_Table_i[4];
  real_T ScheduledGain_Table_p[5];
  real_T ScheduledGain_Table_pf[4];
  real_T ScheduledGain_Table_ir[5];
  real_T CompareToConstant_const;
  real_T CompareToConstant_const_j;
  real_T RateLimiterVariableTs_lo;
  real_T RateLimiterVariableTs_lo_o;
  real_T RateLimiterVariableTs_up;
  real_T RateLimiterVariableTs_up_i;
  real_T Constant_Value;
  real_T Constant1_Value;
  real_T Gain_Gain;
  real_T beta_Value;
  real_T beta_Value_m;
  real_T beta_Value_b;
  real_T beta_Value_e;
  real_T Gain1_Gain;
  real_T Gain1_Gain_j;
  real_T Gain1_Gain_k;
  real_T msftmin_Gain;
  real_T Constant_Value_p;
  real_T Constant3_Value;
  real_T ftmintoms_Gain;
  real_T Gain_Gain_j;
  real_T Gain_Gain_g;
  real_T Constant2_Value;
  real_T Gain4_Gain;
  real_T Switch_Threshold;
  real_T Gain1_Gain_e;
  real_T Gain_Gain_c;
  real_T Gain1_Gain_h;
  real_T Gain1_Gain_g;
  real_T msftmin_Gain_c;
  real_T ftmintoms_Gain_e;
  real_T Gain_Gain_e;
  real_T Gain_Gain_ev;
  real_T Switch_Threshold_k;
  real_T Gain1_Gain_gv;
  real_T Gain1_Gain_ec;
  real_T msftmin_Gain_e;
  real_T Constant_Value_b;
  real_T Gain_Gain_el;
  real_T ftmintoms_Gain_i;
  real_T Gain_Gain_h;
  real_T Gain_Gain_l;
  real_T Gain1_Gain_c;
  real_T Gain1_Gain_p;
  real_T msftmin_Gain_g;
  real_T Gain_Gain_f;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T ftmintoms_Gain_m;
  real_T Gain_Gain_jg;
  real_T Gain_Gain_i;
  real_T Constant_Value_j;
  real_T Y_Y0;
  real_T Gain1_Gain_i;
  real_T GainTheta_Gain;
  real_T GainTheta1_Gain;
  real_T Gain_Gain_a;
  real_T Constant1_Value_p;
  real_T Saturation_UpperSat_k;
  real_T Saturation_LowerSat_m;
  real_T Gain1_Gain_k1;
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Constant3_Value_k;
  real_T Gain_Gain_n;
  real_T Constant2_Value_l;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Constant3_Value_l;
  real_T Gain_Gain_m;
  real_T Constant_Value_c;
  real_T Constant3_Value_f;
  real_T Gain_Gain_d;
  real_T Constant_Value_i;
  real_T Constant3_Value_o;
  real_T Constant_Value_h;
  real_T Gain_Gain_nu;
  real_T Saturation_UpperSat_kr;
  real_T Saturation_LowerSat_p;
  real_T Gain2_Gain;
  real_T Gain1_Gain_n;
  real_T Gain_Gain_au;
  real_T Constant_Value_cw;
  real_T Constant3_Value_j;
  real_T Constant_Value_c2;
  real_T Gain1_Gain_jl;
  real_T Gain2_Gain_n;
  real_T Saturation1_UpperSat_i;
  real_T Saturation1_LowerSat_g;
  real_T Gain3_Gain;
  real_T Gain_Gain_ep;
  real_T Constant_Value_e;
  real_T Constant3_Value_f0;
  real_T Constant_Value_n;
  real_T Gain3_Gain_i;
  real_T Delay_InitialCondition;
  real_T Constant_Value_j1;
  real_T Delay1_InitialCondition;
  real_T Gain_Gain_b;
  real_T Constant_Value_p1;
  real_T Gain_Gain_dl;
  real_T Constant2_Value_h;
  real_T Gain1_Gain_kf;
  real_T Delay_InitialCondition_m;
  real_T Constant_Value_e2;
  real_T Delay1_InitialCondition_j;
  real_T Saturation_UpperSat_h;
  real_T Saturation_LowerSat_g;
  real_T Constant_Value_f;
  real_T kntoms_Gain;
  real_T IC_Value;
  real_T kntoms_Gain_e;
  real_T IC_Value_d;
  real_T Gain3_Gain_f;
  real_T Delay_InitialCondition_k;
  real_T Constant_Value_k;
  real_T Delay1_InitialCondition_m;
  real_T Saturation_UpperSat_ho;
  real_T Saturation_LowerSat_c;
  real_T Gain1_Gain_b;
  real_T Constant_Value_em;
  real_T Gain1_Gain_k0;
  real_T Gain1_Gain_ez;
  real_T Gain_Gain_ax;
  real_T Gain_Gain_b3;
  real_T Constant1_Value_e;
  real_T Gain_Gain_h4;
  real_T Constant_Value_ke;
  real_T Gain1_Gain_b0;
  real_T Gain1_Gain_m;
  real_T kntoms_Gain_i;
  real_T msftmin_Gain_m;
  real_T ftmintoms_Gain_n;
  real_T IC_Value_h;
  real_T Gain_Gain_lu;
  real_T Gain_Gain_ji;
  real_T kntoms_Gain_a;
  real_T IC_Value_n;
  real_T Gain1_Gain_jlf;
  real_T Gain3_Gain_o;
  real_T Delay_InitialCondition_e;
  real_T Constant_Value_p0;
  real_T Delay1_InitialCondition_f;
  real_T kntoms_Gain_av;
  real_T IC_Value_m;
  real_T Constant1_Value_d;
  real_T Gain1_Gain_kp;
  real_T Gain3_Gain_fd;
  real_T Delay_InitialCondition_l;
  real_T Constant_Value_cs;
  real_T Delay1_InitialCondition_n;
  real_T Saturation_UpperSat_n;
  real_T Saturation_LowerSat_h;
  real_T Gain1_Gain_pc;
  real_T Constant_Value_jb;
  real_T Gain1_Gain_k4;
  real_T Gain1_Gain_l;
  real_T Gain_Gain_gs;
  real_T Gain_Gain_o;
  real_T Constant1_Value_l;
  real_T Gain_Gain_hk;
  real_T Constant_Value_in;
  real_T Gain2_Gain_g;
  real_T Gain1_Gain_pd;
  real_T Gain1_Gain_a;
  real_T kntoms_Gain_f;
  real_T msftmin_Gain_n;
  real_T ftmintoms_Gain_a;
  real_T IC_Value_g;
  real_T Gain_Gain_gb;
  real_T Gain_Gain_fm;
  real_T Constant1_Value_i;
  real_T Constant2_Value_h1;
  real_T Gain1_Gain_b5;
  real_T Delay_InitialCondition_a;
  real_T Constant_Value_a;
  real_T Delay1_InitialCondition_p;
  real_T Saturation_UpperSat_o;
  real_T Saturation_LowerSat_ha;
  real_T Constant_Value_i0;
  uint8_T ManualSwitch_CurrentSetting;
  uint8_T ManualSwitch_CurrentSetting_b;
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

