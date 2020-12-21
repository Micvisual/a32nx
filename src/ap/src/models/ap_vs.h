#ifndef RTW_HEADER_ap_vs_h_
#define RTW_HEADER_ap_vs_h_
#include <cmath>
#include <cstring>
#ifndef ap_vs_COMMON_INCLUDES_
# define ap_vs_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ap_vs_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_j;
  uint8_T icLoad;
  boolean_T IC_FirstOutputTime;
} D_Work_ap_vs_T;

typedef struct {
  boolean_T enabled;
  real_T H_c_ft;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T alpha_deg;
  real_T V_TAS_kts;
  real_T Ts;
} ExternalInputs_ap_vs_T;

typedef struct {
  real_T Theta_FD;
  real_T Theta_AP;
} ExternalOutputs_ap_vs_T;

struct Parameters_ap_vs_T_ {
  real_T LagFilter_C1;
  real_T Gain1_Gain;
  real_T Constant1_Value;
  real_T Gain2_Gain;
  real_T Gain1_Gain_m;
  real_T Gain1_Gain_i;
  real_T kntoms_Gain;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T IC_Value;
  real_T Gain_Gain;
  real_T Constant2_Value;
  real_T Gain1_Gain_o;
  real_T Delay_InitialCondition;
  real_T Constant_Value;
  real_T Delay1_InitialCondition;
};

class ap_vsModelClass {
 public:
  ExternalInputs_ap_vs_T ap_vs_U;
  ExternalOutputs_ap_vs_T ap_vs_Y;
  void initialize();
  void step();
  void terminate();
  ap_vsModelClass();
  ~ap_vsModelClass();
 private:
  static Parameters_ap_vs_T ap_vs_P;
  D_Work_ap_vs_T ap_vs_DWork;
};

#endif

