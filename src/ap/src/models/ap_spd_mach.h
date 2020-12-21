#ifndef RTW_HEADER_ap_spd_mach_h_
#define RTW_HEADER_ap_spd_mach_h_
#include <cstring>
#ifndef ap_spd_mach_COMMON_INCLUDES_
# define ap_spd_mach_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ap_spd_mach_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_f;
  real_T Delay1_DSTATE;
  real_T Delay_DSTATE_fp;
  real_T Delay_DSTATE_a;
  uint8_T icLoad;
} D_Work_ap_spd_mach_T;

typedef struct {
  boolean_T enabled;
  real_T V_c_kts;
  real_T V_IAS_kts;
  real_T Theta_deg;
  real_T Ts;
} ExternalInputs_ap_spd_mach_T;

typedef struct {
  real_T Theta_FD;
  real_T Theta_AP;
} ExternalOutputs_ap_spd_mach_T;

struct Parameters_ap_spd_mach_T_ {
  real_T LagFilter1_C1;
  real_T LagFilter_C1;
  real_T DiscreteDerivativeVariableTs_Gain;
  real_T DiscreteDerivativeVariableTs_InitialCondition;
  real_T Gain1_Gain;
  real_T Constant_Value;
  real_T Gain3_Gain;
  real_T Delay_InitialCondition;
  real_T Constant_Value_d;
  real_T Delay1_InitialCondition;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_a;
  real_T Gain2_Gain;
  real_T Constant2_Value;
  real_T Gain1_Gain_h;
  real_T Delay_InitialCondition_g;
  real_T Constant_Value_k;
  real_T Delay1_InitialCondition_p;
};

class ap_spd_machModelClass {
 public:
  ExternalInputs_ap_spd_mach_T ap_spd_mach_U;
  ExternalOutputs_ap_spd_mach_T ap_spd_mach_Y;
  void initialize();
  void step();
  void terminate();
  ap_spd_machModelClass();
  ~ap_spd_machModelClass();
 private:
  static Parameters_ap_spd_mach_T ap_spd_mach_P;
  D_Work_ap_spd_mach_T ap_spd_mach_DWork;
};

#endif

