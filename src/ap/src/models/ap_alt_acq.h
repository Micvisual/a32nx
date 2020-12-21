#ifndef RTW_HEADER_ap_alt_acq_h_
#define RTW_HEADER_ap_alt_acq_h_
#include <cmath>
#include <cstring>
#ifndef ap_alt_acq_COMMON_INCLUDES_
# define ap_alt_acq_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ap_alt_acq_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_j;
  uint8_T icLoad;
  boolean_T IC_FirstOutputTime;
} D_Work_ap_alt_acq_T;

typedef struct {
  boolean_T enabled;
  real_T H_c_ft;
  real_T H_ft;
  real_T Theta_deg;
  real_T Phi_deg;
  real_T alpha_deg;
  real_T V_TAS_kts;
  real_T Ts;
} ExternalInputs_ap_alt_acq_T;

typedef struct {
  real_T Theta_FD;
  real_T Theta_AP;
} ExternalOutputs_ap_alt_acq_T;

struct Parameters_ap_alt_acq_T_ {
  real_T LagFilter_C1;
  real_T Gain1_Gain;
  real_T Constant1_Value;
  real_T Gain2_Gain;
  real_T Gain_Gain;
  real_T Gain1_Gain_m;
  real_T Gain1_Gain_i;
  real_T kntoms_Gain;
  real_T msftmin_Gain;
  real_T ftmintoms_Gain;
  real_T IC_Value;
  real_T Gain_Gain_l;
  real_T Constant2_Value;
  real_T Gain1_Gain_o;
  real_T Delay_InitialCondition;
  real_T Constant_Value;
  real_T Delay1_InitialCondition;
};

class ap_alt_acqModelClass {
 public:
  ExternalInputs_ap_alt_acq_T ap_alt_acq_U;
  ExternalOutputs_ap_alt_acq_T ap_alt_acq_Y;
  void initialize();
  void step();
  void terminate();
  ap_alt_acqModelClass();
  ~ap_alt_acqModelClass();
 private:
  static Parameters_ap_alt_acq_T ap_alt_acq_P;
  D_Work_ap_alt_acq_T ap_alt_acq_DWork;
};

#endif

