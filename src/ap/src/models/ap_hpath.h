#ifndef RTW_HEADER_ap_hpath_h_
#define RTW_HEADER_ap_hpath_h_
#include <cfloat>
#include <cmath>
#include <cstring>
#ifndef ap_hpath_COMMON_INCLUDES_
# define ap_hpath_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ap_hpath_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_f;
  uint8_T icLoad;
  uint8_T is_active_c3_ap_hpath;
  uint8_T is_c3_ap_hpath;
} D_Work_ap_hpath_T;

typedef struct {
  boolean_T enabled;
  real_T XTK_nm;
  real_T TAE_deg;
  real_T Psi_track_deg;
  real_T V_TAS_kts;
  real_T Phi_deg;
  real_T Ts;
} ExternalInputs_ap_hpath_T;

typedef struct {
  real_T Phi_FD;
  real_T Phi_AP;
} ExternalOutputs_ap_hpath_T;

struct Parameters_ap_hpath_T_ {
  real_T LagFilter_C1;
  real_T Gain1_Gain;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Constant3_Value;
  real_T Constant_Value;
  real_T Gain_Gain;
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T Gain1_Gain_a;
  real_T Gain_Gain_p;
  real_T Constant4_Value;
  real_T Gain_Gain_b;
  real_T Gain2_Gain;
  real_T Constant1_Value;
  real_T Gain1_Gain_n;
  real_T Delay_InitialCondition;
  real_T Constant_Value_h;
  real_T Delay1_InitialCondition;
};

class ap_hpathModelClass {
 public:
  ExternalInputs_ap_hpath_T ap_hpath_U;
  ExternalOutputs_ap_hpath_T ap_hpath_Y;
  void initialize();
  void step();
  void terminate();
  ap_hpathModelClass();
  ~ap_hpathModelClass();
 private:
  static Parameters_ap_hpath_T ap_hpath_P;
  D_Work_ap_hpath_T ap_hpath_DWork;
};

#endif

