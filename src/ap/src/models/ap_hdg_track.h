#ifndef RTW_HEADER_ap_hdg_track_h_
#define RTW_HEADER_ap_hdg_track_h_
#include <cfloat>
#include <cmath>
#include <cstring>
#ifndef ap_hdg_track_COMMON_INCLUDES_
# define ap_hdg_track_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ap_hdg_track_types.h"

typedef struct {
  real_T Delay_DSTATE;
  real_T Delay_DSTATE_a;
  uint8_T icLoad;
  uint8_T is_active_c3_ap_hdg_track;
  uint8_T is_c3_ap_hdg_track;
} D_Work_ap_hdg_track_T;

typedef struct {
  boolean_T enabled;
  real_T Psi_c_deg;
  real_T Psi_deg;
  real_T V_TAS_kts;
  real_T Phi_deg;
  real_T Ts;
} ExternalInputs_ap_hdg_track_T;

typedef struct {
  real_T Phi_FD;
  real_T Phi_AP;
} ExternalOutputs_ap_hdg_track_T;

struct Parameters_ap_hdg_track_T_ {
  real_T LagFilter_C1;
  real_T Gain1_Gain;
  real_T ROLLLIM1_tableData[5];
  real_T ROLLLIM1_bp01Data[5];
  real_T Constant3_Value;
  real_T Gain_Gain;
  real_T Constant4_Value;
  real_T Gain_Gain_k;
  real_T Gain2_Gain;
  real_T Constant1_Value;
  real_T Gain1_Gain_d;
  real_T Delay_InitialCondition;
  real_T Constant_Value;
  real_T Delay1_InitialCondition;
};

class ap_hdg_trackModelClass {
 public:
  ExternalInputs_ap_hdg_track_T ap_hdg_track_U;
  ExternalOutputs_ap_hdg_track_T ap_hdg_track_Y;
  void initialize();
  void step();
  void terminate();
  ap_hdg_trackModelClass();
  ~ap_hdg_trackModelClass();
 private:
  static Parameters_ap_hdg_track_T ap_hdg_track_P;
  D_Work_ap_hdg_track_T ap_hdg_track_DWork;
};

#endif

