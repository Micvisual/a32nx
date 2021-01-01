#ifndef RTW_HEADER_Autopilot_types_h_
#define RTW_HEADER_Autopilot_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_base_raw_input_
#define DEFINED_TYPEDEF_FOR_base_raw_input_

typedef struct {
  real_T trigger_ap_master;
  real_T trigger_ap_off;
  real_T trigger_hdg_mode;
  real_T trigger_alt_mode;
  real_T trigger_vs_mode;
  real_T trigger_loc;
  real_T trigger_appr;
} base_raw_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_time_
#define DEFINED_TYPEDEF_FOR_base_raw_time_

typedef struct {
  real_T dt;
  real_T simulation_time;
} base_raw_time;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_data_
#define DEFINED_TYPEDEF_FOR_base_raw_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T q_rad_s;
  real_T r_rad_s;
  real_T p_rad_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T V_c_srs_kn;
  real_T gear_strut_compression_1;
  real_T gear_strut_compression_2;
} base_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_input_
#define DEFINED_TYPEDEF_FOR_ap_input_

typedef struct {
  base_raw_input input;
  base_raw_time time;
  base_raw_data data;
} ap_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_data_
#define DEFINED_TYPEDEF_FOR_base_data_

typedef struct {
  real_T Theta_deg;
  real_T Phi_deg;
  real_T qk_deg_s;
  real_T rk_deg_s;
  real_T pk_deg_s;
  real_T V_ias_kn;
  real_T V_tas_kn;
  real_T V_mach;
  real_T alpha_deg;
  real_T H_ft;
  real_T H_ind_ft;
  real_T H_radio_ft;
  real_T H_dot_ft_min;
  real_T Psi_magnetic_deg;
  real_T Psi_magnetic_track_deg;
  real_T Psi_true_deg;
  real_T bx_m_s2;
  real_T by_m_s2;
  real_T bz_m_s2;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T V_c_srs_kn;
  real_T on_ground;
} base_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_output_command_
#define DEFINED_TYPEDEF_FOR_base_raw_output_command_

typedef struct {
  real_T Theta_c_deg;
  real_T Phi_c_deg;
  real_T Beta_c_deg;
} base_raw_output_command;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_output_
#define DEFINED_TYPEDEF_FOR_base_raw_output_

typedef struct {
  real_T ap_on;
  base_raw_output_command flight_director;
  base_raw_output_command autopilot;
} base_raw_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_output_
#define DEFINED_TYPEDEF_FOR_ap_output_

typedef struct {
  base_raw_time time;
  base_raw_input input;
  base_data data;
  base_raw_output output;
} ap_output;

#endif

#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH_Autopilot_T
#define typedef_skA4KFEZ4HPkJJBOYCrevdH_Autopilot_T

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH_Autopilot_T;

#endif

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E_Autopilot_T
#define typedef_sJCxfmxS8gBOONUZjbjUd9E_Autopilot_T

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E_Autopilot_T;

#endif

typedef struct Parameters_Autopilot_T_ Parameters_Autopilot_T;

#endif

