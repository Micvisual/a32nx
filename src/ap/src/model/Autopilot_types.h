#ifndef RTW_HEADER_Autopilot_types_h_
#define RTW_HEADER_Autopilot_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_base_raw_mode_
#define DEFINED_TYPEDEF_FOR_base_raw_mode_

typedef struct {
  real_T lateral_mode;
  real_T vertical_mode;
} base_raw_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_raw_input_
#define DEFINED_TYPEDEF_FOR_base_raw_input_

typedef struct {
  boolean_T AP_1_push;
  boolean_T AP_2_push;
  boolean_T AP_DISCONNECT_push;
  boolean_T HDG_push;
  boolean_T HDG_pull;
  boolean_T ALT_push;
  boolean_T ALT_pull;
  boolean_T VS_push;
  boolean_T VS_pull;
  boolean_T LOC_push;
  boolean_T APPR_push;
  real_T Psi_fcu_deg;
  real_T H_fcu_ft;
  real_T H_dot_fcu_fpm;
  real_T FPA_fcu_deg;
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
  real_T V_gnd_kn;
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
  boolean_T ap_fd_active;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T V2_kn;
  boolean_T is_flight_plan_available;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T gear_strut_compression_1;
  real_T gear_strut_compression_2;
  real_T zeta_pos;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
} base_raw_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ap_input_
#define DEFINED_TYPEDEF_FOR_ap_input_

typedef struct {
  base_raw_mode mode;
  base_raw_input input;
  base_raw_time time;
  base_raw_data data;
} ap_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lateral_mode_
#define DEFINED_TYPEDEF_FOR_lateral_mode_

typedef enum {
  lateral_mode_NONE = 0,
  lateral_mode_HDG = 10,
  lateral_mode_TRACK = 11,
  lateral_mode_NAV = 20,
  lateral_mode_LOC_CPT = 30,
  lateral_mode_LOC_TRACK = 31,
  lateral_mode_LAND = 32,
  lateral_mode_FLARE = 33,
  lateral_mode_ROLL_OUT = 34,
  lateral_mode_RWY = 40,
  lateral_mode_RWY_TRACK = 41,
  lateral_mode_GA_TRACK = 50
} lateral_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lateral_law_
#define DEFINED_TYPEDEF_FOR_lateral_law_

typedef enum {
  lateral_law_NONE = 0,
  lateral_law_HDG,
  lateral_law_TRACK,
  lateral_law_HPATH,
  lateral_law_LOC_CPT,
  lateral_law_LOC_TRACK,
  lateral_law_ROLL_OUT
} lateral_law;

#endif

#ifndef DEFINED_TYPEDEF_FOR_vertical_mode_
#define DEFINED_TYPEDEF_FOR_vertical_mode_

typedef enum {
  vertical_mode_NONE = 0,
  vertical_mode_ALT = 10,
  vertical_mode_ALT_CPT = 11,
  vertical_mode_OP_CLB = 12,
  vertical_mode_OP_DES = 13,
  vertical_mode_VS = 14,
  vertical_mode_ALT_CST = 20,
  vertical_mode_ALT_CST_CPT = 21,
  vertical_mode_CLB = 22,
  vertical_mode_DES = 23,
  vertical_mode_GS_CPT = 30,
  vertical_mode_GS_TRACK = 31,
  vertical_mode_LAND = 32,
  vertical_mode_FLARE = 33,
  vertical_mode_ROLL_OUT = 34,
  vertical_mode_SRS = 40
} vertical_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_athr_mode_
#define DEFINED_TYPEDEF_FOR_athr_mode_

typedef enum {
  athr_mode_NONE = 0,
  athr_mode_SPEED,
  athr_mode_THRUST_IDLE,
  athr_mode_THRUST_CLB
} athr_mode;

#endif

#ifndef DEFINED_TYPEDEF_FOR_vertical_law_
#define DEFINED_TYPEDEF_FOR_vertical_law_

typedef enum {
  vertical_law_NONE = 0,
  vertical_law_ALT_HOLD,
  vertical_law_ALT_ACQ,
  vertical_law_SPD_MACH,
  vertical_law_VS,
  vertical_law_FPA,
  vertical_law_GS,
  vertical_law_FLARE,
  vertical_law_SRS
} vertical_law;

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
  real_T V_gnd_kn;
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
  boolean_T ap_fd_active;
  real_T ap_V_c_kn;
  real_T ap_H_c_ft;
  real_T ap_Psi_c_deg;
  real_T ap_H_dot_c_ft_min;
  real_T ap_FPA_c_deg;
  boolean_T nav_valid;
  real_T nav_loc_deg;
  real_T nav_radial_error_deg;
  real_T nav_dme_nmi;
  real_T nav_gs_error_deg;
  real_T flight_guidance_xtk_nmi;
  real_T flight_guidance_tae_deg;
  real_T V2_kn;
  boolean_T is_flight_plan_available;
  real_T thrust_reduction_altitude;
  real_T thrust_reduction_altitude_go_around;
  real_T on_ground;
  real_T zeta_deg;
  real_T throttle_lever_1_pos;
  real_T throttle_lever_2_pos;
} base_data;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_lateral_input_
#define DEFINED_TYPEDEF_FOR_base_lateral_input_

typedef struct {
  boolean_T HDG_push;
  boolean_T HDG_pull;
  boolean_T LOC_push;
  boolean_T APPR_push;
  real_T Psi_fcu_deg;
} base_lateral_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_lateral_armed_
#define DEFINED_TYPEDEF_FOR_base_lateral_armed_

typedef struct {
  boolean_T NAV;
  boolean_T LOC;
} base_lateral_armed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_lateral_condition_
#define DEFINED_TYPEDEF_FOR_base_lateral_condition_

typedef struct {
  boolean_T NAV;
  boolean_T LOC_CPT;
  boolean_T LOC_TRACK;
  boolean_T LAND;
  boolean_T FLARE;
  boolean_T ROLL_OUT;
  boolean_T GA_TRACK;
  boolean_T RWY;
  boolean_T RWY_TRACK;
} base_lateral_condition;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_lateral_output_
#define DEFINED_TYPEDEF_FOR_base_lateral_output_

typedef struct {
  lateral_mode mode;
  lateral_law law;
  real_T Psi_c_deg;
} base_lateral_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_lateral_
#define DEFINED_TYPEDEF_FOR_base_lateral_

typedef struct {
  base_lateral_input input;
  base_lateral_armed armed;
  base_lateral_condition condition;
  base_lateral_output output;
} base_lateral;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_vertical_input_
#define DEFINED_TYPEDEF_FOR_base_vertical_input_

typedef struct {
  boolean_T ALT_push;
  boolean_T ALT_pull;
  boolean_T VS_push;
  boolean_T VS_pull;
  boolean_T LOC_push;
  boolean_T APPR_push;
  real_T H_fcu_ft;
  real_T H_dot_fcu_fpm;
  real_T FPA_fcu_deg;
} base_vertical_input;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_vertical_armed_
#define DEFINED_TYPEDEF_FOR_base_vertical_armed_

typedef struct {
  boolean_T ALT;
  boolean_T ALT_CST;
  boolean_T CLB;
  boolean_T DES;
  boolean_T GS;
} base_vertical_armed;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_vertical_condition_
#define DEFINED_TYPEDEF_FOR_base_vertical_condition_

typedef struct {
  boolean_T ALT;
  boolean_T ALT_CPT;
  boolean_T ALT_CST;
  boolean_T ALT_CST_CPT;
  boolean_T CLB;
  boolean_T DES;
  boolean_T GS_CPT;
  boolean_T GS_TRACK;
  boolean_T LAND;
  boolean_T FLARE;
  boolean_T ROLL_OUT;
  boolean_T SRS;
  boolean_T THR_RED;
} base_vertical_condition;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_vertical_output_
#define DEFINED_TYPEDEF_FOR_base_vertical_output_

typedef struct {
  vertical_mode mode;
  athr_mode mode_autothrust;
  vertical_law law;
  real_T H_c_ft;
  real_T H_dot_c_fpm;
  real_T FPA_c_deg;
} base_vertical_output;

#endif

#ifndef DEFINED_TYPEDEF_FOR_base_vertical_
#define DEFINED_TYPEDEF_FOR_base_vertical_

typedef struct {
  base_vertical_input input;
  base_vertical_armed armed;
  base_vertical_condition condition;
  base_vertical_output output;
} base_vertical;

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
  base_raw_mode mode;
  base_raw_input input;
  base_data data;
  base_lateral lateral;
  base_vertical vertical;
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

