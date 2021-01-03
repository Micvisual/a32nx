/*
 * A32NX
 * Copyright (C) 2020 FlyByWire Simulations and its contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <MSFS/Legacy/gauges.h>
#include <SimConnect.h>

struct SimData {
  double Theta_deg;
  double Phi_deg;
  SIMCONNECT_DATA_XYZ bodyRotationVelocity;
  double V_ias_kn;
  double V_tas_kn;
  double V_mach;
  double V_gnd_kn;
  double alpha_deg;
  double H_ft;
  double H_ind_ft;
  double H_radio_ft;
  double H_dot_ft_min;
  double Psi_magnetic_deg;
  double Psi_magnetic_track_deg;
  double Psi_true_deg;
  double bx_m_s2;
  double by_m_s2;
  double bz_m_s2;
  double ap_V_c_kn;
  double ap_H_c_ft;
  double ap_Psi_c_deg;
  double ap_H_dot_c_ft_min;
  double nav_loc_deg;
  double nav_radial_error_deg;
  double nav_dme_nmi;
  double nav_gs_error_deg;
  double gear_strut_compression_1;
  double gear_strut_compression_2;
  double zeta_pos;
  double simulation_time;
  bool slew_on;
};

struct SimInput {
  double trigger_ap_master;
  double trigger_ap_off;
  double trigger_hdg_mode;
  double trigger_alt_mode;
  double trigger_vs_mode;
  double trigger_loc;
  double trigger_appr;
};
