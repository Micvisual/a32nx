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

#include <iomanip>
#include <iostream>

#include "AutopilotInterface.h"
#include "SimConnectDataAutopilot.h"

using namespace std;

bool AutopilotInterface::connect() {
  // register L variable for custom fly-by-wire interface
  idAutopilotUseLvar = register_named_variable("A32NX_AUTOPILOT_USE_LVAR");
  idAutopilotOn = register_named_variable("A32NX_AUTOPILOT_ON");
  idAutopilotPitch = register_named_variable("A32NX_AUTOPILOT_PITCH");
  idAutopilotBank = register_named_variable("A32NX_AUTOPILOT_BANK");
  idAutopilotYaw = register_named_variable("A32NX_AUTOPILOT_YAW");

  // register L variable for flight director
  idFlightDirectorBank = register_named_variable("A32NX_FLIGHT_DIRECTOR_BANK");
  idFlightDirectorPitch = register_named_variable("A32NX_FLIGHT_DIRECTOR_PITCH");
  idFlightDirectorYaw = register_named_variable("A32NX_FLIGHT_DIRECTOR_YAW");

  // register L variables for flight guidance
  idFlightGuidanceCrossTrackError = register_named_variable("A32NX_FG_CROSS_TRACK_ERROR");
  idFlightGuidanceTrackAngleError = register_named_variable("A32NX_FG_TRACK_ANGLE_ERROR");

  // initialize model
  model.initialize();

  // set lvar
  set_named_variable_value(idAutopilotUseLvar, 1);

  // connect to sim connect
  return simConnectInterface.connect();
}

void AutopilotInterface::disconnect() {
  // set lvar
  set_named_variable_value(idAutopilotUseLvar, 0);

  // disconnect from sim connect
  simConnectInterface.disconnect();

  // terminate model
  model.terminate();
}

bool AutopilotInterface::update(double sampleTime) {
  bool result = true;

  // get data & inputs
  result &= getModelInputDataFromSim(sampleTime);

  // step model
  model.step();

  // write output
  result &= writeModelOuputDataToSim();

  // return result
  return result;
}

bool AutopilotInterface::getModelInputDataFromSim(double sampleTime) {
  // reset input
  simConnectInterface.resetSimInput();

  // request data
  if (!simConnectInterface.requestData()) {
    cout << "WASM: Request data failed!" << endl;
    return false;
  }

  // read data
  if (!simConnectInterface.readData()) {
    cout << "WASM: Read data failed!" << endl;
    return false;
  }

  // get data from interface
  SimData simData = simConnectInterface.getSimData();
  SimInput simInput = simConnectInterface.getSimInput();

  // detect pause
  bool isInPause = false;
  if ((simData.simulation_time == previousSimulationTime) || (simData.simulation_time < 0.2)) {
    isInPause = true;
  }
  previousSimulationTime = simData.simulation_time;

  // fill time into model
  model.Autopilot_U.in.time.dt = sampleTime;
  model.Autopilot_U.in.time.simulation_time = simData.simulation_time;

  // fill data into model
  model.Autopilot_U.in.data.Theta_deg = simData.Theta_deg;
  model.Autopilot_U.in.data.Phi_deg = simData.Phi_deg;
  model.Autopilot_U.in.data.q_rad_s = simData.bodyRotationVelocity.x;
  model.Autopilot_U.in.data.r_rad_s = simData.bodyRotationVelocity.y;
  model.Autopilot_U.in.data.p_rad_s = simData.bodyRotationVelocity.z;
  model.Autopilot_U.in.data.V_ias_kn = simData.V_ias_kn;
  model.Autopilot_U.in.data.V_tas_kn = simData.V_tas_kn;
  model.Autopilot_U.in.data.V_mach = simData.V_mach;
  model.Autopilot_U.in.data.alpha_deg = simData.alpha_deg;
  model.Autopilot_U.in.data.H_ft = simData.H_ft;
  model.Autopilot_U.in.data.H_ind_ft = simData.H_ind_ft;
  model.Autopilot_U.in.data.H_radio_ft = simData.H_radio_ft;
  model.Autopilot_U.in.data.H_dot_ft_min = simData.H_dot_ft_min;
  model.Autopilot_U.in.data.Psi_magnetic_deg = simData.Psi_magnetic_deg;
  model.Autopilot_U.in.data.Psi_magnetic_track_deg = simData.Psi_magnetic_track_deg;
  model.Autopilot_U.in.data.Psi_true_deg = simData.Psi_true_deg;
  model.Autopilot_U.in.data.bx_m_s2 = simData.bx_m_s2;
  model.Autopilot_U.in.data.by_m_s2 = simData.by_m_s2;
  model.Autopilot_U.in.data.bz_m_s2 = simData.bz_m_s2;
  model.Autopilot_U.in.data.ap_V_c_kn = simData.ap_V_c_kn;
  model.Autopilot_U.in.data.ap_H_c_ft = simData.ap_H_c_ft;
  model.Autopilot_U.in.data.ap_Psi_c_deg = simData.ap_Psi_c_deg;
  model.Autopilot_U.in.data.ap_H_dot_c_ft_min = simData.ap_H_dot_c_ft_min;
  model.Autopilot_U.in.data.nav_loc_deg = simData.nav_loc_deg;
  model.Autopilot_U.in.data.nav_radial_error_deg = simData.nav_radial_error_deg;
  model.Autopilot_U.in.data.nav_dme_nmi = simData.nav_dme_nmi;
  model.Autopilot_U.in.data.nav_gs_error_deg = simData.nav_gs_error_deg;
  model.Autopilot_U.in.data.flight_guidance_xtk_nmi = get_named_variable_value(idFlightGuidanceCrossTrackError);
  model.Autopilot_U.in.data.flight_guidance_tae_deg = get_named_variable_value(idFlightGuidanceTrackAngleError);
  model.Autopilot_U.in.data.gear_strut_compression_1 = simData.gear_strut_compression_1;
  model.Autopilot_U.in.data.gear_strut_compression_2 = simData.gear_strut_compression_2;
  // TODO: add SLEW detection!

  // fill inputs into model
  model.Autopilot_U.in.input.trigger_ap_master = simInput.trigger_ap_master;
  model.Autopilot_U.in.input.trigger_ap_off = simInput.trigger_ap_off;
  model.Autopilot_U.in.input.trigger_hdg_mode = simInput.trigger_hdg_mode;
  model.Autopilot_U.in.input.trigger_alt_mode = simInput.trigger_alt_mode;
  model.Autopilot_U.in.input.trigger_vs_mode = simInput.trigger_vs_mode;
  model.Autopilot_U.in.input.trigger_loc = simInput.trigger_loc;
  model.Autopilot_U.in.input.trigger_appr = simInput.trigger_appr;

  // success
  return true;
}

bool AutopilotInterface::writeModelOuputDataToSim() {
  // set flight director
  set_named_variable_value(idFlightDirectorPitch, -1.0 * model.Autopilot_Y.out.output.flight_director.Theta_c_deg);
  set_named_variable_value(idFlightDirectorBank, -1.0 * model.Autopilot_Y.out.output.flight_director.Phi_c_deg);
  set_named_variable_value(idFlightDirectorYaw, model.Autopilot_Y.out.output.flight_director.Beta_c_deg);

  // set autopilot
  set_named_variable_value(idAutopilotOn, model.Autopilot_Y.out.output.ap_on);
  set_named_variable_value(idAutopilotPitch, model.Autopilot_Y.out.output.autopilot.Theta_c_deg);
  set_named_variable_value(idAutopilotBank, model.Autopilot_Y.out.output.autopilot.Phi_c_deg);
  set_named_variable_value(idAutopilotYaw, model.Autopilot_Y.out.output.autopilot.Beta_c_deg);

  // success
  return true;
}
