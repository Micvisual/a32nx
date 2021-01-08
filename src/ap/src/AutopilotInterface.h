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

#include "Autopilot.h"
#include "SimConnectInterfaceAutopilot.h"

class AutopilotInterface {
 public:
  bool connect();

  void disconnect();

  bool update(double sampleTime);

 private:
  double previousSimulationTime = 0;

  SimConnectInterface simConnectInterface;
  AutopilotModelClass model;

  ID idAutopilotUseLvar;
  ID idAutopilotOn;
  ID idAutopilotPitch;
  ID idAutopilotBank;
  ID idAutopilotYaw;

  ID idFlightDirectorBank;
  ID idFlightDirectorPitch;
  ID idFlightDirectorYaw;

  ID idFlightGuidanceCrossTrackError;
  ID idFlightGuidanceTrackAngleError;

  ID idFlightPhase;
  // ID idFmgcFlightPlanAvailable;
  ID idFmgcV2;
  ID idFmgcThrustReductionAltitude;
  ID idFmgcThrustReductionAltitudeGoAround;

  ID idFmaLateralMode;
  ID idFmaLateralArmed;
  ID idFmaVerticalMode;
  ID idFmaVerticalArmed;

  bool getModelInputDataFromSim(double sampleTime);

  bool writeModelOuputDataToSim();
};
