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

#include "AutopilotLaws.h"
#include "AutopilotStateMachine.h"
#include "FlightDataRecorder.h"
#include "FlyByWire.h"
#include "InterpolatingLookupTable.h"
#include "SimConnectInterface.h"

class FlyByWireInterface {
 public:
  bool connect();

  void disconnect();

  bool update(double sampleTime);

 private:
  const std::string MODEL_CONFIGURATION_FILEPATH = "\\work\\ModelConfiguration.ini";
  const std::string THROTTLE_CONFIGURATION_FILEPATH = "\\work\\ThrottleConfiguration.ini";

  bool isThrottleLoggingEnabled = false;
  bool isThrottleHandlingEnabled = false;
  bool useReverseOnAxis = false;
  bool useReverseIdle = false;
  double idleThrottleInput = 0;
  double throttleDetentDeadZone = 2.0;

  double lastThrottleInput_1 = -1;
  double lastThrottleInput_2 = -1;

  double previousSimulationTime = 0;

  bool autopilotStateMachineEnabled = false;
  bool autopilotLawsEnabled = false;
  bool flyByWireEnabled = false;

  bool pauseDetected = false;

  FlightDataRecorder flightDataRecorder;

  SimConnectInterface simConnectInterface;
  FlyByWireModelClass flyByWire;
  AutopilotStateMachineModelClass autopilotStateMachine;
  AutopilotLawsModelClass autopilotLaws;
  InterpolatingLookupTable throttleLookupTable;

  ID idSideStickPositionX;
  ID idSideStickPositionY;
  ID idSideStickLeftPositionX;
  ID idSideStickLeftPositionY;
  ID idSideStickRightPositionX;
  ID idSideStickRightPositionY;

  ID idRudderPositionOverrideOn;
  ID idRudderPosition;

  ID idThrottlePositionOverrideOn;
  ID idThrottlePosition_1;
  ID idThrottlePosition_2;

  ID idFmaLateralMode;
  ID idFmaLateralArmed;
  ID idFmaVerticalMode;
  ID idFmaVerticalArmed;

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

  ap_raw_laws_input autopilotStateMachineOutput;
  ap_raw_output autopilotLawsOutput;

  bool readDataAndLocalVariables(double sampleTime);

  bool updateAutopilotStateMachine(double sampleTime);

  bool updateAutopilotLaws(double sampleTime);

  bool updateFlyByWire(double sampleTime);

  void initializeThrottles();

  bool processThrottles();

  double calculateDeadzones(double deadzone, double input);
  double calculateDeadzone(double deadzone, double target, double input);
};
