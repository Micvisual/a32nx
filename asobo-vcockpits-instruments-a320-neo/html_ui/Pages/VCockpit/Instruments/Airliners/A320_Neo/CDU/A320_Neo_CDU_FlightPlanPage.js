class CDUFlightPlanPage {
    static ShowPage(mcdu, offset = 0) {
        mcdu.clearDisplay();
        CDUFlightPlanPage._timer = 0;
        mcdu.pageUpdate = () => {
            CDUFlightPlanPage._timer++;
            if (CDUFlightPlanPage._timer >= 100) {
                CDUFlightPlanPage.ShowPage(mcdu, offset);
            }
        };
        const isFlying = mcdu.getIsFlying();
        let originIdentCell = "----";
        if (mcdu.flightPlanManager.getOrigin()) {
            originIdentCell = mcdu.flightPlanManager.getOrigin().ident;
            const runway = mcdu.flightPlanManager.getDepartureRunway();
            if (runway) {
                originIdentCell += Avionics.Utils.formatRunway(runway.designation);
            }
        }
        let originTimeCell = "----";
        if (mcdu.flightPlanManager.getOrigin()) {
            if (isFlying) {
                originTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getOrigin().estimatedTimeOfArrivalFP);
            } else {
                originTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getOrigin().cumulativeEstimatedTimeEnRouteFP);
            }
        }
        let originLineColor = "green";
        if (mcdu.flightPlanManager.getCurrentFlightPlanIndex() === 1) {
            originLineColor = "yellow";
        }
        let destCell = "----";
        if (mcdu.flightPlanManager.getDestination()) {
            destCell = mcdu.flightPlanManager.getDestination().ident;
            const approachRunway = mcdu.flightPlanManager.getApproachRunway();
            if (approachRunway) {
                destCell += Avionics.Utils.formatRunway(approachRunway.designation);
            }
        }
        let rows = [[""], [""], [""], [""], [""], [""], [""], [""], [""], [""], [""], [""],];
        let rowsCount = 6;
        if (mcdu.flightPlanManager.getCurrentFlightPlanIndex() === 1) {
            rowsCount = 5;
            rows[10] = ["TMPY[color]red", "TMPY[color]red"];
            rows[11] = ["*ERASE[color]red", "INSERT*[color]red"];
            mcdu.onLeftInput[5] = async () => {
                mcdu.eraseTemporaryFlightPlan(() => {
                    CDUFlightPlanPage.ShowPage(mcdu, 0);
                });
            };
            mcdu.onRightInput[5] = async () => {
                mcdu.insertTemporaryFlightPlan(() => {
                    CDUFlightPlanPage.ShowPage(mcdu, 0);
                });
            };
        }
        const waypointsWithDiscontinuities = [];
        const routeFirstWaypointIndex = 1 + mcdu.flightPlanManager.getDepartureWaypointsCount();
        const routeLastWaypointIndex = mcdu.flightPlanManager.getWaypointsCount() - 2 - mcdu.flightPlanManager.getArrivalWaypointsCount();
        let first = 0;
        let firstApproach = 0;
        if (mcdu.flightPlanManager.isActiveApproach()) {
            first = mcdu.flightPlanManager.getWaypointsCount() - 1;
        } else {
            first = Math.max(0, mcdu.flightPlanManager.getActiveWaypointIndex(false, true) - 1);
        }
        if (mcdu.currentFlightPhase <= FlightPhase.FLIGHT_PHASE_TAKEOFF) {
            first = 0;
        }
        if (mcdu.flightPlanManager.getIsDirectTo()) {
            const directToTarget = mcdu.flightPlanManager.getDirectToTarget();
            if (directToTarget) {
                first = mcdu.flightPlanManager.getWaypoints().findIndex(wp => {
                    return wp.icao === directToTarget.icao;
                });
                if (first === -1) {
                    first = 0;
                    firstApproach = mcdu.flightPlanManager.getApproachWaypoints().findIndex(wp => {
                        return wp.icao === directToTarget.icao;
                    });
                    if (firstApproach != -1) {
                        first = Infinity;
                    }
                }
            }
        }
        for (let i = first; i < mcdu.flightPlanManager.getWaypointsCount(); i++) {
            const prev = waypointsWithDiscontinuities[waypointsWithDiscontinuities.length - 1];
            const wp = mcdu.flightPlanManager.getWaypoint(i);
            if (wp && wp.ident != "USR") {
                if (!prev || (prev.wp && prev.wp.ident != wp.ident)) {
                    waypointsWithDiscontinuities.push({ wp: mcdu.flightPlanManager.getWaypoint(i), fpIndex: i });
                }
            }
        }
        const approachWaypoints = mcdu.flightPlanManager.getApproachWaypoints();
        const destination = waypointsWithDiscontinuities.pop();
        for (let i = firstApproach; i < approachWaypoints.length; i++) {
            const prev = waypointsWithDiscontinuities[waypointsWithDiscontinuities.length - 1];
            const wp = approachWaypoints[i];
            if (wp && wp.ident != "USR") {
                if (!prev || (prev.wp && prev.wp.ident != wp.ident)) {
                    waypointsWithDiscontinuities.push({
                        wp: wp,
                        fpIndex: mcdu.flightPlanManager.getWaypointsCount() - 1 + i
                    });
                }
            }
        }
        if (destination) {
            waypointsWithDiscontinuities.push(destination);
        }
        if (mcdu.flightPlanManager.decelWaypoint) {
            waypointsWithDiscontinuities.splice(mcdu.flightPlanManager.decelPrevIndex + 1, 0, {
                wp: mcdu.flightPlanManager.decelWaypoint,
                fpIndex: -42
            });
        }
        if (waypointsWithDiscontinuities.length === 0) {
            rowsCount = 0;
            originIdentCell = "";
            rows = [
                ["", "SPD/ALT", "TIME"],
                ["PPOS", "---/ ---", "----"],
                [""],
                ["---F-PLN DISCONTINUITY---"],
                [""],
                ["------END OF F-PLN-------"],
                [""],
                ["-----NO ALTN F-PLN-------"],
                [""],
                [""],
                ["DEST", "DIST EFOB", "TIME"],
                ["------", "---- ----", "----"]
            ];
        }
        let iWaypoint = offset;
        let lastAltitudeConstraint = "";
        let lastSpeedConstraint = "";
        for (let i = 0; i < rowsCount; i++) {
            if (waypointsWithDiscontinuities.length > 0) {
                while (iWaypoint >= waypointsWithDiscontinuities.length) {
                    iWaypoint -= waypointsWithDiscontinuities.length;
                }
            }
            const index = iWaypoint;
            iWaypoint++;
            if (index === 0 && first === 0) {
                rows[2 * i] = ["FROM", "SPD/ALT", isFlying ? "UTC" : "TIME"];
                rows[2 * i + 1] = [originIdentCell + "[color]" + originLineColor, "---/ ---", originTimeCell + "[color]" + originLineColor];
                mcdu.onLeftInput[i] = async () => {
                    const value = mcdu.inOut;
                    if (value === "") {
                        CDULateralRevisionPage.ShowPage(mcdu, mcdu.flightPlanManager.getOrigin(), 0);
                    }
                };
                if (i === 0) {
                    mcdu.currentFlightPlanWaypointIndex = 0;
                }
            } else if (index === waypointsWithDiscontinuities.length - 1 || (i === rowsCount - 1)) {
                let destTimeCell = "----";
                let destDistCell = "---";
                if (mcdu.flightPlanManager.getDestination()) {
                    destDistCell = mcdu.flightPlanManager.getDestination().cumulativeDistanceInFP.toFixed(0);
                    if (isFlying) {
                        destTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getDestination().estimatedTimeOfArrivalFP);
                    } else {
                        destTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getDestination().cumulativeEstimatedTimeEnRouteFP);
                    }
                    mcdu.onLeftInput[i] = () => {
                        const value = mcdu.inOut;
                        mcdu.clearUserInput();
                        if (value === "") {
                            CDULateralRevisionPage.ShowPage(mcdu, mcdu.flightPlanManager.getDestination(), mcdu.flightPlanManager.getWaypointsCount() - 1);
                        } else if (value === FMCMainDisplay.clrValue) {
                        } else if (value.length > 0) {
                            mcdu.insertWaypoint(value, mcdu.flightPlanManager.getWaypointsCount() - 1, () => {
                                CDUFlightPlanPage.ShowPage(mcdu, offset);
                            });
                        }
                    };
                }
                rows[2 * i] = ["DEST", "DIST EFOB", isFlying ? "UTC" : "TIME"];
                rows[2 * i + 1] = [destCell, destDistCell + " ----", destTimeCell];
                i++;
                if (i < rowsCount) {
                    rows[2 * i + 1] = ["------END OF F-PLN-------"];
                }
                if (i === 0) {
                    mcdu.currentFlightPlanWaypointIndex = mcdu.flightPlanManager.getWaypointsCount() + approachWaypoints.length - 1;
                }
            } else if (index < waypointsWithDiscontinuities.length - 1) {
                let prevWaypoint;
                let waypoint;
                let fpIndex = 0;
                if (waypointsWithDiscontinuities[index]) {
                    waypoint = waypointsWithDiscontinuities[index].wp;
                    if (waypointsWithDiscontinuities[index - 1]) {
                        prevWaypoint = waypointsWithDiscontinuities[index - 1].wp;
                    }
                    fpIndex = waypointsWithDiscontinuities[index].fpIndex;
                }
                if (i === 0) {
                    mcdu.currentFlightPlanWaypointIndex = fpIndex;
                }
                if (!waypoint) {
                    console.error("Should not reach.");
                } else {
                    let timeCell = "----";
                    if (isFlying) {
                        if (isFinite(waypoint.timeWasReached)) {
                            timeCell = FMCMainDisplay.secondsTohhmm(waypoint.timeWasReached);
                        } else if (isFinite(waypoint.estimatedTimeOfArrivalFP)) {
                            timeCell = FMCMainDisplay.secondsTohhmm(waypoint.estimatedTimeOfArrivalFP);
                        }
                    } else {
                        if (isFinite(waypoint.cumulativeEstimatedTimeEnRouteFP)) {
                            timeCell = FMCMainDisplay.secondsTohhmm(waypoint.cumulativeEstimatedTimeEnRouteFP);
                        }
                    }
                    if (fpIndex > mcdu.flightPlanManager.getDepartureWaypointsCount()) {
                        if (fpIndex < mcdu.flightPlanManager.getWaypointsCount() - mcdu.flightPlanManager.getArrivalWaypointsCount()) {
                            if (waypoint.infos.airwayIdentInFP === "") {
                                const prevWaypointWithDiscontinuity = waypointsWithDiscontinuities[index - 1];
                                let prevWaypoint;
                                if (prevWaypointWithDiscontinuity) {
                                    prevWaypoint = prevWaypointWithDiscontinuity.wp;
                                }
                            }
                        }
                    }
                    if (i < rowsCount - 1) {
                        let airwayName = "";
                        if (prevWaypoint && waypoint) {
                            const airway = IntersectionInfo.GetCommonAirway(prevWaypoint, waypoint);
                            if (airway) {
                                airwayName = airway.name;
                            }
                        }
                        let speedConstraint = "---";
                        if (waypoint.speedConstraint > 10) {
                            speedConstraint = waypoint.speedConstraint.toFixed(0);
                            if (speedConstraint === lastSpeedConstraint) {
                                speedConstraint = " \" ";
                            } else {
                                lastSpeedConstraint = speedConstraint;
                            }
                        }
                        let altitudeConstraint = "---";
                        if (waypoint.legAltitudeDescription !== 0) {
                            if (waypoint.legAltitudeDescription === 1) {
                                altitudeConstraint = waypoint.legAltitude1.toFixed(0);
                            }
                            if (waypoint.legAltitudeDescription === 2) {
                                altitudeConstraint = waypoint.legAltitude1.toFixed(0) + "A";
                            }
                            if (waypoint.legAltitudeDescription === 3) {
                                altitudeConstraint = waypoint.legAltitude1.toFixed(0) + "B";
                            } else if (waypoint.legAltitudeDescription === 4) {
                                altitudeConstraint = ((waypoint.legAltitude1 + waypoint.legAltitude2) * 0.5).toFixed(0);
                            }
                        } else if (index < routeFirstWaypointIndex) {
                            if (index === routeFirstWaypointIndex - 1) {
                                altitudeConstraint = "FL" + mcdu.cruiseFlightLevel;
                            } else {
                            }
                        } else if ((index === routeFirstWaypointIndex - 1) || (index === routeLastWaypointIndex + 1)) {
                            altitudeConstraint = "FL" + mcdu.cruiseFlightLevel;
                        } else {
                            if (index >= routeFirstWaypointIndex && index <= routeLastWaypointIndex) {
                                altitudeConstraint = "FL" + mcdu.cruiseFlightLevel;
                            }
                        }
                        if (altitudeConstraint === lastAltitudeConstraint) {
                            altitudeConstraint = "  \"  ";
                        } else {
                            lastAltitudeConstraint = altitudeConstraint;
                        }
                        let color = "green";
                        let airwayNameColor = "white";
                        const activeWaypoint = mcdu.flightPlanManager.getActiveWaypoint(false, true);
                        if (mcdu.flightPlanManager.getCurrentFlightPlanIndex() === 1) {
                            color = "yellow";
                            airwayNameColor = "yellow";
                        } else if (activeWaypoint && (waypoint.icaoNoSpace === activeWaypoint.icaoNoSpace)) {
                            color = "white";
                        }
                        rows[2 * i] = [airwayName + "[color]" + airwayNameColor, waypoint.cumulativeDistanceInFP.toFixed(0) + "[color]" + color];
                        rows[2 * i + 1] = [waypoint.ident + "[color]" + color, speedConstraint + "/" + altitudeConstraint + "[s-text][color]" + color, timeCell + "[color]" + color];
                        if (fpIndex !== -42) {
                            mcdu.onLeftInput[i] = async () => {
                                const value = mcdu.inOut;
                                mcdu.clearUserInput();
                                if (value === "") {
                                    if (waypoint) {
                                        CDULateralRevisionPage.ShowPage(mcdu, waypoint, fpIndex);
                                    }
                                } else if (value === FMCMainDisplay.clrValue) {
                                    mcdu.removeWaypoint(fpIndex, () => {
                                        CDUFlightPlanPage.ShowPage(mcdu, offset);
                                    });
                                } else if (value.length > 0) {
                                    mcdu.insertWaypoint(value, fpIndex, () => {
                                        CDUFlightPlanPage.ShowPage(mcdu, offset);
                                    });
                                }
                            };
                            mcdu.onRightInput[i] = async () => {
                                if (waypoint) {
                                    CDUVerticalRevisionPage.ShowPage(mcdu, waypoint);
                                }
                            };
                        }
                    } else {
                        let destTimeCell = "----";
                        let destDistCell = "---";
                        if (mcdu.flightPlanManager.getDestination()) {
                            destDistCell = mcdu.flightPlanManager.getDestination().infos.totalDistInFP.toFixed(0);
                            if (isFlying) {
                                destTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getDestination().estimatedTimeOfArrivalFP);
                            } else {
                                destTimeCell = FMCMainDisplay.secondsTohhmm(mcdu.flightPlanManager.getDestination().cumulativeEstimatedTimeEnRouteFP);
                            }
                        }
                        rows[2 * i] = ["DEST", "DIST EFOB", isFlying ? "UTC" : "TIME"];
                        rows[2 * i + 1] = [destCell, destDistCell + " ----", destTimeCell];
                        mcdu.onLeftInput[i] = () => {
                            CDULateralRevisionPage.ShowPage(mcdu, mcdu.flightPlanManager.getDestination(), mcdu.flightPlanManager.getWaypointsCount() - 1);
                        };
                    }
                }
            }
        }
        const wpCount = mcdu.flightPlanManager.getWaypointsCount() + approachWaypoints.length;
        if (wpCount > 0) {
            while (mcdu.currentFlightPlanWaypointIndex < 0) {
                mcdu.currentFlightPlanWaypointIndex += wpCount;
            }
            while (mcdu.currentFlightPlanWaypointIndex >= wpCount) {
                mcdu.currentFlightPlanWaypointIndex -= wpCount;
            }
        }
        mcdu.setTemplate([
            ["FROM " + originIdentCell],
            ...rows
        ]);
        mcdu.onDown = () => {
            offset = Math.max(offset - 1, 0);
            CDUFlightPlanPage.ShowPage(mcdu, offset);
        };
        mcdu.onUp = () => {
            offset++;
            CDUFlightPlanPage.ShowPage(mcdu, offset);
        };
    }
}
CDUFlightPlanPage._timer = 0;
//# sourceMappingURL=A320_Neo_CDU_FlightPlanPage.js.map