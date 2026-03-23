// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Haptics;
import java.util.HashSet;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class AllianceShiftTracker {
  private static final double UPDATE_INTERVAL_SECONDS = 1.0;
  private static final double AUTO_DURATION = 20.0;
  private static final double TRANSITION_DURATION = 10.0;
  private static final double SHIFT_DURATION = 25.0;
  private static final int SHIFT_COUNT = 4;
  private static final double ENDGAME_DURATION = 30.0;
  private static final double TELEOP_DURATION =
      TRANSITION_DURATION + (SHIFT_DURATION * SHIFT_COUNT) + ENDGAME_DURATION;

  private AutoWinner latchedAutoWinner = AutoWinner.UNKNOWN;
  private Optional<Alliance> myAlliance;
  private boolean isMyAllianceActive = false;
  private Haptics[] haptics;
  private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  private double lastTelemetryUpdateTimestamp = Double.NEGATIVE_INFINITY;

  private final Set<String> warnedShiftPhases = new HashSet<>();

  public AllianceShiftTracker() {}

  public AllianceShiftTracker(Haptics... haptics) {
    this.haptics = haptics;
  }

  public void update() {
    double now = Timer.getFPGATimestamp();
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isDisabled() && matchTime <= 0) {
      latchedAutoWinner = AutoWinner.UNKNOWN;
      warnedShiftPhases.clear();
      myAlliance = DriverStation.getAlliance();
    }
    if (matchTime >= 130){ //130 seconds
      updateAutoWinner(DriverStation.getGameSpecificMessage());
      myAlliance = DriverStation.getAlliance();
    }

    ShiftState shiftState = computeShiftState(matchTime, latchedAutoWinner);
    maybeRunShiftWarning(shiftState);
    isMyAllianceActive =
        myAlliance.map(alliance -> shiftState.activeAlliance.isActiveFor(alliance)).orElse(false);

    if (now - lastTelemetryUpdateTimestamp >= UPDATE_INTERVAL_SECONDS) {
      lastTelemetryUpdateTimestamp = now;
      logTelemetry(matchTime, latchedAutoWinner, shiftState, myAlliance, isMyAllianceActive);
    }
  }

  private static void logTelemetry(
      double matchTime,
      AutoWinner autoWinner,
      ShiftState shiftState,
      Optional<Alliance> myAlliance,
      boolean isMyAllianceActive) {
    Logger.recordOutput("AllianceShift/MatchTimeRemaining", matchTime);
    Logger.recordOutput("AllianceShift/AutoWinner", autoWinner.displayName);
    Logger.recordOutput("AllianceShift/CurrentPhase", shiftState.phaseName);
    Logger.recordOutput("AllianceShift/CurrentActive", shiftState.activeAlliance.displayName);
    Logger.recordOutput("AllianceShift/NextActive", shiftState.nextActiveAlliance.displayName);
    Logger.recordOutput("AllianceShift/SecondsToNextShift", shiftState.secondsToNextShift);
    Logger.recordOutput("AllianceShift/MyAlliance", formatAlliance(myAlliance));
    Logger.recordOutput("AllianceShift/IsMyAllianceActive", formatAllianceActivity(isMyAllianceActive));
  }

  private static ShiftState computeShiftState(double matchTime, AutoWinner autoWinner) {
    if (matchTime <= 0) {
      return new ShiftState("Unknown", ActiveAlliance.UNKNOWN, ActiveAlliance.UNKNOWN, 0.0);
    }

    if (DriverStation.isAutonomousEnabled()) {
      double secondsLeft = Math.min(matchTime, AUTO_DURATION);
      return new ShiftState("Autonomous", ActiveAlliance.BOTH, ActiveAlliance.BOTH, secondsLeft);
    }

    if (DriverStation.isTeleopEnabled()) {
      double teleopRemaining = Math.min(matchTime, TELEOP_DURATION);
      double teleopElapsed = TELEOP_DURATION - teleopRemaining;

      if (teleopElapsed < TRANSITION_DURATION) {
        double secondsLeft = TRANSITION_DURATION - teleopElapsed;
        ActiveAlliance nextActive = activeAllianceForShift(1, autoWinner, ActiveAlliance.UNKNOWN);
        return new ShiftState("Transition", ActiveAlliance.BOTH, nextActive, secondsLeft);
      }

      double shiftElapsed = teleopElapsed - TRANSITION_DURATION;
      double shiftsTotal = SHIFT_DURATION * SHIFT_COUNT;
      if (shiftElapsed < shiftsTotal) {
        int shiftIndex = (int) (shiftElapsed / SHIFT_DURATION) + 1;
        double secondsLeft = SHIFT_DURATION - (shiftElapsed - ((shiftIndex - 1) * SHIFT_DURATION));
        ActiveAlliance active =
            activeAllianceForShift(shiftIndex, autoWinner, ActiveAlliance.UNKNOWN);
        ActiveAlliance nextActive =
            (shiftIndex < SHIFT_COUNT)
                ? activeAllianceForShift(shiftIndex + 1, autoWinner, ActiveAlliance.UNKNOWN)
                : ActiveAlliance.BOTH;
        return new ShiftState("Shift " + shiftIndex, active, nextActive, secondsLeft);
      }

      double endgameElapsed = shiftElapsed - shiftsTotal;
      if (endgameElapsed < ENDGAME_DURATION) {
        double secondsLeft = ENDGAME_DURATION - endgameElapsed;
        return new ShiftState("Endgame", ActiveAlliance.BOTH, ActiveAlliance.UNKNOWN, secondsLeft);
      }

      return new ShiftState("PostMatch", ActiveAlliance.UNKNOWN, ActiveAlliance.UNKNOWN, 0.0);
    }

    return new ShiftState("Disabled", ActiveAlliance.UNKNOWN, ActiveAlliance.UNKNOWN, 0.0);
  }

  private void maybeRunShiftWarning(ShiftState shiftState) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    boolean shouldWarnEndGame = 
      shiftState.secondsToNextShift <= 15.0 &&
      shiftState.secondsToNextShift > 0.0 &&
      "Endgame".equals(shiftState.phaseName);

     boolean shouldWarnShift =
        shiftState.secondsToNextShift > 0.0 && shiftState.secondsToNextShift <= 6.0;

    if (shouldWarnEndGame && warnedShiftPhases.add(shiftState.phaseName)) {
      commandScheduler.schedule( 
        haptics[0].endGame(),
        haptics[1].endGame());
      return;
    }

    if (shouldWarnShift && warnedShiftPhases.add(shiftState.phaseName)) {
      System.out.println(shiftState.phaseName);
      if("Shift 4".equals(shiftState.phaseName)) {
        commandScheduler.schedule( 
        haptics[0].shiftChangeToMyAlliance(),
        haptics[1].shiftChangeToMyAlliance());
        return;
      }
      if (!isMyAllianceActive) {
        commandScheduler.schedule( 
        haptics[0].shiftChangeToMyAlliance(),
        haptics[1].shiftChangeToMyAlliance());
        return;
      }
      if (isMyAllianceActive) {
        commandScheduler.schedule( 
        haptics[0].shiftChangeOutOfMyAlliance(),
        haptics[1].shiftChangeOutOfMyAlliance());
        return;
      }
    }
  }

  private static ActiveAlliance activeAllianceForShift(
      int shiftIndex, AutoWinner autoWinner, ActiveAlliance fallback) {
    if (autoWinner == AutoWinner.UNKNOWN) {
      return fallback;
    }

    boolean autoWinnerGetsShift = (shiftIndex % 2 == 0);
    return autoWinnerGetsShift ? autoWinner.toActiveAlliance() : autoWinner.opponent();
  }

  private AutoWinner updateAutoWinner(String gameData) {
    if (gameData == null || gameData.isBlank()) {
      return latchedAutoWinner;
    }

    String normalized = gameData.trim().toUpperCase(Locale.ROOT);
    if (normalized.startsWith("B")) {
      latchedAutoWinner = AutoWinner.BLUE;
      return latchedAutoWinner;
    }
    if (normalized.startsWith("R")) {
      latchedAutoWinner = AutoWinner.RED;
      return latchedAutoWinner;
    }
    return latchedAutoWinner;
  }

  private static String formatAlliance(Optional<Alliance> alliance) {
    return alliance.map(AllianceShiftTracker::formatAlliance).orElse("Unknown");
  }

  private static String formatAlliance(Alliance alliance) {
    return alliance == Alliance.Blue ? "Blue" : "Red";
  }

  private static String formatAllianceActivity(boolean isMyAllianceActive) {
    return isMyAllianceActive ? "SHOOOOOT" : "DO NOT SHOOT";
  }

  private enum AutoWinner {
    BLUE("Blue"),
    RED("Red"),
    UNKNOWN("Unknown");

    private final String displayName;

    AutoWinner(String displayName) {
      this.displayName = displayName;
    }

    private ActiveAlliance toActiveAlliance() {
      return this == BLUE ? ActiveAlliance.BLUE : ActiveAlliance.RED;
    }

    private ActiveAlliance opponent() {
      return this == BLUE ? ActiveAlliance.RED : ActiveAlliance.BLUE;
    }
  }

  private enum ActiveAlliance {
    BLUE("Blue"),
    RED("Red"),
    BOTH("Both"),
    UNKNOWN("Unknown");

    private final String displayName;

    ActiveAlliance(String displayName) {
      this.displayName = displayName;
    }

    private boolean isActiveFor(Alliance alliance) {
      if (this == BOTH) {
        return true;
      }
      if (this == BLUE) {
        return alliance == Alliance.Blue;
      }
      if (this == RED) {
        return alliance == Alliance.Red;
      }
      return false;
    }
  }

  private static class ShiftState {
    private final String phaseName;
    private final ActiveAlliance activeAlliance;
    private final ActiveAlliance nextActiveAlliance;
    private final double secondsToNextShift;

    private ShiftState(
        String phaseName,
        ActiveAlliance activeAlliance,
        ActiveAlliance nextActiveAlliance,
        double secondsToNextShift) {
      this.phaseName = phaseName;
      this.activeAlliance = activeAlliance;
      this.nextActiveAlliance = nextActiveAlliance;
      this.secondsToNextShift = secondsToNextShift;
    }
  }
}
