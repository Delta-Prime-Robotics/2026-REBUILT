// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Locale;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AllianceShiftTracker {
  private static final double AUTO_DURATION = 20.0;
  private static final double TRANSITION_DURATION = 10.0;
  private static final double SHIFT_DURATION = 25.0;
  private static final int SHIFT_COUNT = 4;
  private static final double ENDGAME_DURATION = 30.0;
  private static final double TELEOP_DURATION =
      TRANSITION_DURATION + (SHIFT_DURATION * SHIFT_COUNT) + ENDGAME_DURATION;

  private AutoWinner latchedAutoWinner = AutoWinner.UNKNOWN;

  public void update() {
    double matchTime = DriverStation.getMatchTime();
    Optional<Alliance> myAlliance = DriverStation.getAlliance();
    if (DriverStation.isDisabled() && matchTime <= 0) {
      latchedAutoWinner = AutoWinner.UNKNOWN;
    }
    AutoWinner autoWinner = updateAutoWinner(DriverStation.getGameSpecificMessage());

    ShiftState shiftState = computeShiftState(matchTime, autoWinner);
    boolean isMyAllianceActive =
        myAlliance.map(alliance -> shiftState.activeAlliance.isActiveFor(alliance)).orElse(false);

    Logger.recordOutput("AllianceShift/MatchTimeRemaining", matchTime);
    Logger.recordOutput("AllianceShift/AutoWinner", autoWinner.displayName);
    Logger.recordOutput("AllianceShift/CurrentPhase", shiftState.phaseName);
    Logger.recordOutput("AllianceShift/CurrentActive", shiftState.activeAlliance.displayName);
    Logger.recordOutput("AllianceShift/NextActive", shiftState.nextActiveAlliance.displayName);
    Logger.recordOutput("AllianceShift/SecondsToNextShift", shiftState.secondsToNextShift);
    Logger.recordOutput("AllianceShift/MyAlliance", formatAlliance(myAlliance));
    Logger.recordOutput("AllianceShift/IsMyAllianceActive", isMyAllianceActive);
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
