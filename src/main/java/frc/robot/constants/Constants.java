// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }
  // Poseitioning Constants
  // public static final double fieldCenterY = Units.inchesToMeters(158.32); // Inches

  public static final class MotorConstants {
    // Kv Values
    public static final int kVortexKv = 565;
    // Free Speeds
    public static final int kNeoFreeSpeedRpm = 5676;
    public static final int kVortexFreeSpeedRpm = 6784;
    // Smart Current Limits
    public static final int kNeo550SmartCurrentLimit = 30; // amps
    public static final int kNeoSmartCurrentLimit = 50; // amps
    public static final int kVortexSmartCurrentLimit = 50; // amps
  }

  public static final class CanIdsOtherThanDrive {
    // 0 is reserved for RoboRIO
    // 1 isnt reserved, but is commonly used devices that cant change their ID
    // Ids 2-9 are reserved for drive motors

    // Intake
    public static final int kIntakeId = 10;
    public static final int kLeftArmId = 11;
    public static final int kRightArmId = 12;

    // Belt Conveyor
    public static final int kBeltnadoMotorId = 13;

    // Kicker / Indexer
    public static final int kKickdexerBottomMotorId = 14;
    public static final int kKickdexerTopMotorId = 15;

    // Shooter
    public static final int kShooterLeaderMotorId = 16;
    // public static final int kShooterFollowerMotorId = 16;
  }

  public static final class IntakeConstants {
    public static final double kArmStowPosition = 22;
    public static final double kArmIntakePosition = 1;
    public static final double kArmThrustInwardPosition = 10;
    public static final double kArmThrustOutwardPosition = 10;
    public static final double kArmMinAngle = 0.9;
    public static final double kArmMaxAngle = 22.8;

    public static final double kIntakeSpeed = 0.2;
    public static final double kOuttakeSpeed = -0.2;
    public static final double kStopSpeed = 0;

    public enum IntakeState {
      INTAKING,
      OUTTAKING,
      STOWED,
      THRUSTING
    }
  }
}
