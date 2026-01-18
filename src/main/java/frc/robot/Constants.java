// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
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

  public static final boolean kDriveTypeisNeo = true; // true for NEO, false for Vortex

  public static final class MotorConstants {
    // Kv Values
    public static final int kVortexKv = 565;
    // Free Speeds
    public static final int kNeoFreeSpeedRpm = 5676;
    public static final int kVortexFreeSpeedRpm = 6784;
    // Smart Current Limits
    public static final int kNeo550SetCurrent = 20; // amps
    public static final int kNeoSetCurrent = 50; // amps
    public static final int kVortexSetCurrent = 50; // amps
  }

  public static final double fieldCenterY = Units.inchesToMeters(158.32); // Inches
}
