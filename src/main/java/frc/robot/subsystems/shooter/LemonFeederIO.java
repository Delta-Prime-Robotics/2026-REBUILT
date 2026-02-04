// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface LemonFeederIO {
  @AutoLog
  class LemonFeederIOInputs {
    public boolean connected = false;
    public double dutyCycle = 0.0;
    public double appliedVolts = 0.0;
    public double velocityRpm = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(LemonFeederIOInputs inputs) {}

  /** Run the feeder at the specified speed (-1 to 1). */
  default void setMotor(double speed) {}

  /** Stop the feeder. */
  default void stop() {}
}
