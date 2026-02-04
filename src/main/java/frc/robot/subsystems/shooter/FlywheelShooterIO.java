// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelShooterIO {
  @AutoLog
  class FlywheelShooterIOInputs {
    public boolean connected = false;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double setpointRpm = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(FlywheelShooterIOInputs inputs) {}

  /** Run the flywheel at the specified velocity in RPM. */
  default void setVelocityRpm(double velocityRpm) {}

  /** Stop the flywheel. */
  default void stop() {}
}
