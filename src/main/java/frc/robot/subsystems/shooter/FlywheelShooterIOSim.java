// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.Constants.MotorConstants;

public class FlywheelShooterIOSim implements FlywheelShooterIO {
  private double setpointRpm = 0.0;

  @Override
  public void updateInputs(FlywheelShooterIOInputs inputs) {
    inputs.connected = true;
    inputs.setpointRpm = setpointRpm;
    inputs.velocityRpm = setpointRpm;
    inputs.appliedVolts = (setpointRpm / MotorConstants.kNeoFreeSpeedRpm) * 12.0;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setVelocityRpm(double velocityRpm) {
    setpointRpm = velocityRpm;
  }

  @Override
  public void stop() {
    setpointRpm = 0.0;
  }
}
