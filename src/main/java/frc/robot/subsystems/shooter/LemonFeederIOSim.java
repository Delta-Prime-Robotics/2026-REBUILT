// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.Constants.MotorConstants;

public class LemonFeederIOSim implements LemonFeederIO {
  private double dutyCycle = 0.0;

  @Override
  public void updateInputs(LemonFeederIOInputs inputs) {
    inputs.connected = true;
    inputs.dutyCycle = dutyCycle;
    inputs.appliedVolts = dutyCycle * 12.0;
    inputs.velocityRpm = dutyCycle * MotorConstants.kNeoFreeSpeedRpm;
    inputs.currentAmps = 0.0;
  }

  @Override
  public void setMotor(double speed) {
    dutyCycle = speed;
  }

  @Override
  public void stop() {
    dutyCycle = 0.0;
  }
}
