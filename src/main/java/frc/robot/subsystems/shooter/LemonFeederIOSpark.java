// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CanIdsOtherThanDrive;
import frc.robot.Constants.MotorConstants;

public class LemonFeederIOSpark implements LemonFeederIO {
  private static final SparkMaxConfig feederConfig = new SparkMaxConfig();

  private final SparkMax feeder;

  static {
    feederConfig
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .idleMode(IdleMode.kBrake);
  }

  public LemonFeederIOSpark() {
    feeder = new SparkMax(CanIdsOtherThanDrive.kLemonFeederId, SparkMax.MotorType.kBrushless);
    feeder.configureAsync(
        feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(LemonFeederIOInputs inputs) {
    inputs.connected = true;
    inputs.dutyCycle = feeder.getAppliedOutput();
    inputs.appliedVolts = inputs.dutyCycle * feeder.getBusVoltage();
    inputs.velocityRpm = feeder.getEncoder().getVelocity();
    inputs.currentAmps = feeder.getOutputCurrent();
  }

  @Override
  public void setMotor(double speed) {
    feeder.set(speed);
  }

  @Override
  public void stop() {
    feeder.stopMotor();
  }
}
