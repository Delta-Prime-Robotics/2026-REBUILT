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

public class IndexerIOSpark implements IndexerIO {
  private static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

  private final SparkMax indexer;

  static {
    indexerConfig
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .idleMode(IdleMode.kBrake);
  }

  public IndexerIOSpark() {
    indexer = new SparkMax(CanIdsOtherThanDrive.kIndexerId, SparkMax.MotorType.kBrushless);
    indexer.configureAsync(
        indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected = true;
    inputs.dutyCycle = indexer.getAppliedOutput();
    inputs.appliedVolts = inputs.dutyCycle * indexer.getBusVoltage();
    inputs.velocityRpm = indexer.getEncoder().getVelocity();
    inputs.currentAmps = indexer.getOutputCurrent();
  }

  @Override
  public void setMotor(double speed) {
    indexer.set(speed);
  }

  @Override
  public void stop() {
    indexer.stopMotor();
  }
}
