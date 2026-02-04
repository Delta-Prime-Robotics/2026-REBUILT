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

public class BeltNadoIOSpark implements BeltNadoIO {
  private static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private static final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final SparkMax beltLeader;
  private final SparkMax beltFollower;

  static {
    leaderConfig
        .smartCurrentLimit(MotorConstants.kNeo550SmartCurrentLimit)
        .idleMode(IdleMode.kBrake);

    followerConfig.apply(leaderConfig).follow(CanIdsOtherThanDrive.kBeltNadoLeaderId, false);
  }

  public BeltNadoIOSpark() {
    beltLeader =
        new SparkMax(CanIdsOtherThanDrive.kBeltNadoLeaderId, SparkMax.MotorType.kBrushless);
    beltLeader.configureAsync(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    beltFollower =
        new SparkMax(CanIdsOtherThanDrive.kBeltNadoFollowerId, SparkMax.MotorType.kBrushless);
    beltFollower.configureAsync(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(BeltNadoIOInputs inputs) {
    inputs.connected = true;
    inputs.dutyCycle = beltLeader.getAppliedOutput();
    inputs.appliedVolts = inputs.dutyCycle * beltLeader.getBusVoltage();
    inputs.velocityRpm = beltLeader.getEncoder().getVelocity();
    inputs.currentAmps = beltLeader.getOutputCurrent() + beltFollower.getOutputCurrent();
  }

  @Override
  public void setMotor(double speed) {
    beltLeader.set(speed);
  }

  @Override
  public void stop() {
    beltLeader.stopMotor();
  }
}
