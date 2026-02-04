// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CanIdsOtherThanDrive;
import frc.robot.Constants.MotorConstants;

public class FlywheelShooterIOSpark implements FlywheelShooterIO {
  private static final double kShooterAllowableErrorRPM = 50.0; // TODO: tune

  private static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private static final SparkMaxConfig followerConfig = new SparkMaxConfig();

  private final SparkMax shooterLeader;
  private final SparkMax shooterFollower;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  private double setpointRpm = 0.0;

  static {
    leaderConfig
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .idleMode(IdleMode.kCoast);

    leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .allowedClosedLoopError(kShooterAllowableErrorRPM, ClosedLoopSlot.kSlot0)
        .maxMotion
        .maxAcceleration(0.0, ClosedLoopSlot.kSlot0);

    leaderConfig.closedLoop.feedForward.sva(
        0.0, // ks
        0.0, // kv
        0.0 // ka
        );

    followerConfig.apply(leaderConfig).follow(CanIdsOtherThanDrive.kShooterLeaderId, false);
  }

  public FlywheelShooterIOSpark() {
    shooterLeader =
        new SparkMax(CanIdsOtherThanDrive.kShooterLeaderId, SparkMax.MotorType.kBrushless);
    shooterFollower =
        new SparkMax(CanIdsOtherThanDrive.kShooterFollowerId, SparkMax.MotorType.kBrushless);

    encoder = shooterLeader.getEncoder();
    encoder.setPosition(0.0);

    shooterLeader.configureAsync(
        leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollower.configureAsync(
        followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    shooterLeader.setCANMaxRetries(5);
    shooterFollower.setCANMaxRetries(5);
    shooterLeader.setCANTimeout(10);
    shooterFollower.setCANTimeout(10);

    controller = shooterLeader.getClosedLoopController();
  }

  @Override
  public void updateInputs(FlywheelShooterIOInputs inputs) {
    inputs.connected = true;
    inputs.velocityRpm = encoder.getVelocity();
    inputs.appliedVolts = shooterLeader.getAppliedOutput() * shooterLeader.getBusVoltage();
    inputs.currentAmps = shooterLeader.getOutputCurrent() + shooterFollower.getOutputCurrent();
    inputs.setpointRpm = setpointRpm;
  }

  @Override
  public void setVelocityRpm(double velocityRpm) {
    setpointRpm = velocityRpm;
    controller.setSetpoint(setpointRpm, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    setVelocityRpm(0.0);
  }
}
