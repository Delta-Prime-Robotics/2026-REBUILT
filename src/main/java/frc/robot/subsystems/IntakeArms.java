// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanIdsOtherThanDrive;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.constants.Constants.IntakeConstants.*;
import frc.robot.constants.Constants.MotorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// We want it take the fuel from the floor into the robot. We might use a laser break sensor and we
// will need sparkmax motor controllers. And put encoder on the controllers.

public class IntakeArms extends SubsystemBase {
  private final SparkMax m_leftArm; // NEO
  private final SparkMax m_rightArm; // NEO

  private static SparkMaxConfig m_rightArmConfig = new SparkMaxConfig();
  private static SparkMaxConfig m_leftArmConfig = new SparkMaxConfig();

  private final RelativeEncoder m_rightArmEncoder;
  private final RelativeEncoder m_leftArmEncoder;
  private final SparkClosedLoopController m_rightArmController;
  private final SparkClosedLoopController m_leftArmController;

  public static IntakeState currentIntakeState = IntakeState.STOWED;

  static {
    // Arm config
    m_rightArmConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .inverted(true);

    m_leftArmConfig.apply(m_rightArmConfig);

    // Right Arm Config
    m_rightArmConfig
        .closedLoop
        .outputRange(-0.5, 0.5)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0.0, 0.0)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(IntakeConstants.kArmMinAngle, IntakeConstants.kArmMaxAngle)
        .maxMotion
        .cruiseVelocity(0.0)
        .maxAcceleration(0.0, ClosedLoopSlot.kSlot0); // rpm per second
    m_rightArmConfig.closedLoop.feedForward.svacr(0.015, 0, 0, 0, 0);

    // Left Arm Config
    m_leftArmConfig
        .inverted(false)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.04, 0.0, 0.0)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(IntakeConstants.kArmMinAngle, IntakeConstants.kArmMaxAngle)
        .maxMotion
        .cruiseVelocity(0.0)
        .maxAcceleration(0.0, ClosedLoopSlot.kSlot0); // rpm per second
    m_leftArmConfig.closedLoop.feedForward.svacr(0.015, 0, 0, 0, 0);

    // m_armConfig
    //     .absoluteEncoder
    //     .inverted(true)
    //     .positionConversionFactor(75); // 5:1, 5:1, 75:1 Encoder on finalShaft

  }

  /** Creates a new Intake. */
  public IntakeArms() {

    m_leftArm = new SparkMax(CanIdsOtherThanDrive.kLeftArmId, SparkMax.MotorType.kBrushless);
    m_rightArm = new SparkMax(CanIdsOtherThanDrive.kRightArmId, SparkMax.MotorType.kBrushless);

    m_leftArm.configure(
        m_leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightArm.configure(
        m_rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_leftArmController = m_leftArm.getClosedLoopController();
    m_rightArmController = m_rightArm.getClosedLoopController();

    m_leftArmEncoder = m_leftArm.getEncoder();
    m_rightArmEncoder = m_rightArm.getEncoder();
  }

  @AutoLogOutput(key = "Intake/LeftArmAngle")
  public double getLeftArmAngle() {
    return m_leftArmEncoder.getPosition();
  }

  @AutoLogOutput(key = "Intake/RightArmAngle")
  public double getRightArmAngle() {
    return m_rightArmEncoder.getPosition();
  }

  @AutoLogOutput(key = "Intake/State")
  public IntakeState getCurrentIntakeState() {
    return currentIntakeState;
  }

  private void zeroArmPose() {
    m_leftArmEncoder.setPosition(0);
    m_rightArmEncoder.setPosition(0);
  }

  private void setArmSpeed(SparkMax arm, double speed) {
    arm.set(speed);
  }

  private void setArmSpeeds(double speed) {
    setArmSpeed(m_leftArm, speed);
    setArmSpeed(m_rightArm, speed);
  }

  private void stopArm() {
    m_leftArm.stopMotor();
    m_rightArm.stopMotor();
  }

  private void setArmSetpoint(double setpointDeg) {
    setpointDeg =
        MathUtil.clamp(setpointDeg, IntakeConstants.kArmMinAngle, IntakeConstants.kArmMaxAngle);
    Logger.recordOutput("Intake/ArmSetpoint", setpointDeg);
    m_leftArmController.setSetpoint(setpointDeg, ControlType.kPosition);
    m_rightArmController.setSetpoint(setpointDeg, ControlType.kPosition);
  }

  @AutoLogOutput(key = "Intake/isArmAtSetPoint")
  public boolean isArmsAtSetpoint() {
    return m_leftArmController.isAtSetpoint() && m_rightArmController.isAtSetpoint();
  }

  public Command zeroArmPoseCommand() {
    return runOnce(() -> zeroArmPose());
  }

  public Command runArmWithSpeedsCommand(double speed) {
    return runEnd(() -> setArmSpeeds(speed), () -> stopArm());
  }

  public Command runArmToAngleCommand(double armPosZeroToOne) {
    return this.run(() -> setArmSetpoint(armPosZeroToOne))
        .until(this::isArmsAtSetpoint)
        .finallyDo(() -> stopArm());
  }

  public Command thrustingCommand() {
    double rateSecs = 1;
    return Commands.repeatingSequence(
            this.runArmToAngleCommand(IntakeConstants.kArmThrustInwardPosition),
            Commands.waitSeconds(rateSecs),
            this.runArmToAngleCommand(IntakeConstants.kArmThrustOutwardPosition),
            Commands.waitSeconds(rateSecs))
        .finallyDo(
            () -> {
              stopArm();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.THRUSTING);
  }

  public Command intakingCommand() {
    return this.runArmToAngleCommand(IntakeConstants.kArmIntakePosition)
        .finallyDo(
            () -> {
              stopArm();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.INTAKING);
  }

  public Command stowingCommand() {
    return this.runArmToAngleCommand(IntakeConstants.kArmStowPosition)
        .finallyDo(
            () -> {
              stopArm();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.STOWED);
  }

  public Command stopArmCommand() {
    return this.run(() -> stopArm());
  }

  public Command runArmToIntakeStateCommand(IntakeState intakeState) {
    switch (intakeState) {
      case INTAKING:
        return intakingCommand();
      case OUTTAKING:
        return intakingCommand();
      case STOWED:
        return stowingCommand();
      case THRUSTING:
        return thrustingCommand();
      default:
        return run(
            () -> {
              stopArm();
            });
    }
  }
}

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
