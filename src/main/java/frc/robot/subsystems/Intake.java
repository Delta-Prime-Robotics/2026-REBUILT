// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
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

public class Intake extends SubsystemBase {
  private final SparkFlex m_intake; // Vortex
  private final SparkMax m_leftArm; // NEO
  private final SparkMax m_rightArm; // NEO

  private static SparkFlexConfig m_intakeConfig = new SparkFlexConfig();
  private static SparkMaxConfig m_armConfig = new SparkMaxConfig();

  private final SparkAbsoluteEncoder m_armEncoder;
  private final SparkClosedLoopController m_armController;

  public static IntakeState currentIntakeState = IntakeState.STOWED;
  public static double m_armPose = 0.0; // 0 to 1, where 0 is stowed and 1 is fully extended

  static {
    m_intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kVortexSmartCurrentLimit);

    m_armConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .inverted(false)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(0.04, 0.0, 0.0)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot0)
        .positionWrappingEnabled(false)
        .positionWrappingInputRange(IntakeConstants.kArmMinAngle, IntakeConstants.kArmMaxAngle)
        .maxMotion
        .cruiseVelocity(0.0)
        .maxAcceleration(0.0, ClosedLoopSlot.kSlot0); // rpm per second

    m_armConfig
        .absoluteEncoder
        .inverted(true)
        .positionConversionFactor(75); // 5:1, 5:1, 75:1 Encoder on finalShaft

    m_armConfig.closedLoop.feedForward.svacr(0.015, 0, 0, 0, 0);
  }

  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkFlex(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);
    m_leftArm = new SparkMax(CanIdsOtherThanDrive.kLeftArmId, SparkMax.MotorType.kBrushless);
    m_rightArm = new SparkMax(CanIdsOtherThanDrive.kRightArmId, SparkMax.MotorType.kBrushless);

    m_intake.configure(
        m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftArm.configure(
        m_armConfig.follow(CanIdsOtherThanDrive.kLeftArmId, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_rightArm.configure(
        m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_armController = m_rightArm.getClosedLoopController();

    m_armEncoder = m_rightArm.getAbsoluteEncoder();
  }

  @AutoLogOutput(key = "Intake/ArmAngle")
  public double getArmAngleZeroToOne() {
    return m_armPose;
    // return m_armEncoder.getPosition();
  }

  @AutoLogOutput(key = "Intake/State")
  public IntakeState getCurrentIntakeState() {
    return currentIntakeState;
  }

  private void setIntakeSpeed(double speed) {
    m_intake.set(speed);
    Logger.recordOutput("Intake/IntakeSpeed", speed);
  }

  private void stopIntake() {
    setIntakeSpeed(0);
    Logger.recordOutput("Intake/IntakeSpeed", 0);
  }

  private void setArmSpeed(double speed) {
    double outSpeed = 0.0;
    Logger.recordOutput("Intake/IntakeSpeed", speed);
    if ((getArmAngleZeroToOne() > IntakeConstants.kArmMinAngle)
        && (getArmAngleZeroToOne() < IntakeConstants.kArmMaxAngle)) {
      outSpeed = speed;
    } else {
      outSpeed = 0.0;
    }

    m_rightArm.set(outSpeed);
  }

  private void stopArm() {
    m_leftArm.stopMotor();
    m_rightArm.stopMotor();
  }

  private void setArmSetpoint(double setpointDeg) {
    setpointDeg =
        MathUtil.clamp(setpointDeg, IntakeConstants.kArmMinAngle, IntakeConstants.kArmMaxAngle);
    Logger.recordOutput("Intake/ArmSetpoint", setpointDeg);
    m_armController.setSetpoint(setpointDeg, ControlType.kMAXMotionPositionControl);
  }

  @AutoLogOutput(key = "Intake/isArmAtSetPoint")
  public boolean isArmAtSetpoint() {
    return m_armController.isAtSetpoint();
  }

  public Command runIntake(double speed) {
    return this.runEnd(() -> setIntakeSpeed(speed), () -> stopIntake());
  }

  public Command runArmToAngle(double armPosZeroToOne) {
    return Commands.runOnce(() -> setArmSetpoint(armPosZeroToOne), this)
        .andThen(
            Commands.waitUntil(this::isArmAtSetpoint)
                .alongWith(
                    Commands.waitSeconds(0.25)
                        .andThen(
                            runOnce(
                                () -> {
                                  m_armPose = armPosZeroToOne;
                                }))))
        .finallyDo(interrupted -> stopArm());
  }

  public Command thrustingCommand() {
    return this.runArmToAngle(IntakeConstants.kArmThrustInwardPosition)
        .andThen(this.runArmToAngle(IntakeConstants.kArmThrustOutwardPosition))
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.THRUSTING);
  }

  public Command intakingCommand(double speed) {
    return this.runArmToAngle(IntakeConstants.kArmIntakePosition)
        .alongWith(Commands.run(() -> setIntakeSpeed(speed)))
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.INTAKING);
  }

  public Command stowingCommand() {
    return Commands.sequence(
            Commands.runOnce(this::stopIntake, this),
            this.runArmToAngle(IntakeConstants.kArmStowPosition))
        .beforeStarting(() -> currentIntakeState = IntakeState.STOWED);
  }

  public Command runArmToIntakeState(IntakeState intakeState) {
    switch (intakeState) {
      case INTAKING:
        return intakingCommand(0.75);
      case OUTTAKING:
        return intakingCommand(-0.75);
      case STOWED:
        return stowingCommand();
      case THRUSTING:
        return thrustingCommand();
      default:
        return run(
            () -> {
              stopArm();
              stopIntake();
            });
    }
  }
}

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
