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
  private static SparkMaxConfig m_rightArmConfig = new SparkMaxConfig();
  private static SparkMaxConfig m_leftArmConfig = new SparkMaxConfig();

  private final RelativeEncoder m_rightArmEncoder;
  private final RelativeEncoder m_leftArmEncoder;
  private final SparkClosedLoopController m_rightArmController;
  private final SparkClosedLoopController m_leftArmController;

  public static IntakeState currentIntakeState = IntakeState.STOWED;

  static {
    // Intake Config
    m_intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).closedLoop.pid(0, 0, 0);

    // Arm config
    m_rightArmConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .inverted(true);

    m_leftArmConfig.apply(m_rightArmConfig);

    // Right Arm Config
    m_rightArmConfig
        .closedLoop
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
  public Intake() {
    m_intake = new SparkFlex(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);
    m_leftArm = new SparkMax(CanIdsOtherThanDrive.kLeftArmId, SparkMax.MotorType.kBrushless);
    m_rightArm = new SparkMax(CanIdsOtherThanDrive.kRightArmId, SparkMax.MotorType.kBrushless);

    m_intake.configure(
        m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftArm.configure(
        m_leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightArm.configure(
        m_rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_rightArmController = m_rightArm.getClosedLoopController();
    m_leftArmController = m_leftArm.getClosedLoopController();

    m_rightArmEncoder = m_rightArm.getEncoder();
    m_leftArmEncoder = m_leftArm.getEncoder();
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

  private void setIntakeSpeed(double speed) {
    m_intake.set(speed);
    Logger.recordOutput("Intake/IntakeSpeed", speed);
  }

  private void stopIntake() {
    setIntakeSpeed(0);
    Logger.recordOutput("Intake/IntakeSpeed", 0);
  }

  private void setLeftArmSpeed(double speed) {
    double outSpeed = 0.0;
    Logger.recordOutput("Intake/ArmSpeed", speed);
    if ((getLeftArmAngle() > IntakeConstants.kArmMinAngle)
        && (getLeftArmAngle() < IntakeConstants.kArmMaxAngle)) {
      outSpeed = speed;
    } else {
      outSpeed = 0.0;
    }
    m_leftArm.set(outSpeed);
  }

  private void setRightArmSpeed(double speed) {
    double outSpeed = 0.0;
    Logger.recordOutput("Intake/ArmSpeed", speed);
    if ((getRightArmAngle() > IntakeConstants.kArmMinAngle)
        && (getRightArmAngle() < IntakeConstants.kArmMaxAngle)) {
      outSpeed = speed;
    } else {
      outSpeed = 0.0;
    }
    m_rightArm.set(outSpeed);
  }

  private void setArmSpeeds(double speed) {
    setLeftArmSpeed(speed);
    setRightArmSpeed(speed);
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

  private void setArmSetpointAndIntake(double armSetpoint, double intakeSpeed) {
    setArmSetpoint(armSetpoint);
    setIntakeSpeed(intakeSpeed);
  }

  @AutoLogOutput(key = "Intake/isArmAtSetPoint")
  public boolean isArmsAtSetpoint() {
    return m_leftArmController.isAtSetpoint() && m_rightArmController.isAtSetpoint();
  }

  public Command runIntake(double speed) {
    return this.runEnd(() -> setIntakeSpeed(speed), () -> stopIntake());
  }

  public Command runArmToAngleAndIntake(double armPosZeroToOne, double intakeSpeed) {
    return Commands.runOnce(() -> setArmSetpointAndIntake(armPosZeroToOne, intakeSpeed), this)
        .andThen(Commands.waitUntil(this::isArmsAtSetpoint))
        // .alongWith(
        //     Commands.waitSeconds(0.25)
        //         .andThen(
        //             runOnce(
        //                 () -> {
        //                   m_armPose = armPosZeroToOne;
        //                 }))))
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            });
  }

  public Command thrustingCommand() {
    return this.runArmToAngleAndIntake(
            IntakeConstants.kArmThrustInwardPosition, IntakeConstants.kStopSpeed)
        .andThen(
            this.runArmToAngleAndIntake(
                IntakeConstants.kArmThrustOutwardPosition, IntakeConstants.kStopSpeed))
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.THRUSTING);
  }

  public Command intakingCommand(double speed) {
    return this.runArmToAngleAndIntake(IntakeConstants.kArmIntakePosition, speed)
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.INTAKING);
  }

  public Command stowingCommand() {
    return this.runArmToAngleAndIntake(
            IntakeConstants.kArmStowPosition, IntakeConstants.kIntakeSpeed)
        .finallyDo(
            () -> {
              stopArm();
              stopIntake();
            })
        .beforeStarting(() -> currentIntakeState = IntakeState.STOWED);
  }

  public Command runArmToIntakeState(IntakeState intakeState) {
    switch (intakeState) {
      case INTAKING:
        return intakingCommand(IntakeConstants.kIntakeSpeed);
      case OUTTAKING:
        return intakingCommand(IntakeConstants.kOuttakeSpeed);
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
