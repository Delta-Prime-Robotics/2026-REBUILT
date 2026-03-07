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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanIdsOtherThanDrive;
import frc.robot.constants.Constants.MotorConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kickdexer extends SubsystemBase {
  private static SparkMax m_topMotor; // NEO
  private static SparkMax m_bottomMotor; // NEO
  private static RelativeEncoder m_topEncoder;
  private static RelativeEncoder m_bottomEncoder;

  private static SparkClosedLoopController m_topClosedLoopController;
  private static SparkClosedLoopController m_bottomClosedLoopController;

  private static SparkMaxConfig m_topMotorConfig = new SparkMaxConfig();
  private static SparkMaxConfig m_bottomMotorConfig = new SparkMaxConfig();

  static {
    m_topMotorConfig
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .encoder
        .velocityConversionFactor(1 / 4.0); // gear ratio of 4:1

    m_bottomMotorConfig.apply(m_topMotorConfig).inverted(true);

    // Closed Loop Top Motor
    m_topMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .feedForward
        .sv(0, 0);

    // Closed Loop Bottom Motor
    m_bottomMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .feedForward
        .sv(0, 0);
  }

  /** Creates a new Kickdexer. */
  public Kickdexer() {
    m_topMotor = new SparkMax(CanIdsOtherThanDrive.kKickdexerTopMotorId, MotorType.kBrushless);
    m_bottomMotor =
        new SparkMax(CanIdsOtherThanDrive.kKickdexerBottomMotorId, MotorType.kBrushless);

    m_topMotor.configureAsync(
        m_topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomMotor.configureAsync(
        m_bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_topEncoder = m_topMotor.getEncoder();
    m_bottomEncoder = m_bottomMotor.getEncoder();

    m_topClosedLoopController = m_topMotor.getClosedLoopController();
    m_bottomClosedLoopController = m_bottomMotor.getClosedLoopController();
  }

  @AutoLogOutput(key = "Kickdexer/TopVelocityRPM")
  public double getTopEncoderVelocityRPM() {
    return m_topEncoder.getVelocity();
  }

  @AutoLogOutput(key = "Kickdexer/BottomVelocityRPM")
  public double getBottomEncoderVelocityRPM() {
    return m_bottomEncoder.getVelocity();
  }

  private void setMotorSpeeds(double topMotorSpeed, double bottomMotorSpeed) {
    // topMotorSpeed = MathUtil.clamp(topMotorSpeed, -1.0, 1.0);
    // bottomMotorSpeed = MathUtil.clamp(bottomMotorSpeed, -1.0, 1.0);
    m_topMotor.set(topMotorSpeed);
    m_bottomMotor.set(bottomMotorSpeed);

    Logger.recordOutput("Kickdexer/TopPercentOutput", topMotorSpeed);
    Logger.recordOutput("Kickdexer/BottomPercentOutput", bottomMotorSpeed);
  }

  public Command setMotorSpeedsCommand(double topSpeed, double bottomSpeed) {
    return this.runOnce(() -> setMotorSpeeds(topSpeed, bottomSpeed)).finallyDo(this::stopMotors);
  }

  private void setVelocitySetpoints(double topSetpointRPM, double bottomSetpointRPM) {
    m_topClosedLoopController.setSetpoint(
        topSetpointRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_bottomClosedLoopController.setSetpoint(
        bottomSetpointRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    Logger.recordOutput("Kickdexer/TopSetpointRPM", topSetpointRPM);
    Logger.recordOutput("Kickdexer/BottomSetpointRPM", bottomSetpointRPM);
  }

  private void stopMotors() {
    m_topClosedLoopController.setSetpoint(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    m_bottomClosedLoopController.setSetpoint(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

    Logger.recordOutput("Kickdexer/TopSetpointRPM", 0.0);
    Logger.recordOutput("Kickdexer/BottomSetpointRPM", 0.0);
  }

  public Command setVelocitySetpointsCommand(double topSetpointRPM, double bottomSetpointRPM) {
    return this.runOnce(() -> setVelocitySetpoints(topSetpointRPM, bottomSetpointRPM));
  }

  // public Command stopKickdexerCommand() {
  //   return this.runOnce(this::stopMotors);
  // }

  public Command runAtSpeedsCommand(double topMotorSpeed, double bottomMotorSpeed) {
    return this.runEnd(() -> setMotorSpeeds(topMotorSpeed, bottomMotorSpeed), () -> stopMotors());
  }

  public Command runKickdexerForward() {
    return runAtSpeedsCommand(-0.5, -0.5);
  }

  /*void setSetpoints() {

  }*/

  @Override
  public void periodic() {
    Logger.recordOutput("Kickdexer/TopVelocityRPM", getTopEncoderVelocityRPM());
    Logger.recordOutput("Kickdexer/BottomVelocityRPM", getBottomEncoderVelocityRPM());
  }
}
