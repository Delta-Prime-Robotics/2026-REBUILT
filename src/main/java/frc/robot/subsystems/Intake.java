// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanIdsOtherThanDrive;
import org.littletonrobotics.junction.AutoLogOutput;

public class Intake extends SubsystemBase {
  private final SparkFlex m_intake; // Vortex
  private static SparkFlexConfig m_intakeConfig = new SparkFlexConfig();
  private static RelativeEncoder m_encoder;
  private static SparkClosedLoopController m_ClosedLoopController;

  static {
    // Intake Config
    m_intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(60)
        .inverted(true)
        .closedLoop
        .pid(0.00009, 0, 0)
        .feedForward
        .sv(0.15, 0.00182);
  }
  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkFlex(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);
    m_intake.configure(
        m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_encoder = m_intake.getEncoder();
    m_ClosedLoopController = m_intake.getClosedLoopController();
  }

  @AutoLogOutput(key = "Intake/VelocityRPM")
  public double getIntakeVelocity() {
    return m_encoder.getVelocity();
  }

  private void setIntakeSpeed(double speed) {
    m_intake.set(speed);
    // Logger.recordOutput("Intake/IntakeSpeed", speed);
  }

  private void setIntakeToSetpoint(double RPM) {
    m_ClosedLoopController.setSetpoint(RPM, ControlType.kVelocity);
  }

  public void stopIntake() {
    m_intake.stopMotor();
    // Logger.recordOutput("Intake/IntakeSpeed", 0);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(() -> stopIntake());
  }

  public Command runIntakeAtSpeedCommand(double speed) {
    return this.runEnd(() -> setIntakeSpeed(speed), () -> stopIntake());
  }

  public Command runIntakeAtSpeedWithoutStopingCommand(double speed) {
    return this.run(() -> setIntakeSpeed(speed));
  }

  public Command runIntakeAtRPMCommand(double RPM) {
    return this.runEnd(() -> setIntakeToSetpoint(RPM), () -> setIntakeToSetpoint(0));
  }

  public Command runIntakeAtRPMCommandWithoutStopping(double RPM) {
    return this.run(() -> setIntakeToSetpoint(RPM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
