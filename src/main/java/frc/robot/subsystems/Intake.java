// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanIdsOtherThanDrive;

public class Intake extends SubsystemBase {
  private final SparkFlex m_intake; // Vortex
  private static SparkFlexConfig m_intakeConfig = new SparkFlexConfig();

  static {
    // Intake Config
    m_intakeConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60).closedLoop.pid(0, 0, 0);
  }
  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkFlex(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);
     m_intake.configure(
        m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setIntakeSpeed(double speed) {
    m_intake.set(speed);
    Logger.recordOutput("Intake/IntakeSpeed", speed);
  }

  public void stopIntake() {
    setIntakeSpeed(0);
    Logger.recordOutput("Intake/IntakeSpeed", 0);
  }

  public Command stopIntakeCommand() {
    return Commands.runOnce(()-> stopIntake());
  }

  public Command runIntakeAtSpeedCommand(double speed) {
    return this.runEnd(() -> setIntakeSpeed(speed), () -> stopIntake());
  }

  public Command runIntakeWithoutStopingCommand(double speed) {
    return this.run(() -> setIntakeSpeed(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
