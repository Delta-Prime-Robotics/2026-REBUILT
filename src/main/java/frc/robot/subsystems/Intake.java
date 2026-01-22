// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdsOtherThanDrive;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  private final SparkMax m_intake; //NEO 550

  private static SparkMaxConfig m_intakeConfig = new SparkMaxConfig();

  static {
    m_intakeConfig.idleMode(IdleMode.kBrake);
    m_intakeConfig.smartCurrentLimit(MotorConstants.kNeo550SmartCurrentLimit);
  }

  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkMax(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);

    m_intake.configure(m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setIntakeSpeed(double speed){
    m_intake.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
