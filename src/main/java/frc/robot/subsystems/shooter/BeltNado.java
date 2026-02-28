// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

public class BeltNado extends SubsystemBase {
  /** Creates a new BeltNado. */
  private SparkMax m_Motor;
  private static SparkMaxConfig m_MotorConfig= new SparkMaxConfig();
  static{
  m_MotorConfig
  .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
  .idleMode(IdleMode.kCoast);
}

public BeltNado() {
  m_Motor = new SparkMax(Constants.CanIdsOtherThanDrive.kBeltnadoMotorId, MotorType.kBrushless);
  m_Motor.configure(m_MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

private void setMotorSpeed(double speed) {
  m_Motor.set(speed);
}

private void stopMotor() {
  m_Motor.set(0);
}

@Override
public void periodic() {
    // This method will be called once per scheduler run
  }
}
