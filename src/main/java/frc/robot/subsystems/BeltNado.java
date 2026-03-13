// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.MotorConstants;

public class BeltNado extends SubsystemBase {
  /** Creates a new BeltNado. */
  private SparkMax m_Motor; // NEO 550

  private static SparkMaxConfig m_MotorConfig = new SparkMaxConfig();

  static {
    m_MotorConfig
        .smartCurrentLimit(MotorConstants.kNeo550SmartCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(true);
  }

  public BeltNado() {
    m_Motor = new SparkMax(Constants.CanIdsOtherThanDrive.kBeltnadoMotorId, MotorType.kBrushless);
    m_Motor.configure(
        m_MotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void setMotorSpeed(double speed) {
    m_Motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  private void stopMotor() {
    m_Motor.set(0);
  }

  public Command runMotorCommand(double speed) {
    return this.runEnd(() -> setMotorSpeed(speed), () -> stopMotor());
  }

  public Command stopMotorCommand() {
    return this.runOnce(this::stopMotor);
  }

  // public Command sinWaveMotorCommand(double frequencyHz) {
  //   return this.runEnd(
  //       () -> {
  //         double waveOutput = (Math.sin(2.0 * Math.PI * frequencyHz * Timer.getFPGATimestamp()));
  //         setMotorSpeed(waveOutput);
  //       },
  //       this::stopMotor);
  // }

  public Command shimmyEhShimmyAhCommand() {
    return new RepeatCommand(
            runMotorCommand(1).withTimeout(1).andThen(runMotorCommand(-1)).withTimeout(1))
        .finallyDo(() -> stopMotor());
  }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}
