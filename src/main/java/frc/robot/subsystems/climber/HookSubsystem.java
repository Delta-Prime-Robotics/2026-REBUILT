// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.NeoMotorConstants;

public class HookSubsystem extends SubsystemBase {

  //Setting arms and Encoder
  private final SparkMax m_motor; // climber motor, id = 18

  private static SparkMaxConfig m_motorConfig = new SparkMaxConfig();

  /** Creates a new ArmSubsystem. */
  public HookSubsystem() {
    // kHookCanId defined as 18 in subsystems/Constants.java
    m_motor = new SparkMax(HookConstants.kHookCanId, MotorType.kBrushless);
    
    //set smartCurrentLimits
    m_motorConfig.smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit).idleMode(IdleMode.kBrake);
  }

  private void motorRun(double speed) {
    m_motor.set(speed);
  }

  /**
   * @param direction true = forwards false = backwards
   * @return runs Hook
   */
  public Command hookRunCommand(boolean direction) {
    double speed = 0.75;
    return this.startEnd(
      direction  
      ? () -> motorRun(speed) 
      : () -> motorRun(-speed),
      ()-> m_motor.stopMotor()
    );
  }
  
  public void runHook(DoubleSupplier hookStick, boolean reverse) {
    double motorHook = hookStick.getAsDouble();
    if(!reverse){
      motorHook *= -1;
    }
    motorRun(motorHook);
  }
  

  
  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }
}