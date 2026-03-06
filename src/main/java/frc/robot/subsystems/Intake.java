// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdsOtherThanDrive;
import frc.robot.Constants.MotorConstants;
  

//We want it take the fuel from the floor into the robot. We might use a laser break sensor and we will need sparkmax motor controllers. And put encoder on the controllers.


public class Intake extends SubsystemBase {
  private final SparkMax m_intake; // NEO 550
 
  private final SparkMax m_arm; // NEO make a pid for position that only allows the arm to go down 90?

  private static SparkMaxConfig m_intakeConfig = new SparkMaxConfig();
  
  private static SparkMaxConfig m_armConfig = new SparkMaxConfig();

  private final SparkClosedLoopController m_armController;
 
  static {
    m_intakeConfig.idleMode(IdleMode.kBrake);
    m_intakeConfig.smartCurrentLimit(MotorConstants.kNeo550SmartCurrentLimit); 
   
    m_armConfig.idleMode(IdleMode.kBrake);
    m_armConfig.smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit);

    m_armConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0, 0.0, 0.0)
        .allowedClosedLoopError(0, ClosedLoopSlot.kSlot0)
        .maxMotion
        .maxAcceleration(0.0, ClosedLoopSlot.kSlot0); // rpm per second

    m_armConfig.closedLoop.feedForward.svacr(0, 0, 0, 0, 0);

  }

  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkMax(CanIdsOtherThanDrive.kIntakeId, SparkMax.MotorType.kBrushless);

    m_intake.configure(m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    m_arm = new SparkMax(CanIdsOtherThanDrive.kArmId, SparkMax.MotorType.kBrushless);
   
    m_arm.configure(m_armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_armController = m_arm.getClosedLoopController();

  }

  private void setIntakeSpeed(double speed){
    m_intake.set(speed);

  }
   
  private void setArmSpeed(double speed){ 
    m_arm.set(speed);
  
  }
    // add speed 

  private void setArmSetpoint(double setpointDeg) {
    setpointDeg = MathUtil.clamp(setpointDeg, 90, 0);
    m_armController.setSetpoint(setpointDeg, ControlType.kMAXMotionPositionControl);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
