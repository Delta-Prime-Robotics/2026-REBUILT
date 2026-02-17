// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdsOtherThanDrive;
import frc.robot.Constants.MotorConstants;

public class Kickdexer extends SubsystemBase {
  private static SparkMax m_topMotor; //NEO
  private static SparkMax m_bottomMotor; //NEO
  private static RelativeEncoder m_TopEncoder;
  private static RelativeEncoder m_BottomEncoder;
  private static SparkClosedLoopController m_TopClosedLoopController;
  private static SparkClosedLoopController m_BottomClosedLoopController;

  private static SparkMaxConfig m_topMotorConfig = new SparkMaxConfig();
  private static SparkMaxConfig m_bottomMotorConfig = new SparkMaxConfig();


  static {
    m_topMotorConfig
        .smartCurrentLimit(MotorConstants.kNeoSmartCurrentLimit)
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .encoder
        .velocityConversionFactor(1/4); // gear ratio of 4:1, so we need to divide by 4 to get the output velocity
      
    m_bottomMotorConfig
        .apply(m_topMotorConfig)
        .inverted(true);
    
    //Closed Loop Top Motor
    m_topMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .feedForward
        .sv(0,0);
    
    //Closed Loop Bottom Motor
    m_bottomMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0, 0, 0)
        .feedForward
        .sv(0,0);
    
  }

  /** Creates a new Kickdexer. */
  public Kickdexer() {
    m_topMotor = new SparkMax(CanIdsOtherThanDrive.kKickdexerTopMotorId, MotorType.kBrushless);
    m_bottomMotor = new SparkMax(CanIdsOtherThanDrive.kKickdexerBottomMotorId, MotorType.kBrushless);

    m_topMotor.configureAsync(m_topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomMotor.configureAsync(m_bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_TopEncoder = m_topMotor.getEncoder();
    m_BottomEncoder = m_bottomMotor.getEncoder();

    m_TopClosedLoopController = m_topMotor.getClosedLoopController();
    m_BottomClosedLoopController = m_bottomMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
