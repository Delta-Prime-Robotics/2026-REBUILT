// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIdsOtherThanDrive;

public class Indexer extends SubsystemBase {

  private final SparkMax m_indexer; //NEO Motor

  /** Creates a new Indexer. */
  public Indexer() {
    m_indexer = new SparkMax(CanIdsOtherThanDrive.kIndexerId, SparkMax.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
