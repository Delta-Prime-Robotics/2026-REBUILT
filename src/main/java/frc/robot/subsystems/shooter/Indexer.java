// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setIndexerMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    io.setMotor(clampedSpeed);
    Logger.recordOutput("Shooter/Indexer/RequestedSpeed", clampedSpeed);
  }

  public void stopIndexer() {
    io.stop();
    Logger.recordOutput("Shooter/Indexer/RequestedSpeed", 0.0);
  }

  public Command runIndexerCommand(double speed) {
    return this.runEnd(() -> setIndexerMotor(speed), this::stopIndexer);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Indexer", inputs);
  }
}
