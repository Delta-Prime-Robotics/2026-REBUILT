// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LemonFeeder extends SubsystemBase {
  private final LemonFeederIO io;
  private final LemonFeederIOInputsAutoLogged inputs = new LemonFeederIOInputsAutoLogged();

  public LemonFeeder(LemonFeederIO io) {
    this.io = io;
  }

  public void setFeederMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    io.setMotor(clampedSpeed);
    Logger.recordOutput("Shooter/LemonFeeder/RequestedSpeed", clampedSpeed);
  }

  public void stopFeeder() {
    io.stop();
    Logger.recordOutput("Shooter/LemonFeeder/RequestedSpeed", 0.0);
  }

  public Command runFeederCommand(double speed) {
    return this.runEnd(() -> setFeederMotor(speed), this::stopFeeder);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/LemonFeeder", inputs);
  }
}
