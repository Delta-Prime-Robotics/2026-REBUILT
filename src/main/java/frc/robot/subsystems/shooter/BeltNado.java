// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class BeltNado extends SubsystemBase {
  private final BeltNadoIO io;
  private final BeltNadoIOInputsAutoLogged inputs = new BeltNadoIOInputsAutoLogged();

  public BeltNado(BeltNadoIO io) {
    this.io = io;
  }

  public void setBeltMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    io.setMotor(clampedSpeed);
    Logger.recordOutput("Shooter/BeltNado/RequestedSpeed", clampedSpeed);
  }

  public void stopBelt() {
    io.stop();
    Logger.recordOutput("Shooter/BeltNado/RequestedSpeed", 0.0);
  }

  public Command runBeltCommand(double speed) {
    return this.runEnd(() -> setBeltMotor(speed), this::stopBelt);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/BeltNado", inputs);
  }
}
