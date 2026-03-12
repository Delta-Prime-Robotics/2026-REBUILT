// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BeltNado;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Kickdexer;

/** Add your docs here. */
public class AutoCommands {

  public static Command feedShooter(Kickdexer kickdexer, BeltNado beltNado) {
    return new ParallelCommandGroup(kickdexer.runKickdexerForward(), beltNado.runMotorCommand(0.75))
        .withTimeout(10);
  }

  public static Command shoot(FlywheelShooter shooter) {
    return shooter.runAtRPMCommand(3000);
  }

  public static Command windUpShooter(FlywheelShooter shooter) {
    return shooter.runAtRPMCommand(3000).until(() -> shooter.isAtSetpoint());
  }
}
