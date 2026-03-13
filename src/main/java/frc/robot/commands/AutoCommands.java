// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.BeltNado;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.Kickdexer;

/** Add your docs here. */
public class AutoCommands {

  public Command feedShooter(Kickdexer kickdexer, BeltNado beltNado) {
    return new ParallelCommandGroup(kickdexer.runKickdexerForward(), beltNado.runMotorCommand(0.75))
        .withTimeout(10);
  }

  public Command shoot(FlywheelShooter shooter) {
    return shooter
        .runAtRPMSCommand(3000)
        .beforeStarting(() -> System.out.println("shooting"))
        .finallyDo(() -> System.out.println("shooting finished"))
        .withTimeout(10);
  }

  public Command windUpShooter(FlywheelShooter shooter) {
    return shooter.runAtRPMSCommand(3000).until(() -> shooter.isAtSetpoint());
  }
}
