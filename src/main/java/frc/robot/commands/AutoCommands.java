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
  private final Kickdexer m_kickdexer;
  private final BeltNado m_beltNado;
  private final FlywheelShooter m_shooter;

  public AutoCommands(Kickdexer kickdexer, BeltNado beltNado, FlywheelShooter shooter) {
    this.m_kickdexer = kickdexer;
    this.m_beltNado = beltNado;
    this.m_shooter = shooter;
  }

  public Command feedShooter() {
    return new ParallelCommandGroup(
            m_kickdexer.runKickdexerForward(), m_beltNado.runMotorCommand(0.75))
        .withTimeout(10);
  }

  public Command shoot() {
    return m_shooter
        .runAtRPMSCommand(3000)
        .beforeStarting(() -> System.out.println("shooting"))
        .finallyDo(() -> System.out.println("shooting finished"))
        .withTimeout(10);
  }

  public Command windUpShooter() {
    return m_shooter.runAtRPMSCommand(3000).until(() -> m_shooter.isAtSetpoint());
  }
}
