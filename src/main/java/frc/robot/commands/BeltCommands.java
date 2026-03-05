package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.*;

public class BeltCommands {

  public static Command runOnce(Runnable action, Subsystem... requirements) {
    return new InstantCommand(action, requirements);
  }

  public Command StartMotorcommand() {
    return null;
  }

  public Command StopMotor() {
    return this.runOnce(
        () -> {
          StopMotor();
        }); // added command for belt
  }
}
