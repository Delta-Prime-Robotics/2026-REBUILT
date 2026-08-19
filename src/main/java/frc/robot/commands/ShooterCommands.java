// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ShooterCommands {
  public static Command shootAtHubWhileDriving(
      Drive drive, FlywheelShooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return new ParallelCommandGroup(
        shooter.autoShootRange(), DriveCommands.aimAtHubWhileDriving(drive, xSupplier, ySupplier));
  }
}
