// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.FlywheelShooter;
import frc.robot.subsystems.drive.Drive;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterCommands {
  public static Command shootAtHubWhileDriving(
      Drive drive, FlywheelShooter shooter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    AtomicReference<DoubleSupplier> distanceToHubMeters = new AtomicReference<>(() -> 0.0);

    Supplier<Pose2d> hubPose =
        () -> {
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          return isFlipped
              // Red Hub center field coords
              ? new Pose2d(
                  Units.inchesToMeters(468.56), Units.inchesToMeters(158.32), Rotation2d.kZero)
              // Blue Hub center field coords
              : new Pose2d(
                  Units.inchesToMeters(181.56), Units.inchesToMeters(158.32), Rotation2d.kZero);
        };

    return new ParallelCommandGroup(
            shooter.autoShootRange(distanceToHubMeters.get()),
            DriveCommands.aimAtHubWhileDriving(drive, xSupplier, ySupplier, hubPose))
        .beforeStarting(
            () -> {
              distanceToHubMeters.set(
                  () -> {
                    Pose2d target = hubPose.get();
                    if (target == null) {
                      return 0.0;
                    }
                    Pose2d robotPose = drive.getPose();
                    double distance =
                        target.getTranslation().getDistance(robotPose.getTranslation());
                    return distance;
                  });
              Logger.recordOutput("Shooter/Distance to Hub",+ distanceToHubMeters.get().getAsDouble());
            });
  }
}
