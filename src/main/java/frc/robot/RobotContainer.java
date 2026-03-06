// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.shooter.BeltNado;
import frc.robot.subsystems.shooter.FlywheelShooter;
import frc.robot.subsystems.shooter.Kickdexer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final FlywheelShooter flywheelShooter = new FlywheelShooter();
  private final Kickdexer kickdexer = new Kickdexer();
  private final BeltNado beltNado = new BeltNado();

  // Controller
  private final CommandXboxController operatorController = new CommandXboxController(2);
  private final CommandXboxController driverController = new CommandXboxController(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private static Field2d field;
  private boolean autoWindupEnabled = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    setAutoCommands();

    // uncomment to set drive characterization commands on auto chooser
    // setDriveCharacterizationCommands();

    // Configure the button bindings
    configureButtonBindings();

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    FieldConstants.plotZones();
  }

  public static Field2d getField() {
    return field;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 45° when A button is held
    // to go over bumps
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.fromDegrees(45)));

    // Aim at hub when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.aimAtHubWhileDriving(
                drive,
                () -> -driverController.getLeftY(),
                () ->
                    -driverController
                        .getLeftX())); // to-do, maybe create constant for these field coords
    // I also saw another team that was houseing these coords in a separate file
    // that way only one boolean to flip the allience was needed for all coords

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Operator controls
    operatorController.b().whileTrue(kickdexer.runAtSpeedsCommand(0.5, 0.5));
    operatorController.a().whileTrue(kickdexer.runAtSpeedsCommand(-0.5, -0.5));

    operatorController.x().whileTrue(beltNado.runMotorCommand(0.5));

    operatorController.rightTrigger().whileTrue(flywheelShooter.runAtSpeedCommand(0.25));

    // Auto-range shooter control without coupling the shooter command to the Drive subsystem API.
    operatorController
        .leftTrigger()
        .whileTrue(flywheelShooter.autoShootRange(this::getDistanceToHubMeters));

    // Keep shooter wound up while in alliance zone. As a default command, this resumes
    // automatically after any other shooter command is interrupted or finishes.
    // flywheelShooter.setDefaultCommand(flywheelShooter.windUpShooterCommand().until(() ->
    // !isRobotInAllianceZone()));
  }

  private double getDistanceToHubMeters() {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    Translation2d hubTranslation =
        isFlipped
            ? new Translation2d(Units.inchesToMeters(468.56), Units.inchesToMeters(158.32))
            : new Translation2d(Units.inchesToMeters(181.56), Units.inchesToMeters(158.32));
    return drive.getPose().getTranslation().getDistance(hubTranslation);
  }

  public void setAutoCommands() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
  }

  public void setDriveCharacterizationCommands() {
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
