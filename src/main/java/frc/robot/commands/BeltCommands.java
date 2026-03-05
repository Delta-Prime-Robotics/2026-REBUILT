package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
public class BeltCommands {

    public static Command runOnce(Runnable action, Subsystem... requirements) {
        return new InstantCommand(action, requirements);
    }

    public Command StartMotorcommand(){
        return null;
    }

    public Command StopMotor(){
        return this.runOnce ( () -> {StopMotor();});//added command for belt 
    }
    
} 

