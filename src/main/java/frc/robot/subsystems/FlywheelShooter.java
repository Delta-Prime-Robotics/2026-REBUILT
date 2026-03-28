// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CanIdsOtherThanDrive;
import frc.robot.constants.Constants.MotorConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class FlywheelShooter extends SubsystemBase {
  private final SparkFlex m_shooterLeader; // Vortex
  // private final SparkMax m_shooterFollower; // NEO
  private final RelativeEncoder m_Encoder; // Primary Encoder Leader
  private final SparkClosedLoopController m_ClosedLoopController;

  private static SparkFlexConfig m_leaderConfig = new SparkFlexConfig();
  // private static SparkMaxConfig m_followerConfig = new SparkMaxConfig();

  private static final double kShooterMaxRPM =
      Constants.MotorConstants.kVortexFreeSpeedRpm; // NEO free speed RPM
  private static final double kShooterAllowableErrorRPM = 1; // RPM //To-Do: tune this value

  // Distance in meters -> shooter RPM. Tune these points from on-field data.
  private static final InterpolatingDoubleTreeMap kShooterDistanceToRpmMap =
      new InterpolatingDoubleTreeMap();

  private static final double kMinCalibratedDistanceMeters = 1.2;
  private static final double kMaxCalibratedDistanceMeters = 6.0;
  private static double smartdashboardSetpoint = 0.0;

  // static configuration block
  static {
    kShooterDistanceToRpmMap.put(1.8, 2900.0);
    kShooterDistanceToRpmMap.put(2.1, 3050.0);
    kShooterDistanceToRpmMap.put(2.5, 3100.0);
    kShooterDistanceToRpmMap.put(3.2, 3200.0);
    kShooterDistanceToRpmMap.put(3.6, 3400.0);
    kShooterDistanceToRpmMap.put(4.5, 3700.0);
    kShooterDistanceToRpmMap.put(6.0, 4000.0);

    // configure Shooter Leader
    m_leaderConfig
        .smartCurrentLimit(MotorConstants.kVortexSmartCurrentLimit)
        .idleMode(IdleMode.kCoast)
        .inverted(true);

    // configure Spark Closed Loop Feedforward and PID... To-do: tune these values
    m_leaderConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.000001, 0.0, 0.00001)
        .allowedClosedLoopError(kShooterAllowableErrorRPM, ClosedLoopSlot.kSlot0);
    // .maxMotion
    // .maxAcceleration(0.0, ClosedLoopSlot.kSlot0); // rpm per second

    m_leaderConfig.closedLoop.feedForward.sva(
        0.34, // ks(volts)
        0.00166, // kv(volts per motor rpm)
        0.0 // ka(volts per motor rpm squared)
        );

    // configure Shooter Follower
    // m_followerConfig.apply(m_leaderConfig).follow(CanIdsOtherThanDrive.kShooterLeaderId, false);
  }

  /** Creates a new Shooter. */
  public FlywheelShooter() {
    m_shooterLeader =
        new SparkFlex(CanIdsOtherThanDrive.kShooterLeaderMotorId, SparkMax.MotorType.kBrushless);
    // m_shooterFollower =
    //     new SparkMax(CanIdsOtherThanDrive.kShooterFollowerId, SparkMax.MotorType.kBrushless);

    m_Encoder = m_shooterLeader.getEncoder(); // or getAlternateEncoder()
    m_Encoder.setPosition(0);

    // To-Do: make sure this async doesnt cause issues
    m_shooterLeader.configureAsync(
        m_leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // m_shooterFollower.configureAsync(
    //     m_followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Might not need this but it would be intresting to mess with this
    // Could we get faster response times with less retries and timeouts?
    // and could we up the number of retries while using the PID controller?
    // m_shooterLeader.setCANMaxRetries(5);
    // m_shooterFollower.setCANMaxRetries(5);
    // m_shooterLeader.setCANTimeout(10);
    // m_shooterFollower.setCANTimeout(10);

    m_ClosedLoopController = m_shooterLeader.getClosedLoopController();
    SmartDashboard.putNumber("ShooterSetpoint", 0);
  }

  /**
   * Gets the current shooter wheel velocity in RPM
   *
   * @return The current shooter wheel velocity in RPM
   */
  @AutoLogOutput(key = "Shooter/VelocityRPM")
  public double getShooterVelocity() {
    return m_Encoder.getVelocity();
  }

  private void setShooterSetpoint(double setpointRPM) {
    if (setpointRPM > kShooterMaxRPM) {
      setpointRPM = kShooterMaxRPM;
      System.out.println(
          "WARNING: Shooter setpoint RPM exceeds maximum. Capping to " + kShooterMaxRPM + " RPM.");
    }
    m_ClosedLoopController.setSetpoint(setpointRPM, ControlType.kVelocity);
    Logger.recordOutput("Shooter/Setpoint RPM", setpointRPM);
  }

  private void stopShooter() {
    m_ClosedLoopController.setSetpoint(0.0, ControlType.kVelocity);
    Logger.recordOutput("Shooter/Setpoint RPM", 0.0);
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean isAtSetpoint() {
    return m_ClosedLoopController.isAtSetpoint();
  }

  /**
   * Immediately stops the shooter motors
   *
   * @waring This method should only be used in emergency situations
   */
  public void hardStopShooter() {
    m_shooterLeader.stopMotor();
    // m_shooterFollower.stopMotor();
    Logger.recordOutput("Shooter/Setpoint RPM", 0.0);
  }

  /** Winds up the shooter to an average setpoint RPM */
  public Command windUpShooterCommand() {
    // Maybe start runing shooter at low speed when on allince side of the field
    // to decrease spinup time when trying to shoot
    final double shooterAvgSetpointRPM = 3000; // Example average shooter setpoint
    return runAtRPMSCommand(shooterAvgSetpointRPM)
        .until(() -> isAtSetpoint())
        .withTimeout(1); // This should only run if the command is interrupted
  }

  public Command idleShooterCommand() {
    return runAtRPMSCommand(1000);
  }

  public Command stopShooterCommand() {
    return this.runOnce(() -> stopShooter());
  }

  /**
   * Shoots at a specific RPM A Run Command
   *
   * @param rpm The target RPM to shoot at
   */
  public Command runAtRPMSCommand(double rpm) {
    // System.out.println("Running shooter at " + rpm + " RPM");
    return this.runEnd(() -> setShooterSetpoint(rpm), this::stopShooter);
  }

  private double getShooterRPMForDistanceMeters(double distanceMeters) {
    double clampedDistanceMeters =
        MathUtil.clamp(distanceMeters, kMinCalibratedDistanceMeters, kMaxCalibratedDistanceMeters);
    double targetRpm = kShooterDistanceToRpmMap.get(clampedDistanceMeters);

    // Logger.recordOutput("Shooter/TargetDistanceMeters", distanceMeters);
    Logger.recordOutput("Shooter/ClampedDistanceMeters", clampedDistanceMeters);

    return targetRpm;
  }

  public Command autoShootRange() {
    return this.runEnd(
        () -> {
          double targetDistanceMeters = Drive.getDistanceToHubMeters();
          double targetRPM = getShooterRPMForDistanceMeters(targetDistanceMeters);
          setShooterSetpoint(targetRPM);
        },
        () -> stopShooter());
  }

  public Command runAtSpeedCommand(double speed) {
    return this.runEnd(() -> m_shooterLeader.set(speed), () -> stopShooter());
  }

  // @Override
  // public void periodic() {

  //   if (smartdashboardSetpoint != SmartDashboard.getNumber("ShooterSetpoint", 0)) {
  //     smartdashboardSetpoint = SmartDashboard.getNumber("ShooterSetpoint", 0);
  //     System.out.println("Shooter Setpoint" + smartdashboardSetpoint);
  //     runAtRPMSCommand(smartdashboardSetpoint).withTimeout(10);
  //   }

  //   // This method will be called once per scheduler run
  // }
}
