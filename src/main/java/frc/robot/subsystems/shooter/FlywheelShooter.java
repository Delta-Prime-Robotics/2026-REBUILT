// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import org.littletonrobotics.junction.Logger;

public class FlywheelShooter extends SubsystemBase {
  public enum ShooterState {
    DISABLED,
    STOPPED,
    IDLE,
    SHOOTING
  }

  private static final double kIdleSetpointRPM = 1000.0; // TODO: tune
  private static final double kDefaultShootingSetpointRpm = 4000.0; // TODO: tune

  private final FlywheelShooterIO io;
  private final FlywheelShooterIOInputsAutoLogged inputs =
      new FlywheelShooterIOInputsAutoLogged();

  private final Command disabledDefaultCommand;
  private final Command stoppedDefaultCommand;
  private final Command idleDefaultCommand;
  private final Command shootingDefaultCommand;

  private ShooterState state = ShooterState.DISABLED;
  private double shootingSetpointRpm = kDefaultShootingSetpointRpm;

  /** Creates a new Shooter. */
  public FlywheelShooter(FlywheelShooterIO io) {
    this.io = io;

    disabledDefaultCommand = this.run(this::stopShooter);
    stoppedDefaultCommand = this.run(this::stopShooter);
    idleDefaultCommand = this.run(() -> setShooterSetpoint(kIdleSetpointRPM));
    shootingDefaultCommand = this.run(() -> setShooterSetpoint(shootingSetpointRpm));

    setState(ShooterState.DISABLED);
  }

  /**
   * Gets the current shooter wheel velocity in RPM.
   *
   * @return The current shooter wheel velocity in RPM.
   */
  public double getShooterVelocity() {
    return inputs.velocityRpm;
  }

  public ShooterState getState() {
    return state;
  }

  public void setState(ShooterState newState) {
    state = newState;
    switch (newState) {
      case DISABLED:
        setDefaultCommand(disabledDefaultCommand);
        break;
      case STOPPED:
        setDefaultCommand(stoppedDefaultCommand);
        break;
      case IDLE:
        setDefaultCommand(idleDefaultCommand);
        break;
      case SHOOTING:
        setDefaultCommand(shootingDefaultCommand);
        break;
      default:
        setDefaultCommand(disabledDefaultCommand);
        break;
    }
    Logger.recordOutput("Shooter/State", newState.toString());
  }

  public Command setStateCommand(ShooterState newState) {
    return this.runOnce(() -> setState(newState));
  }

  public void setShootingSetpointRpm(double setpointRpm) {
    shootingSetpointRpm = setpointRpm;
  }

  private void setShooterSetpoint(double setpointRPM) {
    double clampedSetpoint =
        MathUtil.clamp(setpointRPM, 0.0, MotorConstants.kNeoFreeSpeedRpm);
    if (clampedSetpoint != setpointRPM) {
      System.out.println(
          "WARNING: Shooter setpoint RPM exceeds maximum. Capping to "
              + MotorConstants.kNeoFreeSpeedRpm
              + " RPM.");
    }
    io.setVelocityRpm(clampedSetpoint);
    Logger.recordOutput("Shooter/Setpoint RPM", clampedSetpoint);
  }

  private void stopShooter() {
    io.stop();
    Logger.recordOutput("Shooter/Setpoint RPM", 0.0);
  }

  /** Immediately stops the shooter motors (use only in emergency situations). */
  public void hardStopShooter() {
    io.stop();
    Logger.recordOutput("Shooter/Setpoint RPM", 0.0);
  }

  /** Winds up the shooter to an average setpoint RPM. */
  public Command windUpShooterCommand() {
    setShootingSetpointRpm(kDefaultShootingSetpointRpm);
    return this.runOnce(() -> setState(ShooterState.SHOOTING))
        .handleInterrupt(() -> setState(ShooterState.STOPPED));
  }

  public Command stopShooterCommand() {
    return this.runOnce(() -> setState(ShooterState.STOPPED));
  }

  /**
   * Shoots at a specific RPM.
   *
   * @param rpm The target RPM to shoot at.
   */
  public Command shootAtRPMSCommand(double rpm) {
    return this.runOnce(
            () -> {
              setShootingSetpointRpm(rpm);
              setState(ShooterState.SHOOTING);
            })
        .finallyDo(() -> setState(ShooterState.STOPPED));
  }

  private double getShooterRPMForDistanceMeters(double distanceMeters) {
    return 0; // TODO: put a fancy formula or lookup table here
  }

  public Command autoShootRange(double targetDistanceMeters) {
    double targetRPM = getShooterRPMForDistanceMeters(targetDistanceMeters);
    return this.runOnce(
            () -> {
              setShootingSetpointRpm(targetRPM);
              setState(ShooterState.SHOOTING);
            })
        .finallyDo(() -> setState(ShooterState.STOPPED));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
  }
}
