// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * The Haptics subsystem controls the rumble functionality of a game controller.
 * It provides methods to set the rumble on the left, right, or both sides of
 * the controller.
 */
public class Haptics extends SubsystemBase {
  private GenericHID m_controller;
  private RumbleType kHeavy = RumbleType.kLeftRumble;
  private RumbleType kMedium = RumbleType.kBothRumble;
  private RumbleType kLight = RumbleType.kRightRumble;

  /**
   * Creates a new Haptics subsystem.
   * 
   * @param m_gamepad The XboxController to be used for haptic feedback.
   */
  public Haptics(CommandXboxController m_gamepad) {
    m_controller = new GenericHID(m_gamepad.getHID().getPort());
  }

  /**
   * Sets the rumble on the controller.
   * 
   * @param buzz The intensity of the rumble (0.0 to 1.0).
   * @param type The type of buzz, Heavy, Medium, or Light
   */
  private void setBuzz(double buzz, RumbleType type) {
    MathUtil.clamp(buzz, 0, 1);
    m_controller.setRumble(type, buzz);
  }

  private void setBuzzVarBoth(double lightBuzz, double heavyBuzz) {
    setBuzz(heavyBuzz, kHeavy);
    setBuzz(lightBuzz, kLight);
  }

  /**
   * Stops the rumble on the specified side of the controller.
   * 
   * @param where The side of the controller to stop rumbling.
   */
  private void stopBuzzing(RumbleType where) {
    m_controller.setRumble(where, 0);
  }

  public Command buzzLeft(double buzz) {
    return runEnd(
        () -> setBuzz(buzz, kHeavy),
        () -> stopBuzzing(kHeavy));
  }

  public Command buzzRight(double buzz) {
    return runEnd(
        () -> setBuzz(buzz, kLight),
        () -> stopBuzzing(kLight));
  }

  public Command buzzBoth(double buzz) {
    return startEnd(
        () -> setBuzz(buzz, kMedium),
        () -> stopBuzzing(kMedium));
  }
  
  public Command buzzFor(double buzz, RumbleType type, double time) {
    return runEnd(
        () -> m_controller.setRumble(type, buzz),
        () -> stopBuzzing(type)).withTimeout(time);
  }

  /**
   * Runs the light and heavy buzzes at diffrent speeds for a set amount of time.
   * @param lightBuzz
   * @param heavyBuzz
   * @param seconds
   */
  public Command buzzVarBothFor(double lightBuzz, double heavyBuzz, double seconds) {
    return runEnd(
      ()-> setBuzzVarBoth(lightBuzz, heavyBuzz),
      ()-> stopBuzzing(kMedium)).withTimeout(seconds);
  }

  /**
   * Turns a buzz on and off for a durtaion of time. 
   * Time inbetween buzzes is the buzzTime / 3
   * @param buzz buzz stregnth
   * @param type type of buzz ((left, heavy), (middle,medium), (right, light))
   * @param buzzTime duration of buzz 
   * @param totalTime total duration of pattern
   */
  public Command onOffBuzzRepeat(double buzz, RumbleType type, double buzzTime, double totalTime) {
    double less = buzzTime / 3;
    return new RepeatCommand(
        buzzFor(buzz, type, buzzTime)
        .andThen(Commands.waitSeconds(buzzTime - less))
        )
        .withTimeout(totalTime)
        .finallyDo(() -> stopBuzzing(type));
  }
  
  /**
   * Turns the heavy and light buzz on and off for a durtaion of time. 
   * Time in between buzzes is the buzzTime / 3
   * @param lightBuzz buzz stregnth for the light buzz
   * @param heavyBuzz buzz stregth for the heavy buzz
   * @param buzzTime duration of both buzzess
   * @param totalTime total duration of pattern
   */
  public Command onOffVarBuzzRepeat(double lightbuzz, double heavyBuzz, double buzzTime, double totalTime) {
    double less = buzzTime / 3;
    return new RepeatCommand(buzzVarBothFor(lightbuzz,heavyBuzz, buzzTime))
    .andThen(Commands.waitSeconds(buzzTime - less))
    .withTimeout(totalTime)
    .finallyDo(() -> stopBuzzing(kMedium));
  }

  /**
   * Alternates between the light and heavy buzz for a duration of time. 
   * Time in between the diffrent buzzes is the buzzTime / 3
   * @param lightBuzz buzz stregnth for the light buzz
   * @param heavyBuzz buzz stregth for the heavy buzz
   * @param buzzTime duration of both buzzess
   * @param totalTime total duration of pattern
   */
  public Command alternatingBuzzRepeat(double lightbuzz, double heavyBuzz, double buzzTime, double totalTime) {
    double less = buzzTime / 3;
    return new RepeatCommand(
        buzzFor(lightbuzz, kLight, buzzTime)
        .andThen(Commands.waitSeconds(less))
        .andThen(buzzFor(heavyBuzz, kHeavy, buzzTime)))
        .andThen(Commands.waitSeconds(less)
        )
        .withTimeout(totalTime)
        .finallyDo(() -> stopBuzzing(kMedium));
  }

  public Command shiftChangeIn5Command() {
    return onOffBuzzRepeat(1, kMedium, (3/4), 5);
  }

}
