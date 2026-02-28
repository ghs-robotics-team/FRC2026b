// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeedRoller;

/**
 * Command to run the feed roller only.
 */
public class FeedRollOnly extends Command {
  FeedRoller feedRoller;
  double power;

  /**
   * Creates a new FeedRollOnly command.
   * @param feedRoller The feed roller subsystem to run.
   * @param power The power level to set the feed roller motor to, typically between -1.0 and 1.0.
   */
  public FeedRollOnly(FeedRoller feedRoller, double power) {
    this.feedRoller = feedRoller;
    this.power = power;
    addRequirements(feedRoller);
  }

  /**
   * Stop the feed roller motor.
   */
  @Override
  public void initialize() {
    feedRoller.roll(0);
  }

  /**
   * Runs the feed roller at the specified power level.
   */
  @Override
  public void execute() {
    feedRoller.roll(power);
  }

  /**
   * Stop the feed roller motor.
   */
  @Override
  public void end(boolean interrupted) {
    feedRoller.roll(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
