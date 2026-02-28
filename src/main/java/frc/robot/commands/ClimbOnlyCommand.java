// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/**
 * Runs the climber only.
 */
public class ClimbOnlyCommand extends Command {
  Climber climber;
  double power;

  /**
   * Creates a new ClimbOnlyCommand.
   * @param climber The climber subsystem to run.
   * @param power The power level to set the climber motor to, typically between -
   */
  public ClimbOnlyCommand(Climber climber, double power) {
    this.climber = climber;
    this.power = power;
    addRequirements(climber);
  }

  /**
   * Stop the climber motors.
   */
  @Override
  public void initialize() {
    climber.climb(0);
  }

  /**
   * Runs the climber at the specified power level.
   */
  @Override
  public void execute() {
    climber.climb(power);
  }

  /**
   * Stop the climber motors.
   */
  @Override
  public void end(boolean interrupted) {
    climber.climb(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */ 
  @Override
  public boolean isFinished() {
    return false;
  }
}
