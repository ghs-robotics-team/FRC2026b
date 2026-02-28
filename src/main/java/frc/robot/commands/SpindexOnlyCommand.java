// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;

/**
 * Runs the spindexer only.
 */
public class SpindexOnlyCommand extends Command {
  Spindexer spindexer;
  double power;

  /**
   * Creates a new SpindexOnlyCommand.
   * @param spindexer The spindexer subsystem to run.
   * @param power The power level to set the spindexer motor to, typically between -1.0 and 1.0.
   */
  public SpindexOnlyCommand(Spindexer spindexer, double power) {
    this.spindexer = spindexer;
    this.power = power;
    addRequirements(spindexer);
  }

  /**
   * Stop the spindexer motors.
   */
  @Override
  public void initialize() {
    spindexer.run(0);
  }

  /**
   * Runs the spindexer at the specified power level.
   */
  @Override
  public void execute() {
    spindexer.run(power);
  }

  /**
   * Stop the spindexer motors.
   */
  @Override
  public void end(boolean interrupted) {
    spindexer.run(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
