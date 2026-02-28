// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Puts down the intake arm, or brings it back up.
 */
public class PositionIntakeCommand extends Command {
  private Intake intake;
  private double power;

  /**
   * Creates a new PositionIntakeCommand.
   * @param intake Intake subsystem to control.
   * @param power The power level to set the intake motor to, typically between -1.0 and 1.0.
   */
  public PositionIntakeCommand(Intake intake, double power) {
    this.intake = intake;
    this.power = power;
  }

  /**
   * Sets deploy motor power to zero.
   */
  @Override
  public void initialize() {
    intake.deploy(0);
  }

  @Override
  public void execute() {
    intake.deploy(power);
  }

  /**
   * Sets deploy motor power to zero.
   */
  @Override
  public void end(boolean interrupted) {
    intake.deploy(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
