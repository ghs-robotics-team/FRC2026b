// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Intakes using the Intake only.
 */
public class IntakeOnlyCommand extends Command {
  Intake intake;
  double power;

  /**
   * Creates a new IntakeOnlyCommand.
   * @param intake Intake subsystem to control.
   * @param power The power level to set the intake motor to, typically between -1.0 and 1.0.
   */
  public IntakeOnlyCommand(Intake intake, double power) {
    this.intake = intake;
    this.power = power;
  }

  /**
   * Stops intake motors.
   */
  @Override
  public void initialize() {
    intake.intake(0);
  }

  /**
   * Sets intake motor power to the specified level to intake game pieces.
   */
  @Override
  public void execute() {
    intake.intake(power);
  }

  /**
   * Stops intake motors.
   */
  @Override
  public void end(boolean interrupted) {
    intake.intake(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
