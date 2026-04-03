// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intaking;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.Intake;

/**
 * Keeps the deployed intake steady at its up or down positions using
 * a PID controller.
 */
public class DeploySteady extends Command {
  private Intake intake;
  private PIDController pid;

  /**
   * Creates the PID controller and sets the intake subsystem as a requirement.
   * 
   * @param intake The intake subsystem to control.
   */
  public DeploySteady(Intake intake) {
    this.pid = new PIDController(0, 0, 0);
    this.intake = intake;
    addRequirements(intake);
  }

  /**
   * No initialization needed since the PID controller
   * will handle everything in execute.
   */
  @Override
  public void initialize() {
  }

  /**
   * Calculates the PID output based on the current position of the intake
   * and the target position, and applies it to the intake deploy motor.
   * If the error is within a certain range, it applies a deadzone to prevent
   * jittering.
   */
  @Override
  public void execute() {
    double direction = pid.calculate(intake.getPos(), Globals.targetPos.intakeDeployTarget);
    double error = pid.getPositionError();

    if (error > -25 && error < 25) {
      intake.deploy(0); // deadzone
    } else {
      intake.deploy(direction); // Move Arm
    }
  }

  /**
   * Nothing done in end.
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Command does not finish on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}