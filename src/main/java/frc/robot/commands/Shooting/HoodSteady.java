// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.HoodAngler;

/**
 * Command to hold the hood angler at a steady position using a PID controller.
 */
public class HoodSteady extends Command {
  HoodAngler hoodAngler;
  PIDController pid;

  /**
   * Creates the hood angler and PID controller.
   * 
   * @param hoodAngler The hood angler subsystem to control.
   */
  public HoodSteady(HoodAngler hoodAngler) {
    this.pid = new PIDController(0, 0, 0);
    this.hoodAngler = hoodAngler;
    addRequirements(hoodAngler);
  }

  /**
   * No initialization needed since the PID controller
   * will handle everything in execute.
   */
  @Override
  public void initialize() {
  }

  /**
   * Calculates the PID output based on the current position of the hood angler
   * and the target position, and applies it to the hood angler motor.
   */
  @Override
  public void execute() {
    // Get PID Controller direction for elevator to go, find current error from
    // position.
    double direction = pid.calculate(hoodAngler.getPos(), Globals.targetPos.intakeDeployTarget);
    double error = pid.getPositionError();

    if (error > -25 && error < 25) {
      hoodAngler.adjust(0); // deadzone
    } else {
      hoodAngler.adjust(direction); // Move Arm
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