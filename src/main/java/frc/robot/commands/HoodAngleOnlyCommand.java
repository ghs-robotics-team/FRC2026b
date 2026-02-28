// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodAngler;

/**
 * Command to set the hood angle only, without running the shooter or spindexer.
 */
public class HoodAngleOnlyCommand extends Command {
  HoodAngler hoodAngler;
  double power;
  
  /**
   * Creates a new HoodAngleOnlyCommand.
   * @param hoodAngler The hood angler subsystem to control.
   * @param power The power level to set the hood angler motor to, typically between -1.0 and 1.0.
   */
  public HoodAngleOnlyCommand(HoodAngler hoodAngler, double power) {
    this.hoodAngler = hoodAngler;
    this.power = power;
    addRequirements(hoodAngler);
  }

  /**
   * Sets hood angler motor power to zero.
   */
  @Override
  public void initialize() {
    hoodAngler.adjust(0.1);
  }

  /**
   * Sets hood angler motor power to the specified level to adjust the hood angle.
   */
  @Override
  public void execute() {
    hoodAngler.adjust(power);
  }

  /**
   * Sets hood angler motor power to zero.
   */
  @Override
  public void end(boolean interrupted) {
    hoodAngler.adjust(0);
  }

  /**
   * Returns false to keep the command running until interrupted.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
