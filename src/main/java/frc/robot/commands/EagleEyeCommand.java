// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.EagleEye;

/**
 * Command that uses functionalities in the EagleEye subsystem and
 * displays usage on the SmartDashboard.
 */
public class EagleEyeCommand extends Command {
  private EagleEye eagleEye = new EagleEye();

  /**
   * Sets the EagleEye subsystem for this command to use.
   * 
   * @param eagleEye EagleEye subsystem to use.
   */
  public EagleEyeCommand(EagleEye eagleEye) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.eagleEye = eagleEye;
    addRequirements(this.eagleEye);
  }

  /**
   * No initialization needed for this command.
   */
  @Override
  public void initialize() {
  }

  /**
   * Continuously updates the SmartDashboard with whether the robot
   * is using EagleEye pathing.
   */
  @Override
  public void execute() {
    SmartDashboard.putBoolean("inPath", Globals.inPath);
  }

  /**
   * No actions needed when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * This command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}