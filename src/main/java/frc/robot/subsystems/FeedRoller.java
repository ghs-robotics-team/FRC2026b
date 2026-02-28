// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkFlex;

/**
 * Feeds the game piece to the shooter subsystem.
 */
public class FeedRoller extends SubsystemBase {
  SparkFlex rollerMotor = new SparkFlex(13, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

  /**
   * Nothing done in constructor.
   */
  public FeedRoller() {}

  /**
   * Nothing done in periodic.
   */
  @Override
  public void periodic() {
  }

  /**
   * Runs the FeedRoller motor.
   * @param power Power to run the motor at, ranging from -1 to 1.
   */
  public void roll (double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("Feed Roll V", 0.1);
    }
    rollerMotor.set(power);
  }
}
