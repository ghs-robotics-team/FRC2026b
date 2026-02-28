// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shoots game pieces at end of subsystem chain. 
 * Uses a single motor to shoot and a hood angler to adjust the angle of the shot.
 */
public class Shooter extends SubsystemBase {
  SparkFlex shooterTop = new SparkFlex(18, MotorType.kBrushless);
  SparkFlex shooterBottom = new SparkFlex(8, MotorType.kBrushless);

  /**
   * Nothing done in constructor.
   */
  public Shooter() {}

  /**
   * Sets the power level of the shooter motor to shoot game pieces.
   * @param power The power level to set the shooter motor to, typically between -1.0 and 1.0.
   */
  public void shoot(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("Shooting V", 0.1);
    }
    shooterTop.set(power);
    shooterBottom.set(power);
  }

  /*
   * Periodic is not used.
   */
  @Override
  public void periodic() {
  }
}
