// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem using a hook.
 */
public class Climber extends SubsystemBase {
  SparkFlex climbMotor = new SparkFlex(7, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  double climbMotorEncoder = climbMotor.getAbsoluteEncoder().getPosition();

  /**
   * Nothing done in constructor.
   */
  public Climber() {
  }

  /**
   * Runs the climber motor.
   * 
   * @param power The power to run the motor at, ranging from -1 to 1.
   */
  public void climb(double power) {
    climbMotor.set(-power);
  }

  /**
   * Get absolute encoder position
   * 
   * @return Absolute Encoder Position of the motor.
   */
  public double getPos() {
    return climbMotorEncoder;
  }

  /**
   * Displays the position of the claw motor in SmartDashBoard.
   */
  @Override
  public void periodic() {
    climbMotorEncoder = climbMotor.getAbsoluteEncoder().getPosition();
    SmartDashboard.putNumber("CLM - Position", climbMotorEncoder);
    SmartDashboard.putNumber("CLM - Motor Power", climbMotor.getAppliedOutput());
  }
}