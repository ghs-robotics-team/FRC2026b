// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem using a hook.
 */
public class Climber extends SubsystemBase {
  SparkFlex climbMotor = new SparkFlex(7, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  double climbMotorEncoder = climbMotor.getAbsoluteEncoder().getPosition();

  /**
   * Nothing done in constructor.
   */
  public Climber() {}

  /**
   * Runs the climber motor.
   * @param power The power to run the motor at, ranging from -1 to 1.
   */
  public void climb(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("Climber V", 0.1);
    }
    /* Tune location numbers
    if(power<=0){
      if(getPos() <= 0){ 
        climbMotor.set(-power);
      }
      else{
        climbMotor.set(0);
      }
    }
    else{
      if(getPos() > -20.7){
        climbMotor.set(-power);
      }
      else{
        climbMotor.set(0);
      }
    } */
    climbMotor.set(-power);
  }

  /**
   * Get absolute encoder position
   * @return Absolute Encoder Position of the motor.
   */
  public double getPos(){
    return climbMotorEncoder;
  }

  /**
   * Displays the position of the claw motor in SmartDashBoard.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("CL POS", climbMotorEncoder);
  }
}
