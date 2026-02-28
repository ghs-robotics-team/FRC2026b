// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Globals;

/**
 * Retractable intake subsystem. Deploys and then intakes using a single motor.
 * Feeds into Spindexer, which feeds into Shooter.
 */
public class Intake extends SubsystemBase {
  SparkFlex intakeMotor = new SparkFlex(24, MotorType.kBrushless);
  SparkMax deployMotor = new SparkMax(27, MotorType.kBrushless);
  //double IntakeAbsoluteEncoder = intakeMotor.getPosition().getValue().magnitude();
  double deployAbsoluteEncoder = deployMotor.getAbsoluteEncoder().getPosition();

  /**
   * Nothing done in constructor.
   */
  public Intake() {}

  /**
   * Deploys the intake by setting the deploy motor to a specified power level.
   * @param power The power level to set the deploy motor to, typically between -1.0 and 1.0.
   */
  public void intake(double power) {
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("Intake V", 0.1);
    }
    intakeMotor.set(-power);
  }

  /**
   * Deploys the intake by setting the deploy motor to a specified power level.
   * @param power The power level to set the deploy motor to, typically between -1.0 and 1.0. 
   */
  public void deploy(double power) {
    // When limits are needed on position, check last years code for reference.
    // Needs PID.
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("IntakeDeploy V", 0.1);
    }
    /* Tune location numbers
    if(power<=0){
      if(getPos() <= 0){ 
        deployMotor.set(-power);
      }
      else{
        deployMotor.set(0);
      }
    }
    else{
      if(getPos() > -20.7){
        deployMotor.set(-power);
      }
      else{
        deployMotor.set(0);
      }
    } */
    deployMotor.set(power);
  }

  /**
   * Get absolute encoder position
   * @return Absolute Encoder Position of the motor.
   */
  public double getPos(){
    return deployAbsoluteEncoder;
  }

  /**
   * Periodically updates the SmartDashboard with the current position 
   * of the intake and the target position for deployment.
   */
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("INT Pos", deployAbsoluteEncoder);
    SmartDashboard.putNumber("INT Deploy Target Pos", Globals.targetPos.intakeDeployTarget);
    SmartDashboard.putNumber("INT POS", deployAbsoluteEncoder);
  }
}
