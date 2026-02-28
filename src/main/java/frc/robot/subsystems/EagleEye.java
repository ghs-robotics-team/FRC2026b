// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EagleEyeConstants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;

/**
 * EagleEye subsystem for vision processing and pose estimation.
 */
public class EagleEye extends SubsystemBase {
  

  /**
   * Nothing done in init.
   */
  public EagleEye() {
  }

  /**
   * Determines confidence level of limelight estimations based on distance, tag count, and robot motion.
   * @param limelight The Limelight pose estimate.
   * @return Confidence level between 0 and 1.
   */
  public double limelightMeasurement(LimelightHelpers.PoseEstimate limelight) {
    
    // Hard rejections
    if (limelight.tagCount < 1)
      return 0.0;
    if (limelight.avgTagDist > EagleEyeConstants.MAX_TAG_DISTANCE_METERS)
      return 0.0;
    if (Math.abs(Globals.EagleEye.rotVel) > EagleEyeConstants.MAX_ROTATION_VELOCITY)
      return 0.0;
    if (Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) > EagleEyeConstants.MAX_VISION_SPEED)
      return 0.0;

    // Start with full trust
    double confidence = 1.0;

    // Distance Penalties
    double distMeters = limelight.avgTagDist;
    double distFactor = MathUtil.clamp(1.0 - distMeters / EagleEyeConstants.MAX_TAG_DISTANCE_METERS, 0.0, 1.0);
    confidence *= distFactor;

    // Tag Count Bonuses
    if (limelight.tagCount >= 2) {
      confidence *= 1.0; // keep
    } else {
      confidence *= 0.75; // single-tag penalty
    }

    /*  Compare with Odometry
    double odomError = limelight.pose.getTranslation()
        .getDistance(Globals.EagleEye.position.getTranslation());

    double odomFactor = MathUtil.clamp(1.0 - odomError / 3.0, 0.2, 1.0);
    confidence *= odomFactor;*/

    // Disabled or stationary bonus
    if (DriverStation.isDisabled()) {
      confidence *= 1.2;
    }

    // Clamp final confidence to [0, 1] to avoid invalid values.
    return MathUtil.clamp(confidence, 0.0, 1.0);
  }

  /**
   * Periodically updates vision measurements using two Limelight cameras, 
   * getting confidence levels and storing the latest measurements in Globals.
   */
  @Override
  public void periodic() {
    if (RobotBase.isSimulation())
      return;

    // Don't Read Eagleye during Teleop Paths
    /*
     * if(Constants.EagleEyeConstants.IN_PATH_END && Globals.inPath){
     * Globals.LastVisionMeasurement.confidencea = 0;
     * Globals.LastVisionMeasurement.confidenceb = 0;
     * return;
     * }
     */

    // If we don't update confidence then we don't send the measurement
    double confidenceA = 0;
    double confidenceB = 0;

    LimelightHelpers.SetRobotOrientation("limelight-camb", Globals.EagleEye.rawGyroYaw, 0, 0,
        0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-cama", Globals.EagleEye.rawGyroYaw, 0, 0,
        0, 0, 0);

    LimelightHelpers.PoseEstimate limelightMeasurementA = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-cama");
    LimelightHelpers.PoseEstimate limelightMeasurementB = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-camb");

    if (limelightMeasurementA != null) {
      SmartDashboard.putNumber("EEA NumTags", limelightMeasurementA.tagCount);
      SmartDashboard.putNumber("EEA Avg Tag Dist", limelightMeasurementA.avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("EE Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      confidenceA = limelightMeasurement(limelightMeasurementA);

      Globals.LastVisionMeasurement.positionA = limelightMeasurementA.pose;
      Globals.LastVisionMeasurement.timeStampA = limelightMeasurementA.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;

    }

    if (limelightMeasurementB != null) {

      SmartDashboard.putNumber("EEB NumTags", limelightMeasurementB.tagCount);
      SmartDashboard.putNumber("EEB Avg Tag Dist", limelightMeasurementB.avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("EE Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      confidenceB = limelightMeasurement(limelightMeasurementB);

      // No tag found so check no further or pose not within field boundary
      Globals.LastVisionMeasurement.positionB = limelightMeasurementB.pose;
      Globals.LastVisionMeasurement.timeStampB = limelightMeasurementB.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;

    }
    Globals.LastVisionMeasurement.confidenceA = confidenceA;
    Globals.LastVisionMeasurement.confidenceB = confidenceB;

    // ===== SHOOTING DATA COLLECTION =====
    if (Constants.OperatorConstants.SHOOTING_DATA_COLLECTION_MODE) {
      if (SmartDashboard.getBoolean("Record Data", false)) {
        File file = new File(Paths.get("src", "main", "deploy", "shootingData.txt").toUri());
        try (FileWriter writer = new FileWriter(file)) {
          // Dist Angle
          writer.write(String.valueOf(SmartDashboard.getNumber("dist", 0)) + "  "
              + String.valueOf(SmartDashboard.getNumber("Test Angle", 0)) + "\n");
        } catch (IOException e) {
          e.printStackTrace();
        }

        SmartDashboard.putBoolean("Record Data", false);
      }

      boolean button = SmartDashboard.getBoolean("Record Time Data", false);
      Timer timer = Globals.shootingDataCollectionSettings.timer;

      // Rising-edge detection
      if (button && !Globals.shootingDataCollectionSettings.lastButtonState) {

        if (!Globals.shootingDataCollectionSettings.recording) {

          // ===== START RECORDING =====
          timer.reset();
          timer.start();
          Globals.shootingDataCollectionSettings.recording = true;

          Globals.shootingDataCollectionSettings.startPose = Globals.EagleEye.position;

        } else {

          // ===== STOP RECORDING =====
          timer.stop();

          double elapsedTime = timer.get();
          Globals.shootingDataCollectionSettings.endPose = Globals.EagleEye.position;
          double distance = Globals.shootingDataCollectionSettings.endPose.getTranslation()
          .getDistance(Globals.shootingDataCollectionSettings.startPose.getTranslation());


          File file = new File(Paths.get("src", "main", "deploy", "shootingTimeData.txt").toUri());
          try (FileWriter writer = new FileWriter(file)) {
            // Dist Time
            writer.write(String.valueOf(distance) + "  "
                + String.valueOf(elapsedTime) + "\n");
          } catch (IOException e) {
            e.printStackTrace();
          }

          Globals.shootingDataCollectionSettings.recording = false;
        }

        // Make dashboard act like a button
        SmartDashboard.putBoolean("Record Time Data", false);
      }
      // Save last button state
      Globals.shootingDataCollectionSettings.lastButtonState = button;
    }
  }
}