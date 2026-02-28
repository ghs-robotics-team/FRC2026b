package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum representing target points on the field, each associated with a specific pose.
 * Provides methods to calculate the robot's position relative to the target and to adjust for alliance color
 */
public enum TargetPoints {
    TAG_28(new Pose2d(Units.inchesToMeters(180.08), Units.inchesToMeters(24.85), Rotation2d.fromDegrees(180))),
    TAG_25(new Pose2d(Units.inchesToMeters(157.79), Units.inchesToMeters(172.32), Rotation2d.fromDegrees(180)));
    
    public Pose2d pose;
    private static final double RED_BLUE_OFFSET = Units.inchesToMeters(8.57); // 8.57 inches in meters

    /**
     * Constructor for TargetPoints enum, initializes the pose for each target point.
     * @param pose
     */
    private TargetPoints(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Calculates the robot's position relative to the target point by 
     * adjusting the pose based on a fixed distance (13.77 inches) from the target.
     * @param pose The current pose of the robot, used to determine the direction of adjustment.
     * @return The adjusted pose of the robot relative to the target point.
     */
    public Pose2d distanceFromTag(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        x += Units.inchesToMeters(23.77) * Math.cos(pose.getRotation().getRadians()); 
        y += Units.inchesToMeters(23.77) * Math.sin(pose.getRotation().getRadians());
        return new Pose2d(x, y, Rotation2d.fromDegrees(90).plus(pose.getRotation()));
    }

    /**
     * Calculates a new pose for the robot based on a specified distance (in inches) from the target point,
     * adjusting the position in the direction the robot is facing. The method accounts for the direction 
     * of movement (forward or backward) based on the sign of the inches parameter 
     * and corrects for WPILib's angle measurements.
     * @param pose The current pose of the robot, used to determine the direction of adjustment.
     * @param inches The distance in inches to adjust the robot's position from the target point. 
     * Positive values indicate forward movement, while negative values indicate backward movement.
     * @return The adjusted pose of the robot.
     */
    public static Pose2d tagPos(Pose2d pose, double inches){ 
        // Current X and Y Position of the Robot.
        double x = pose.getX();
        double y = pose.getY();
        double angle_degree;

        // Correcting for Weird WPILib Angle Measurements.
        if (inches >= 0) {
        angle_degree = pose.getRotation().getDegrees() - 90;
        } else {
        angle_degree = pose.getRotation().getDegrees() + 90;
        }
        double angle = Units.degreesToRadians(angle_degree + 90);

        // Calculate new X position based on Trigonometry
        if (inches >= 0) {
            x += Units.inchesToMeters(inches) * Math.cos(angle);
            } else {
            x -= Units.inchesToMeters(inches) * Math.cos(angle);
        }
        
        // Calculate new Y based on if the robot is moving right or left.
        if (inches >= 0) {
        y += Units.inchesToMeters(inches) * Math.sin(angle);
        } else {
        y -= Units.inchesToMeters(inches) * Math.sin(angle);
        }

        return new Pose2d(x, y, pose.getRotation());
    }

    /**
     * Calculates the robot's pose relative to the target point, adjusting for alliance color.
     * @return The adjusted pose of the robot relative to the target point.
     */
    public Pose2d get() {
        Pose2d newpose = distanceFromTag(pose);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return new Pose2d(newpose.getX() + RED_BLUE_OFFSET, newpose.getY(), newpose.getRotation());
        } else {
            return newpose;
        }
    }

    /**
     * Calculates the robot's pose relative to the target point, 
     * adjusting for alliance color and adding a 90 degree rotation.
     * @return The adjusted pose of the robot relative to the target point, with a 90 degree rotation added.
     */
    public Pose2d getForward() {
        Pose2d newpose = distanceFromTag(pose);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return new Pose2d(newpose.getX() + RED_BLUE_OFFSET, newpose.getY(), newpose.getRotation().plus(Rotation2d.fromDegrees(90)));
        } else {
            return new Pose2d(newpose.getX(), newpose.getY(), newpose.getRotation().plus(Rotation2d.fromDegrees(90)));
        }
    }

    /**
     * Prints the name and adjusted pose of each target point in the enum to the console for debugging purposes.
     */
    public static void printPlaces(){
        for(TargetPoints point: TargetPoints.values()){
            System.out.println(point.name() + " :" + TargetPoints.tagPos(new Pose2d(point.get().getX(), point.get().getY(), point.get().getRotation()), -15)); //-2 or -15
        }
    }
}