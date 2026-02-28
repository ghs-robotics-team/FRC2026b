package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.Constants.OperatorConstants;

/**
 * Command that drives the robot to a specific point on the field using auto-pathfinding.
 */
public class DriveToPointCommand extends Command {
  TargetPoints point;
  String heading;

  /**
   * Sets point and heading for the command.
   * @param point The target point on the field to drive to.
   * @param heading The desired heading of the robot at the target point.
   */
  public DriveToPointCommand(TargetPoints point, String heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.point = point;
    this.heading = heading;
  }
  
  /**
   * Initializes the command by building a pathfinding command based on the target point and heading, 
   * and schedules it. Also updates SmartDashboard with the target point and sets a global variable 
   * to indicate that the robot is currently in a pathfinding command.
   */
  @Override
  public void initialize() {
    // Build Command based on Point and Constraints.
    Command pathfindingCommand;
    if(heading.equals("Forward")){
      pathfindingCommand = AutoBuilder.pathfindToPose(point.getForward(), new PathConstraints(Constants.OperatorConstants.MAX_SPEED, 2, 2*Math.PI, 4*Math.PI), 0.0);
    }
    else{
      pathfindingCommand = AutoBuilder.pathfindToPose(point.get(), new PathConstraints(Constants.OperatorConstants.MAX_SPEED, 2, 2*Math.PI, 4*Math.PI), 0.0);
    }
    
    // Setup Variables for Field and Robotpose.
    Field2d field = new Field2d();
    field.setRobotPose(point.get());
    SmartDashboard.putData("DTP target point", field);
    
    // Sends data to Globals to indicate that the robot is currently in a command.
    Globals.inPath = true;
    pathfindingCommand.andThen(new InstantCommand(() -> {
        Globals.inPath = false;
      })).schedule();
  }

  /**
   * This method is intentionally left empty as the actual driving logic 
   * is handled by the scheduled pathfinding command.
   */
  @Override
  public void execute() {}

  /**
   * This method is intentionally left empty as there are 
   * no specific actions needed when the command ends,
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * This command is designed to finish immediately after initialization, 
   * as the actual driving logic is handled by the scheduled pathfinding command.
   * @return True, indicating that the command is finished after initialization.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}