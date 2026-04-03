package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Globals.EagleEye;
import frc.robot.commands.Shooting.HoodAnglerPositionCommand;
import frc.robot.subsystems.HoodAngler;
import frc.robot.ShootingHelpers;

/**
 * Command adjusts shooter angle based on interpolation done in ShootingHelpers.
 */
public class PitchToSpeaker extends Command {
  private HoodAngler angler;

  /**
   * Sets HoodAngler and adds it as a requirement.
   * @param angler Subsystem that controls the angle of the shooter.
   */
  public PitchToSpeaker(HoodAngler angler) {
    addRequirements(angler);
    this.angler = angler;
    SmartDashboard.putNumber("PTS - Test Angle", Constants.SetPointConstants.TEST);
  }

  /**
   * Initialize does nothing.
   */
  @Override
  public void initialize() {
  }

  /**
   * Either angles shooter to target angle on Smartdashboard if in data collection mode, 
   * or angles shooter to target angle based on interpolation from ShootingHelpers.
   */
  @Override
  public void execute() {
    if(!OperatorConstants.SHOOTING_DATA_COLLECTION_MODE){
      Translation2d speakerPos = ShootingHelpers.getTargetPos();

      double targetAngle = ShootingHelpers.angleInterp(speakerPos);

      new HoodAnglerPositionCommand(angler, targetAngle);
    }else{
      SmartDashboard.putNumber("PTS - Distance", ShootingHelpers.getTargetPos().getDistance(EagleEye.position.getTranslation()));
      new HoodAnglerPositionCommand(angler, SmartDashboard.getNumber("PTS - Test Angle", Constants.SetPointConstants.TEST));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}