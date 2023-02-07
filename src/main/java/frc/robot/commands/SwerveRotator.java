// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveRotator extends CommandBase {

  //-----------------------------//
  // TO BE DEPRECATED AND REWRITTEN!
  //-----------------------------//

  /*

  // Create empty variables for reassignment
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> turningSpdFunction;
  private double startHeading;
  private double currentHeading;
  private boolean isDone;

  // Command constructor and requirements 
  public SwerveRotator(SwerveSubsystem swerveSubsystem, Supplier<Double> turningSpdFunction, double startHeading) {

    // Assign empty variables values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.turningSpdFunction = turningSpdFunction;
    this.startHeading = startHeading;
    isDone = false;

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  // Running loop of command
  @Override
  public void execute(){

    // Get current heading
    currentHeading = swerveSubsystem.getHeading();

    // Set turning speed
    double turningSpeed = turningSpdFunction.get();

    // Set chassis speeds2
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(0,0,turningSpeed);

    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Set each module state
    swerveSubsystem.setModuleStates(moduleStates);

    // Stop command if heading is correct
    if(currentHeading >= 89 & currentHeading <= 91){
      isDone = true;
    }
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return isDone;}

  */

  //-----------------------------//
  // TO BE DEPRECATED AND REWRITTEN!
  //-----------------------------//

}