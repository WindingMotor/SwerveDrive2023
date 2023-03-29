// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveRotate extends CommandBase {

  
  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  
  private PIDController thetaController;
  private double angle;
  private boolean finished;
  private double angleTolerance;

  // Command constructor
  public SwerveRotate(SwerveSubsystem swerveSubsystem, double angle){

    this.angle = angle;
    finished = false;

    // Ending command tolerance
    angleTolerance = 0.1;

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;

    // Set default PID values for thetaPID
    thetaController = new PIDController(DriveConstants.kPThetaController, DriveConstants.kIThetaController, DriveConstants.kDThetaController);

    // Enable continuous input for thetaPID
    thetaController.enableContinuousInput(0, 360);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    // Set default command values
    finished = false;
  }

  // Running loop of command
  @Override
  public void execute(){

    if(swerveSubsystem.getRobotDegrees() > angle - angleTolerance && swerveSubsystem.getRobotDegrees() < angle + angleTolerance){
      finished = true;
    }

    // Get current navx heading
    double turningSpeed;

    turningSpeed = thetaController.calculate(swerveSubsystem.getRobotDegrees(), angle);
    
    // Turning motor deadband 
    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

    // Smartdashboard update
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    
    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerveSubsystem.getRotation2d());
    //chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);

    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
    // Set the module state
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return finished;}

  
}
