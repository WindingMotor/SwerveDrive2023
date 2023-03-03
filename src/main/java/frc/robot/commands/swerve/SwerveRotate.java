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

  /* 
  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> headingFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private double initialHeading;
  private PIDController thetaController;

  double turningSpeed = 0;

  // Command constructor
  public SwerveJoystick(SwerveSubsystem swerveSubsystem,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Double> headingFunction){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.headingFunction = headingFunction;

    // Set the inital heading to the navx +||-inf heading. Should be zero on startup!
    this.initialHeading = headingFunction.get();

    // Set default PID values for thetaPID
    thetaController = new PIDController(DriveConstants.kPThetaController, DriveConstants.kIThetaController, DriveConstants.kDThetaController);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    //swerveSubsystem.resetYaw();
    initialHeading = headingFunction.get();

  }

  // Running loop of command
  @Override
  public void execute(){

    // Get current navx heading
    double newHeading = headingFunction.get();

    turningSpeed = thetaController.calculate(newHeading, initialHeading);
    //turningSpeed = (headingFunction.get() - initialHeading) * turningPValue;
    
    // Turning deadband 
    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
    turningSpeed *= -1;

    // Limit turning rat
    if (turningSpeed > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    else if (turningSpeed < -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    
    // Smartdashboard update
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    SmartDashboard.putNumber("Inital Heading", initialHeading);
    SmartDashboard.putNumber("NavX Heading", headingFunction.get());

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerveSubsystem.getRotation2d());
    //chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);

    // Put field oriented value on smart dashboard
    // SmartDashboard.putBoolean("Field Oriented: ", fieldOrientedFunction.get());

    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
    // Set the module state
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return false;}

  */
}
