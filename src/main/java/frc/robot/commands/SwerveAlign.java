// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlign extends CommandBase {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> headingFunction;


  private double initialHeading;
  private PIDController forwardController;

  // Command constructor
  public SwerveAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem VisionSubsystem, Supplier<Double> headingFunction){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    // Set the inital heading to the navx +||-inf heading. Should be zero on startup!
    this.initialHeading = headingFunction.get();

    // Set default PID values for thetaPID
    forwardController = new PIDController(0.001, 0, 0);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  // Running
  @Override
  public void execute(){


    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, swerveSubsystem.getRotation2d());
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
  public boolean isFinished(){return false;}

}
