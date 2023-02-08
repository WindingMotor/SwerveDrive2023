// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlign extends CommandBase {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> headingFunction;
  private final Supplier<Transform3d> transform3d;


  private double initialHeading;
  private PIDController forwardController;

  private double vX;

  // Command constructor
  public SwerveAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
   Supplier<Double> headingFunction, Supplier<Transform3d> transform3d){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    // Set the inital heading to the navx +||-inf heading. Should be zero on startup!
    this.initialHeading = headingFunction.get();

    this.transform3d = transform3d;

    // Set default PID values for thetaPID
    forwardController = new PIDController(0.001, 0, 0);

    vX = 0.2;

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem, visionSubsystem);

  }

  // Running
  @Override
  public void execute(){

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    DriverStation.reportError("e", true);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, 0, 0, swerveSubsystem.getRotation2d());
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
