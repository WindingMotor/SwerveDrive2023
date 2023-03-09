// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlignBasic extends CommandBase {

  
  private boolean finished;
  SwerveSubsystem swerveSubsystem;
  VisionSubsystem visionSubsystem;
  Supplier<Boolean> switchOverride;
  Supplier<Double> headingFunction, setpointFunction;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;
  private double vT;

  private double initalHeading;

  private Pose2d startingPose;
  private Transform3d visionTransform;

  // Command constructor
  public SwerveAlignBasic(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
   Supplier<Double> headingFunction, Supplier<Boolean> switchOverride, Supplier<Double> setpointFunction){

    addRequirements(swerveSubsystem, visionSubsystem);

    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.switchOverride = switchOverride;
    this.headingFunction = headingFunction;
    this.setpointFunction = setpointFunction;

    initalHeading = headingFunction.get();

    vXController = new PIDController(0.5, 0.005, 0);
    vYController = new PIDController(1.4, 0.005, 0);
    thetaController = new PIDController(0.0009, 0, 0);

    vX = 0;
    vY = 0;
    vT = 0;

    finished = false;

  }

  @Override
  public void initialize() {
    finished = false;

    initalHeading = headingFunction.get();
    startingPose = swerveSubsystem.getOdometryMeters();

    if(visionSubsystem.hasTargets()){
      visionTransform = visionSubsystem.getTargetTransform();
    }else{
      finished = true;
    }

  }

  @Override
  public void execute(){

//- 0.28575
    try
    {
      double xError = ( -visionTransform.getX() + (swerveSubsystem.getOdometryMeters().getX() - startingPose.getX()));
      double yError = ( visionTransform.getY() - (swerveSubsystem.getOdometryMeters().getY() - startingPose.getY()));
 
       
         vY = vXController.calculate(visionSubsystem.getTargetTransform().getX(), 0.5); // Y-Axis PID
         vX = vYController.calculate(visionSubsystem.getTargetTransform().getY(), -0.28575); // X-Axis PID
 
         //vT = -thetaController.calculate(swerveSubsystem.getGyroDegrees(), 90) * 300; // Rotation PID
       SmartDashboard.putNumber("vX", vT);
       SmartDashboard.putNumber("vY", vT);
     
       SmartDashboard.putNumber("X Error", xError);
       SmartDashboard.putNumber("Y Error", yError);
  
      if(switchOverride.get() == false){
        finished = true;
      }
  
      // Create chassis speeds
      ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vT, swerveSubsystem.getRotation2d());
      //chassisSpeeds = new ChassisSpeeds(vX, vY, vT);
      //chassisSpeeds = new ChassisSpeeds(0.5,0,0);
  
      // Create states array
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      swerveSubsystem.setModuleStates(moduleStates);
    }
    catch (Exception e)
    {
      e.printStackTrace();
      finished = true;
    }

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return finished;}

  
}


























