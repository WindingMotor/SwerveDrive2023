// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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

    vXController = new PIDController(1.8, 0.005, 0);
    vYController = new PIDController(1.4, 0.005, 0);
    thetaController = new PIDController(0.1, 0, 0);
    thetaController.enableContinuousInput(0, 360);

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
      SmartDashboard.putString("Vision Start Transform", visionSubsystem.getTargetTransform().toString());
    }else{
      finished = true;
    }

  }

  @Override
  public void execute(){

//- 0.28575
    try
    {

      double xError = ( visionTransform.getY() + (swerveSubsystem.getOdometryMeters().getX() - startingPose.getX()));
      double yError = ( visionTransform.getX() + (swerveSubsystem.getOdometryMeters().getY() - startingPose.getY()));
      //double tError = ( (-visionTransform.getRotation().getAngle() * 360) / (2*Math.PI) - (swerveSubsystem.getRobotDegrees())) + 180;
 //0.28575
       
         vX = vXController.calculate(xError, -0.28575); // X-Axis PID
         vY = vYController.calculate(yError, 0.5); // Y-Axis PID
         vT = thetaController.calculate( Math.toRadians(swerveSubsystem.getRobotDegrees()), 0); // Rotation PID

       SmartDashboard.putNumber("vX", vX);
       SmartDashboard.putNumber("vY", vY);
     
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
      DriverStation.reportError("E  xeption", true);
      finished = true;
    }

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return finished;}

  
}


























