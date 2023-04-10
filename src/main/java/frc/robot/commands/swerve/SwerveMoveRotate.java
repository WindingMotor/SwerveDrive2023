// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveMoveRotate extends CommandBase {

  
  private boolean finished;
  SwerveSubsystem swerveSubsystem;
  Supplier<Boolean> switchOverride;
  Supplier<Double> headingFunction, setpointFunction;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;
  private double vT;

  private double xSetpoint;
  private double ySetpoint;
  private double tSetpoint;

  private double initalHeading;
  private boolean reset;
  private double tolerance = 1.0;

  private Pose2d startingPose;
  private Transform3d visionTransform;
  private Pose2d resetPose;

  // Command constructor
  public SwerveMoveRotate(SwerveSubsystem swerveSubsystem,Supplier<Double> headingFunction,
  double xSetpoint, double ySetpoint, double tSetpoint, boolean reset, Pose2d resetPose){

  this.reset = reset;
  this.resetPose = resetPose;

    addRequirements(swerveSubsystem);

    tolerance = 0.03;

    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    this.tSetpoint = tSetpoint;
    this.xSetpoint = xSetpoint * 2;
    this.ySetpoint = ySetpoint * 2;

    initalHeading = headingFunction.get();

    vXController = new PIDController(1.8, 0.005, 0);
    vYController = new PIDController(1.4, 0.005, 0);

    thetaController = new PIDController(0.1, 0.004, 0.02);
    thetaController.enableContinuousInput(0, 360);

    vX = 0;
    vY = 0;
    vT = 0;

    finished = false;

  }

  @Override
  public void initialize() {
    if(reset){
      swerveSubsystem.resetOdometry(resetPose);
    }

    finished = false;

    initalHeading = headingFunction.get();
    startingPose = swerveSubsystem.getOdometryMeters();

  }

  @Override
  public void execute(){

    if(swerveSubsystem.getOdometryMeters().getY()  > (ySetpoint/2) - tolerance){
      if(swerveSubsystem.getOdometryMeters().getY()  < (ySetpoint/2) + tolerance){

        if(swerveSubsystem.getOdometryMeters().getX()  > (xSetpoint/2) - tolerance){
          if(swerveSubsystem.getOdometryMeters().getX()  < (xSetpoint/2) + tolerance){
            finished = true;
          }
        }

      }
    }


      double turningSpeed;

      turningSpeed = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), tSetpoint);
      // Turning motor deadband 
      turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;


      SmartDashboard.putNumber("ROT CACL", turningSpeed);
      SmartDashboard.putNumber("ODO Y", swerveSubsystem.getOdometryMeters().getY());
      SmartDashboard.putNumber("ROBO DEG", swerveSubsystem.getRobotDegrees());
      
      double xError = ( swerveSubsystem.getOdometryMeters().getX() + (swerveSubsystem.getOdometryMeters().getX() - startingPose.getX()));
      double yError = ( swerveSubsystem.getOdometryMeters().getY() + (swerveSubsystem.getOdometryMeters().getY() - startingPose.getY()));
       
      vX = vXController.calculate(xError, xSetpoint); // X-Axis PID
      vY = vYController.calculate(yError, ySetpoint); // Y-Axis PID

      SmartDashboard.putNumber("vX", vX);
      SmartDashboard.putNumber("vY", vY);
     
      SmartDashboard.putNumber("X Error", xError);
      SmartDashboard.putNumber("Y Error", yError);

      // Create chassis speeds
      ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, turningSpeed, swerveSubsystem.getRotation2d());

      // Create states array
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      swerveSubsystem.setModuleStates(moduleStates);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return finished;}

  
}


























