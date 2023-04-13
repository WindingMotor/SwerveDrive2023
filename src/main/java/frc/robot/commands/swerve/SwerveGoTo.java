// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveGoTo extends CommandBase {

  
  private boolean finished;
  private boolean xflag;
  private boolean yflag;
  SwerveSubsystem swerveSubsystem;
  Supplier<Boolean> switchOverride;
  Supplier<Double> headingFunction, setpointFunction;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;

  private double xSetpoint;
  private double ySetpoint;
  private double tSetpoint;

  private boolean reset;
  private double tolerance;


  private Pose2d resetPose;

  // Command constructor
  public SwerveGoTo(SwerveSubsystem swerveSubsystem,Supplier<Double> headingFunction,
  
  double xSetpoint, double ySetpoint, double tSetpoint, boolean reset, Pose2d resetPose,
  double tolerance){

  this.reset = reset;
  this.resetPose = resetPose;

    addRequirements(swerveSubsystem);

    this.tolerance = tolerance;


    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    this.tSetpoint = tSetpoint;
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;

    vXController = new PIDController(6, 0.006, 0.008);
    vYController = new PIDController(6, 0.006, 0.008);


    thetaController = new PIDController(0.1, 0.004, 0.02);
    thetaController.enableContinuousInput(0, 360);

    vX = 0;
    vY = 0;

    finished = false;

  }

  @Override
  public void initialize() {
  
    if(reset){ swerveSubsystem.resetOdometry(resetPose);}

    finished = false;
    xflag = false;
    yflag = false;
  
  }

  @Override
  public void execute(){

      if(swerveSubsystem.getOdometryMeters().getY()  > (ySetpoint) - tolerance){
      if(swerveSubsystem.getOdometryMeters().getY()  < (ySetpoint) + tolerance){
          yflag = true;
        }
      }

      if(swerveSubsystem.getOdometryMeters().getX()  > (xSetpoint) - tolerance){
      if(swerveSubsystem.getOdometryMeters().getX()  < (xSetpoint) + tolerance){
          xflag = true;
        }
      }

      if(yflag && xflag){ finished = true;}

      double turningSpeed;

      turningSpeed = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), tSetpoint);

      turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

      SmartDashboard.putNumber("ROT CACL", turningSpeed);
      SmartDashboard.putNumber("ODO Y", swerveSubsystem.getOdometryMeters().getY());
      SmartDashboard.putNumber("ODO X", swerveSubsystem.getOdometryMeters().getX());
      SmartDashboard.putNumber("ROBO DEG", swerveSubsystem.getRobotDegrees());

      vX = vXController.calculate(swerveSubsystem.getOdometryMeters().getX(), xSetpoint); // X-Axis PID
      vY = vYController.calculate(swerveSubsystem.getOdometryMeters().getY(), ySetpoint); // Y-Axis PID

      SmartDashboard.putNumber("vX", vX);
      SmartDashboard.putNumber("vY", vY);

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


























