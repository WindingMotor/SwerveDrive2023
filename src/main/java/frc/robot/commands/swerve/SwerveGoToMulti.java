// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;

import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveGoToMulti extends CommandBase {

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
  private double vT;

  private double xSetpoint;
  private double ySetpoint;
  private double tSetpoint;

  private double initalHeading;
  private boolean reset;
  private double tolerance;

  private Pose2d startingPose;
  private Transform3d visionTransform;
  private Pose2d resetPose;
  SetPoint currentSetPoint;
  List<SetPoint> setpoints;
  int count;

  // Command constructor
  public SwerveGoToMulti(SwerveSubsystem swerveSubsystem,Supplier<Double> headingFunction,
  //double xSetpoint, double ySetpoint, double tSetpoint, boolean reset, Pose2d resetPose){
  List<SetPoint> setpoints, double tolerance){
    this.setpoints = setpoints;
    count = 0;
    currentSetPoint = setpoints.get(count);


    this.reset = currentSetPoint.getReset();
    this.resetPose = currentSetPoint.getPose2d();

    addRequirements(swerveSubsystem);

    this.tolerance = tolerance;

    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    this.tSetpoint = currentSetPoint.gettSetpoint();
    this.xSetpoint = currentSetPoint.getxSetpoint();
    this.ySetpoint = currentSetPoint.getySetpoint();

    initalHeading = headingFunction.get();

    vXController = new PIDController(6, 0.006, 0.008);
    vYController = new PIDController(6, 0.006, 0.008);

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
    xflag=false;
    yflag = false;

    initalHeading = headingFunction.get();
    startingPose = swerveSubsystem.getOdometryMeters();

  }

  @Override
  public void execute(){

    if(swerveSubsystem.getOdometryMeters().getY()  > (currentSetPoint.getySetpoint()) - tolerance){
      if(swerveSubsystem.getOdometryMeters().getY()  < (currentSetPoint.getySetpoint()) + tolerance){
        yflag = true;
      }
    }
    if(swerveSubsystem.getOdometryMeters().getX()  > (currentSetPoint.getxSetpoint()) - tolerance){
      if(swerveSubsystem.getOdometryMeters().getX()  < (currentSetPoint.getxSetpoint()) + tolerance){
        xflag = true;
      }
    }

    if(yflag && xflag){
        if(count < setpoints.size()-1){
            count++;
            currentSetPoint = setpoints.get(count);
            yflag = false;
            xflag = false;
        } else{
          finished = true;
        }
    }

      double turningSpeed;

      turningSpeed = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), currentSetPoint.gettSetpoint());
      // Turning motor deadband
      turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;


      SmartDashboard.putNumber("ROT CACL", turningSpeed);
      SmartDashboard.putNumber("ODO Y", swerveSubsystem.getOdometryMeters().getY());
      SmartDashboard.putNumber("ODO X", swerveSubsystem.getOdometryMeters().getX());
      SmartDashboard.putNumber("ROBO DEG", swerveSubsystem.getRobotDegrees());

      //double xError = ( swerveSubsystem.getOdometryMeters().getX() + (swerveSubsystem.getOdometryMeters().getX() - startingPose.getX()));
      //double yError = ( swerveSubsystem.getOdometryMeters().getY() + (swerveSubsystem.getOdometryMeters().getY() - startingPose.getY()));

      vX = vXController.calculate(swerveSubsystem.getOdometryMeters().getX(), currentSetPoint.getxSetpoint()); // X-Axis PID
      vY = vYController.calculate(swerveSubsystem.getOdometryMeters().getY(), currentSetPoint.getySetpoint()); // Y-Axis PID

      SmartDashboard.putNumber("vX", vX);
      SmartDashboard.putNumber("vY", vY);

     // SmartDashboard.putNumber("X Error", xError);
     // SmartDashboard.putNumber("Y Error", yError);

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