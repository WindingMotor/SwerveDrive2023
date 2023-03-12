// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {

  // Create 4 swerve modules with attributes from constants
  private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "Front Left");

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "Front Right");

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "Back Left");

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "Back Right"); 

  // The end of this madness ^_^

  // Create the navX using roboRIO expansion port



  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  

  public SwerveModulePosition[] getModulePositions(){
    return( new SwerveModulePosition[]{
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(),
      backRight.getPosition()});
  }
  // Create a robot monitor
  //private final Monitor monitor = new Monitor();

  // BROKEN FOR 2023
  // Create odometer for error correction
  private drivePoseEstimator;
  // Swerve subsystem constructor
  public SwerveSubsystem() {

    //gyro.setAngleAdjustment(90);

    resetAllEncoders();

    // Reset navX heading on new thread when robot starts
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

    /*
    odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
    new Rotation2d(0), getModulePositions());
    */
    drivePoseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, getHeading, getModulePositions, new Pose2d(0,0,Rotation2d.fromDegrees(getHeading())) );
  }

  // Reset gyro heading 
  public void zeroHeading() {
    gyro.reset();
  }

  public void resetYaw(){
    gyro.zeroYaw();
  }

  // Return heading in -180* to 180* format
  public double getHeading(){
    return gyro.getAngle();
  }

  public Pose2d getOdometryMeters(){
    return(odometer.getPoseMeters());
  }

  // Return heading in Rotation2d format
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }
  
  // Stop all module movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

  // Return robot position caculated buy odometer
  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  // Reset odometer to new location
  public void resetOdometry(Pose2d pose){
   // odometer.resetPosition(pose, getRotation2d());
   odometer.resetPosition(getRotation2d(),getModulePositions(), pose);
  }

  public double getGyroDegrees(){
    // odometer.resetPosition(pose, getRotation2d());
    return Math.IEEEremainder(gyro.getRotation2d().getDegrees(), 360);
   }

  // Reset all swerve module encoders
  public void resetAllEncoders(){
    //DriverStation.reportError("RESET ALL ALL ENCODERS", true);
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
  }

  // Get each module reference for Monitor
  public SwerveModule getFrontLeft(){return(frontLeft);}
  public SwerveModule getFrontRight(){return(frontRight);}
  public SwerveModule getBackLeft(){return(backLeft);}
  public SwerveModule getBackRight(){return(backRight);}
  
  public double getRobotDegrees(){
    double rawValue = gyro.getAngle() % 360.0;
    if(rawValue < 0.0){
      return(rawValue + 360.0);
    }else{
      return(rawValue);
    }
  }

  // Periodic looooooop
  @Override
  public void periodic(){

    // Periodicly update odometer for it to caculate position
    //odometer.update(getRotation2d(), getModulePositions());
    drivePoseEstimator.update()
    // Odometry
    SmartDashboard.putNumber("Heading", getHeading());
   SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
   SmartDashboard.putNumber("ROBOT DEGREES NAVX", getRobotDegrees());
   // SmartDashboard.putNumber("R2D deg", getGyroDegrees());
    
    // Update robot monitor
    //monitor.update();

    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();

  }


  
  public Command auto(PathPlannerTrajectory traj, boolean isFirstPath){
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if(isFirstPath){
          this.resetOdometry(traj.getInitialHolonomicPose());
        }
      }),

      new PPSwerveControllerCommand(traj, this::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0), 
      new PIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController), 
      this::setModuleStates)
    );
  }


}


 

