// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /*private SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
  new Rotation2d(0), getModulePositions());*/

  // Create empty right joystick for live speed control: BORKED!
  Joystick rightJoystick;

  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  // Swerve subsystem constructor
  public SwerveSubsystem(Joystick rightJoystick) {

    //gyro.setAngleAdjustment(90);

    // Assign right joystick
    this.rightJoystick = rightJoystick;

    resetAllEncoders();

    // Reset navX heading on new thread when robot starts
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions(), getPose());

  }

  // Reset gyro heading 
  public void zeroHeading() {
    gyro.reset();
  }

  // Return heading in -180* to 180* format
  public double getHeading(){
    return gyro.getAngle();
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

  // Return robot position caculated buy odometer
  public Pose2d getPose(){
    //return odometry.getPoseMeters();
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  // Reset odometer to new location
  public void resetOdometry(Pose2d pose){
   // odometer.resetPosition(pose, getRotation2d());
   //odometer.resetPosition(getRotation2d(),getModulePositions(), pose);
   swerveDrivePoseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
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
  

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setModuleAngles(double[] degrees){
    frontLeft.setDesiredAngleDegrees(degrees[0]);
    frontRight.setDesiredAngleDegrees(degrees[1]);
    backLeft.setDesiredAngleDegrees(degrees[2]);
    backRight.setDesiredAngleDegrees(degrees[3]);
  }




  // Periodic looooooop
  @Override
  public void periodic(){

    // Periodicly update odometer for it to caculate position
    //odometer.update(getRotation2d(), getModulePositions());
    swerveDrivePoseEstimator.update(gyro.getRotation2d(), getModulePositions());
    

    // Odometry
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
    
    // Update robot monitor
    //monitor.update();

    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();

  }




}


 

