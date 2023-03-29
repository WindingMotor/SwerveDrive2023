// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.UltrasonicRangefinder;
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

  // Ultrasonic sensor
  UltrasonicRangefinder ultrasonic;
  
  // Returns positions of the swerve modules for odometry
  public SwerveModulePosition[] getModulePositions(){

    return( new SwerveModulePosition[]{
      frontLeft.getPosition(), 
      frontRight.getPosition(), 
      backLeft.getPosition(),
      backRight.getPosition()});

  }

  // Create odometer for swerve drive
  private SwerveDriveOdometry odometer;

  // Swerve subsystem constructor
  public SwerveSubsystem() {

    // Reset robot encoders on startup
    resetAllEncoders();

    // Zero navX heading on new thread when robot starts
    new Thread(() -> {
        try {
            Thread.sleep(1000);
            gyro.calibrate();
            zeroHeading();
        } catch (Exception e) {
        }
    }).start();

    // Set robot odometry object to current robot heading and swerve module positions
    odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
    getOdometryAngle(), getModulePositions());
    // new Rotation2d(gyro.getYaw() * -1 / 180 * Math.PI), getModulePositions()

  }

  // Reset gyro heading 
  public void zeroHeading() {
    gyro.reset();
  }

  // Reset gyro yaw
  public void resetYaw(){
    gyro.zeroYaw();
  }

  public void calibrateGyro(){
    gyro.calibrate();
  }

  // Return gyro heading, make sure to read navx docs on this
  public double getHeading(){
    return gyro.getAngle();
  }

  // Return the robot odometry in pose meters
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

  // Move the swerve modules to the desired SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // Make sure robot rotation is all ways possible by changing other module roation speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
}

  // Return robot position caculated by odometer
  public Pose2d getPose(){
    return odometer.getPoseMeters();
  }

  // Reset odometer to new Pose2d location
  public void resetOdometry(Pose2d pose){
   odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
  }

  // Reset odometer to new Pose2d location but with roation
  public void resetOdometry(Pose2d pose, Rotation2d rot){
    odometer.resetPosition(rot, getModulePositions(), pose);
  }

  // Return an angle from -180 to 180 for robot odometry
  // The commented out method is for if the gyroscope is reversed direction
  public Rotation2d getOdometryAngle(){
    /* 
    double angle = -gyro.getYaw() + 180;
    if(angle > 180){
      angle -= 360;
    }else if(angle < -180){
      angle += 360;
    }
    return Rotation2d.fromDegrees(angle);
    */
    return(Rotation2d.fromDegrees(gyro.getYaw()));
  }

  // Returns an angle from 0 to 360 that is continuous, meaning it loops 
  public double getRobotDegrees(){
    double rawValue = gyro.getAngle() % 360.0;
    if(rawValue < 0.0){
      return(rawValue + 360.0);
    }else{
      return(rawValue);
    }
  }

  // Reset all swerve module encoders
  public void resetAllEncoders(){
      frontLeft.resetEncoders();
      frontRight.resetEncoders();
      backLeft.resetEncoders();
      backRight.resetEncoders();
  }

  // Periodic looooooop
  @Override
  public void periodic(){

    // Update odometer for it to caculate robot position
    odometer.update(getOdometryAngle(), getModulePositions());

   // Put odometry data on smartdashboard
   SmartDashboard.putNumber("Heading", getHeading());
   SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
   SmartDashboard.putNumber("ROBOT DEGREES NAVX", getRobotDegrees());
   SmartDashboard.putString("ODOMETRY", odometer.getPoseMeters().toString());
   SmartDashboard.putString("Raw R2d ROBOT DEG", getOdometryAngle().toString());
    
  SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
  SmartDashboard.putBoolean("Magnetic Issues", gyro.isMagneticDisturbance());
  SmartDashboard.putBoolean("Magnetic Calibartion", gyro.isMagnetometerCalibrated());


   // Update smartdashboard data for each swerve module object
    frontLeft.update();
    frontRight.update();
    backLeft.update();
    backRight.update();

  }

}


 

