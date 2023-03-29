// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.UltrasonicRangefinder;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveJoystick extends CommandBase {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, headingFunction;
  private final Supplier<Boolean> fieldOrientedFunction, ultrasonicMode;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private final Supplier<Boolean> flickFunction;
  private double initialHeading;
  private PIDController thetaController;
  UltrasonicRangefinder ultrasonic;
  
  private SendableChooser<Double> autoChooser = new SendableChooser<>();
 
  private double added;
  private int counter;

  // Command constructor
  public SwerveJoystick(SwerveSubsystem swerveSubsystem, UltrasonicRangefinder ultrasonic,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Double> headingFunction, Supplier<Boolean> flickFunction,
  Supplier<Boolean> ultrasonicMode){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.headingFunction = headingFunction;
    this.flickFunction = flickFunction;
    this.ultrasonicMode = ultrasonicMode;
    this.ultrasonic = ultrasonic;

    // Set the inital heading to the navx +||-inf heading. Should be zero on startup!
    this.initialHeading = headingFunction.get();

    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Set default PID values for thetaPID
    thetaController = new PIDController(DriveConstants.kPThetaController, DriveConstants.kIThetaController, DriveConstants.kDThetaController);

    // Counting system for flick function
    added = 0;
    counter = 0;

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    //swerveSubsystem.resetYaw();
    initialHeading = headingFunction.get();
    added = 0;
   // counter = 0;

  }

  // Running loop of command
  @Override
  public void execute(){

    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get() * 8;

    // Apply deadband to protect motors
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    // Apply slew rate to joystick input to make robot input smoother and mulitply by max speed
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    //turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // Test heading control, throws away previous turning values
    initialHeading += turningSpeed;

    // Get current navx heading
    double newHeading = headingFunction.get();

    // Offset navx heading to trick PID, makes us instantly rotate
    if(flickFunction.get()){
      counter += 1;
      if(counter > 6){
        added += 90;
        counter = 0;
      }
    }else{counter = 0;}

    turningSpeed = thetaController.calculate(newHeading - added, initialHeading) * 100;
    //turningSpeed = (headingFunction.get() - initialHeading) * turningPValue;
    
    // Turning deadband 
    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;
    turningSpeed *= -1;

    // Limit turning rat
    if (turningSpeed > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    else if (turningSpeed < -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    
    // Smartdashboard update
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    SmartDashboard.putNumber("Inital Heading", initialHeading);
    SmartDashboard.putNumber("NavX Heading", headingFunction.get());

    if(ultrasonicMode.get()){
      SmartDashboard.putBoolean("Ultrasonic Loading Station", ultrasonic.isLoadingWall());
      if(ultrasonic.isLoadingWall()){ }
      ySpeed = 0;

    }

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(swerveSubsystem.getRobotDegrees()));
    //chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);

    // Put field oriented value on smart dashboard
    // SmartDashboard.putBoolean("Field Oriented: ", fieldOrientedFunction.get());

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
