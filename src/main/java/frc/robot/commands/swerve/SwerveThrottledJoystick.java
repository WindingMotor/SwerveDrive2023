// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveThrottledJoystick extends CommandBase {

  //-----------------------------//
  // TO BE DEPRECATED AND REWRITTEN!
  //-----------------------------//

  /*

  // Create empty variables for reassignment
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, throttleFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  //private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  // Command constructor and requirements 
  public SwerveThrottledJoystick(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,  
  Supplier<Double> throttleFunction, Supplier<Boolean> fieldOrientedFunction) {

    // Assign empty variables values passed from constructor and requirements
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.throttleFunction = throttleFunction;

    // Slew rate limiter
    
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  // Running loop of command
  @Override
  public void execute(){

    // Set joystick inputs to speed variables but multiply throttle value
    double throttleSpeed = throttleFunction.get();
    double xSpeed = xSpdFunction.get() * throttleSpeed;
    double ySpeed = ySpdFunction.get() * throttleSpeed;
    double turningSpeed = turningSpdFunction.get() * throttleSpeed;

    // Apply deadband to protect motors
    //xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    //ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    //turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    // Apply slew rate to joystick input to make robot input smoother
    //xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    //ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    //turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // Apply field oriented mode
    ChassisSpeeds chassisSpeeds;

    
    if(fieldOrientedFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    }
    // Apply non-field oriented mode
    else{
      chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
    }
    

    chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);

    // Put field oriented value on smart dashboard
    SmartDashboard.putBoolean("Field Oriented: ", fieldOrientedFunction.get());

    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
    // Set each module state
    swerveSubsystem.setModuleStates(moduleStates);
  }
  
  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules();}

  @Override
  public boolean isFinished(){return false;}


*/

  //-----------------------------//
  // TO BE DEPRECATED AND REWRITTEN!
  //-----------------------------//

}
