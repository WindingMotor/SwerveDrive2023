// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveJoystick extends CommandBase {

  // Create empty variables for reassignment
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, headingFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  private double initialHeading;
  private PIDController thetaController;

  // Command constructor and requirements 
  public SwerveJoystick(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
  Supplier<Boolean> fieldOrientedFunction, Supplier<Double> headingFunction) {

    // Assign empty variables values passed from constructor and requirements
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.headingFunction = headingFunction;

    // Set the inital heading to the navx +||-inf heading. Should be zero on startup!
    this.initialHeading = headingFunction.get();
    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Set PID values for thetaPID
    thetaController = new PIDController(DriveConstants.kPThetaController, DriveConstants.kIThetaController, DriveConstants.kDThetaController);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  // Running loop of command
  @Override
  public void execute(){

    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get() * -1;

    // Apply deadband to protect motors
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    // Apply slew rate to joystick input to make robot input smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // Get the turning PID P value from smart dashboard
    SmartDashboard.putNumber("Turning P", DriveConstants.kPThetaController);
    double smartDashboardP = SmartDashboard.getNumber("Turning P", DriveConstants.kPThetaController);

    // Set P value of the turning PID to smart dashboard value
    thetaController.setP(smartDashboardP);

    // Changes the joystick +||-inf heading angle by adding or subtracting the turning speed
    // Starts at 0 and goes between -inf and +inf depending on rotation
    initialHeading += turningSpeed;

    /* Caculate the desired turning speed using the navx +||-inf heading 
      and the turningspeed +||-inf heading */
    turningSpeed = thetaController.calculate(headingFunction.get(), initialHeading);

    // Put initalHeading on smartdashboard
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    SmartDashboard.putNumber("Inital Heading", initialHeading);
    

    

    // Apply a dead band to not stress out motors
    //turningSpeed = Math.abs(headingFunction.get() - initialHeading) > 0.1 ? turningSpeed : 0.0;

    /* 
    SmartDashboard.putNumber("TURN-S", turningSpeed);
    // Update heading based off changes
    initialHeading += turningSpeed;
    SmartDashboard.putNumber("INIT-H", initialHeading);
    initialHeading = Math.IEEEremainder(initialHeading, 360);
    SmartDashboard.putNumber("INIT-H-REMAIN", initialHeading);

    SmartDashboard.putNumber("PID-O", tunringPidController.calculate(headingFunction.get(), initialHeading));


    // Wpilib PID, takes in current heading and heading to be at
    turningSpeed = tunringPidController.calculate(headingFunction.get(), initialHeading);

    // Apply deadband for motors
    //turningSpeed = Math.abs(headingFunction.get() - initialHeading) > 0.05 ? turningSpeed : 0.0;

    // Limit turning speed
    if (turningSpeed > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    } else if (turningSpeed < -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond){
      turningSpeed = -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
*/

    /* 
    // Test heading control, throws away previous turning values
    initialHeading += turningSpeed;

    turningSpeed = (headingFunction.get() - initialHeading) * turningPValue;
    // Deadband
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;
    // Limit turning rat
    if (turningSpeed > DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond)
    {
      turningSpeed = DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    else if (turningSpeed < -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond)
    {
      turningSpeed = -DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    }
    */

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
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

}
