// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Leds;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.IOConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAutoAlign extends CommandBase {

  
  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Leds leds;
  
  private PIDController thetaController;
  private boolean finished;
  private double angleTolerance;

  private final SlewRateLimiter xLimiter, yLimiter;
  private final Supplier<Double> xSpdFunction, ySpdFunction;
  private final Supplier<Boolean> override;

  // Command constructor
  public SwerveAutoAlign(SwerveSubsystem swerveSubsystem,Leds leds, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Boolean> override){

    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.override = override;

    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    finished = false;

    // Ending command tolerance
    angleTolerance = 1.0;

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.leds = leds;

    // Set default PID values for thetaPID
    thetaController = new PIDController(0.11, 0.004, 0.02);

    // Enable continuous input for thetaPID
    thetaController.enableContinuousInput(0, 360);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);

  }

  @Override
  public void initialize() {
    // Set default command values
    finished = false;
    leds.setAutoAlign(true);
  }

  // Running loop of command
  @Override
  public void execute(){

    if(override.get() == false){
      finished = true;
    }

    double angle;
    if(DriverStation.getAlliance() == Alliance.Red){
       angle = -90;
    }else{
      angle = 90;
    }

    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();

    // Apply deadband to protect motors
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;

    // Apply slew rate to joystick input to make robot input smoother and mulitply by max speed
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

    /* 
    if(swerveSubsystem.getRobotDegrees() > angle - angleTolerance){
      if(swerveSubsystem.getRobotDegrees() < angle + angleTolerance){
        finished = true;
      }
    }
    */

    // Get current navx heading
    double turningSpeed;

    turningSpeed = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), angle);
    SmartDashboard.putNumber("ROT CACL", turningSpeed);
    SmartDashboard.putNumber("ROBO DEG", swerveSubsystem.getRobotDegrees());
    
    // Turning motor deadband 
    turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

    // Smartdashboard update
    SmartDashboard.putNumber("Turning Speed", turningSpeed);
    
    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

    // Create module states using array
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
 
    // Set the module state
    swerveSubsystem.setModuleStates(moduleStates);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules(); leds.setAutoAlign(false);}

  @Override
  public boolean isFinished(){return finished;}

  
}
