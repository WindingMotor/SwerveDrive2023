// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Leds;
import frc.robot.util.LightStrip;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAutoBalance extends CommandBase {

  
  private boolean finished;

  SwerveSubsystem swerveSubsystem;

  Supplier<Double> headingFunction;

  private PIDController vYController;
  private PIDController thetaController;


  private double vY;


  private double xSetpoint;
  private double ySetpoint;

  private Leds leds;

  // Command constructor
  public SwerveAutoBalance(SwerveSubsystem swerveSubsystem,Supplier<Double> headingFunction, Leds leds){


    addRequirements(swerveSubsystem);

    this.leds = leds;
    this.swerveSubsystem = swerveSubsystem;
    this.headingFunction = headingFunction;

    vYController = new PIDController(6, 0.006, 0.008);

    thetaController = new PIDController(0.1, 0.004, 0.02);
    thetaController.enableContinuousInput(0, 360);


    vY = 0;

    finished = false;

  }

  @Override
  public void initialize() {
    finished = false;

  }

  @Override
  public void execute(){


      leds.setAutoBalance(false);

      // Turning PID
      double turningSpeed;
      turningSpeed = -thetaController.calculate(swerveSubsystem.getRobotDegrees(), 0.0);
      turningSpeed = Math.abs(turningSpeed) > 0.05 ? turningSpeed : 0.0;

      double currentRoll = swerveSubsystem.getRoll();

      if(currentRoll < 10.0 && currentRoll > -10.0){
        finished = true;
      }else if(currentRoll > 10.0){
        vY = 0.65;
      }else if(currentRoll < -10.0){
        vY = -0.65;
      }

      SmartDashboard.putNumber("vY", vY);

      // Create chassis speeds
      ChassisSpeeds chassisSpeeds;
  
      // Apply chassis speeds with desired velocities
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, vY, turningSpeed, swerveSubsystem.getRotation2d());

      // Create states array
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
      // Move swerve modules
      swerveSubsystem.setModuleStates(moduleStates);

  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted){swerveSubsystem.stopModules(); leds.setAutoBalance(true);}

  @Override
  public boolean isFinished(){return finished;}

  
}


























