// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlignBasic extends CommandBase {

  private boolean finished;
  SwerveSubsystem swerveSubsystem;
  VisionSubsystem visionSubsystem;
  Supplier<Boolean> switchOverride;
  Supplier<Double> headingFunction, setpointFunction;

  private PIDController vXController, vYController;
  private PIDController thetaController;

  private double vX;
  private double vY;
  private double vT;

  private double initalHeading;

  // Command constructor
  public SwerveAlignBasic(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
   Supplier<Double> headingFunction, Supplier<Boolean> switchOverride, Supplier<Double> setpointFunction){

    addRequirements(swerveSubsystem, visionSubsystem);

    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.switchOverride = switchOverride;
    this.headingFunction = headingFunction;
    this.setpointFunction = setpointFunction;

    initalHeading = headingFunction.get();

    vXController = new PIDController(0.8, 0.005, 0);
    vYController = new PIDController(1.4, 0.005, 0);
    thetaController = new PIDController(0.0009, 0, 0);

    vX = 0;
    vY = 0;
    vT = 0;

    finished = false;

  }

  @Override
  public void initialize() {
    finished = false;
    initalHeading = headingFunction.get();
  }

  @Override
  public void execute(){

    if(!visionSubsystem.hasTargets()){
      //finished = true;
      vX = vX * 0.5;
      vY = vY * 0.5;
      vT = vT * 0.5;
    }else{
      if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
        vX = vXController.calculate(visionSubsystem.getTargetTransform().getX(), 1); // X-Axis PID
        vY = -vYController.calculate(visionSubsystem.getTargetTransform().getY(), 0); // Y-Axis PID
        vT = thetaController.calculate(swerveSubsystem.getGyroDegrees(), 90) * 300; // Rotation PID
        //vT = -Math.abs(vT) > 0.05 ? vT : 0.0; // Deadband
      }else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue){
        vX = -vXController.calculate(visionSubsystem.getTargetTransform().getX(), 1); // X-Axis PID
        vY = vYController.calculate(visionSubsystem.getTargetTransform().getY(), 0); // Y-Axis PID
        vT = thetaController.calculate(swerveSubsystem.getGyroDegrees(), 90) * 300; // Rotation PID
      }
      SmartDashboard.putNumber("VT", vT);
    }

    if(switchOverride.get() == false){
      finished = true;
    }

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    // Apply chassis speeds with desired velocities
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vT, swerveSubsystem.getRotation2d());
    //chassisSpeeds = new ChassisSpeeds(vX, vY, vT);
    //chassisSpeeds = new ChassisSpeeds(0.5,0,0);

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
