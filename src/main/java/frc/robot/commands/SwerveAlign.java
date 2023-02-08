// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants.DriveConstants;

import java.util.ArrayList;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveAlign extends CommandBase {

  // Create variables

  private boolean finished = false;

  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final Supplier<Double> headingFunction;

  private final Supplier<Boolean> switchOverride;

  private PIDController xController;
  private PIDController yController;

  private double vX;
  private double vY;

  private double xSetpoint;
  private double ySetpoint;

  // Command constructor
  public SwerveAlign(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem,
   Supplier<Double> headingFunction, Supplier<Boolean> switchOverride){

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.headingFunction = headingFunction;
    this.switchOverride = switchOverride;

    // Assign PID objects
    xController = new PIDController(0.01, 0, 0);
    yController = new PIDController(0.01, 0, 0);

    vX = 0;
    vY = 0;

    xSetpoint = 1;
    ySetpoint = 0;

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem, visionSubsystem);
  }

  // Running
  @Override
  public void execute(){

    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;

    // Check if targets are found
    if(visionSubsystem.hasTargets()){
       // Caculate velocity with x and y distance from target
      vX = xController.calculate(visionSubsystem.getTargetTransform().getX(), xSetpoint);
      vY = yController.calculate(visionSubsystem.getTargetTransform().getY(), ySetpoint);
    }else{
      // Set velocity to 0 and end the command if no target is found 
      vX = 0;
      vY = 0;
      finished = true;
    }

    // Stop command if switch goes to false
    if(!switchOverride.get()){
      vX = 0;
      vY = 0;
      finished = true;
    }

    // Deadband for motors
    vX =  Math.abs(vX) > 0.05 ? vX : 0.0;
    vY =  Math.abs(vY) > 0.05 ? vY : 0.0;

    // Apply chassis speeds with desired velocities
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, 0, swerveSubsystem.getRotation2d());
    //chassisSpeeds = new ChassisSpeeds(vX,vY,0);

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
