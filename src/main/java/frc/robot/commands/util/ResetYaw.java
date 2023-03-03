// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetYaw extends CommandBase {

  // Unassigned private instance variables
  private SwerveSubsystem swerveSubsystem;

  // Command constructor
  public ResetYaw(SwerveSubsystem swerveSubsystem){
    this.swerveSubsystem = swerveSubsystem;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() {swerveSubsystem.resetYaw();}

  // Stop command once it starts 
  @Override
  public boolean isFinished(){return true;}

}