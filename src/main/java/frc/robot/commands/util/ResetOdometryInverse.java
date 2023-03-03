// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetOdometryInverse extends CommandBase {

  // Unassigned private instance variables
  private SwerveSubsystem swerveSubsystem;

  // Command constructor
  public ResetOdometryInverse(SwerveSubsystem swerveSubsystem){
    this.swerveSubsystem = swerveSubsystem;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() {swerveSubsystem.resetOdometry(new Pose2d(0.0,0.0, Rotation2d.fromDegrees(180)));}

  // Stop command once it starts 
  @Override
  public boolean isFinished(){return true;}

}