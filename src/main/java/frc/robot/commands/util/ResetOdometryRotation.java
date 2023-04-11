// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetOdometryRotation extends CommandBase {

  // Unassigned private instance variables
  private Pose2d pose;
  private SwerveSubsystem swerveSubsystem;
  private Rotation2d rot;

  // Command constructor
  public ResetOdometryRotation(SwerveSubsystem swerveSubsystem, Pose2d pose, Rotation2d rot){
    this.swerveSubsystem = swerveSubsystem;
    this.pose = pose;
    this.rot = rot;
  }

  // Reset robot position when command starts
  @Override
  public void initialize() {
    swerveSubsystem.resetOdometry(pose, rot);
    SmartDashboard.putString("POSE OUTPUT", pose.toString());}

  // Stop command once it starts 
  @Override
  public boolean isFinished(){return true;}

}