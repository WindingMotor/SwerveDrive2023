// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.swerve;
import frc.robot.commands.util.CalibrateGyro;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LightStrip;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class SwerveReset extends SequentialCommandGroup {
 
  public SwerveReset(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(new CalibrateGyro(swerveSubsystem),
    new ResetYaw(swerveSubsystem),
    new ResetOdometry(swerveSubsystem, new Pose2d()));
  }

}
