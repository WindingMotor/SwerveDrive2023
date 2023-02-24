// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReportString extends CommandBase {

  private String string;

  public ReportString(String string){this.string = string;}

  @Override
  public void initialize() {DriverStation.reportWarning(string, true);}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}