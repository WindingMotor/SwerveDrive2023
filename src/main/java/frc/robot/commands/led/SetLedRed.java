// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.led;
import frc.robot.util.LightStrip;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetLedRed extends CommandBase {

  private LightStrip ledStrip;

  public SetLedRed(LightStrip ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledStrip);
    this.ledStrip = ledStrip;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledStrip.setRed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
