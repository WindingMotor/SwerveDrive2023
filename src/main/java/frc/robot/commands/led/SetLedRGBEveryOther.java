// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.led;
import frc.robot.util.LightStrip;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetLedRGBEveryOther extends CommandBase {

  private LightStrip ledStrip;
  private int[] rgb1 = {0,0,0};
  private int[] rgb2 = {0,0,0};

  public SetLedRGBEveryOther(LightStrip ledStrip, int r1, int g1, int b1, int r2, int g2, int b2) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledStrip);
    this.ledStrip = ledStrip;

    rgb1[0] = r1;
    rgb1[1] = g1;
    rgb1[2] = b1;

    rgb2[0] = r2;
    rgb2[1] = g2;
    rgb2[2] = b2;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledStrip.setStripColorEveryOther(rgb1[0], rgb1[1], rgb1[2], rgb2[0], rgb2[1], rgb2[2]);
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
