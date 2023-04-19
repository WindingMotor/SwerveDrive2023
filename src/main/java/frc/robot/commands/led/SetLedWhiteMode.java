// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.led;
import frc.robot.util.Leds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetLedWhiteMode extends CommandBase {

  private Leds leds;
  private String string;

  public SetLedWhiteMode(Leds leds, String string) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
    this.string = string;
    this.leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(string.equals("strobe")){
      leds.setStrobe(true);
      leds.setWhite(false);
    }else if(string.equals("white")){
      leds.setWhite(true);
      leds.setStrobe(false);
    }
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
