// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.grabber.angle;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GrabberIntakeManual extends CommandBase {

  private GrabberSubsystem subsystem;
  private double speed;

  public GrabberIntakeManual(GrabberSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(subsystem);
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.setIntakeSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
