// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorMeters extends CommandBase {

  private ElevatorSubsystem subsystem;
  private double meters;

  public ElevatorMeters(ElevatorSubsystem subsystem, Double meters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.meters = meters;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the elevator to the desired meters
    subsystem.setElevatorMeters(meters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
