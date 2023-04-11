// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorHome extends CommandBase {

  private ElevatorSubsystem subsystem;
  private boolean finished = false;

  public ElevatorHome(ElevatorSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;

    addRequirements(subsystem);
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.reportWarning("Homing elevator!", true);
    finished = false;
    // Home the elevator and return true when finished
   // finished = subsystem.homeElevatorBottom();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stopElevator();
    DriverStation.reportWarning("Elevator homing finished!", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
