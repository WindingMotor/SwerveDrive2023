// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorApriltagSelect extends CommandBase {

  private ElevatorSubsystem subsystem;
  private double targetHeight;
  private double targetID;

  public ElevatorApriltagSelect(ElevatorSubsystem subsystem, Double targetHeight, int targetID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.targetHeight = targetHeight;
    this.targetID = targetID;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.stopElevator();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

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
