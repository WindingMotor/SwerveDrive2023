// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorApriltag extends CommandBase {

  private ElevatorSubsystem subsystem;
  private double targetHeight;
  private static final double cameraHeightMeters = 2;

  public ElevatorApriltag(ElevatorSubsystem subsystem, Double targetHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.targetHeight = targetHeight;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.stopElevator();
    DriverStation.reportWarning("ELEV-APRIL STARTED: " + targetHeight, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriverStation.reportWarning("ELEV-APRIL: " + targetHeight, true);
    subsystem.setElevatorMeters(targetHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("ELEV-APRIL STOPPED: " + targetHeight, true);
    subsystem.stopElevator();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
