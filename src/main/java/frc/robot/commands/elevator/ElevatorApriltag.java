// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorApriltag extends CommandBase {

  
  private ElevatorSubsystem subsystem;
  private VisionSubsystem visionSubsystem;
  
  private double targetHeight;
  private static final double cameraHeightMeters = 2;
  private boolean finished = false;

  public ElevatorApriltag(ElevatorSubsystem subsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    this.targetHeight = targetHeight;
    this.visionSubsystem = visionSubsystem;
    addRequirements(subsystem);
    finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.stopElevator();
    //DriverStation.reportWarning("ELEV-APRIL STARTED", true);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(visionSubsystem.hasTargets()){
      //DriverStation.reportWarning("ELEV-APRIL: " + visionSubsystem.getTargetTransformHeight(), true);
      //subsystem.setElevatorMeters(visionSubsystem.getTargetTransformHeight());
    }else{
     // DriverStation.reportWarning("ELEV-APRIL NO-TARGETS", true);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("ELEV-APRIL STOPPED", true);
    subsystem.stopElevator();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
  
}
