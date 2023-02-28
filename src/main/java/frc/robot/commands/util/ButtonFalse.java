// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ButtonFalse extends CommandBase {

  private Boolean finished = false;
  private XboxController xbox;

  public ButtonFalse(XboxController xbox){
    this.xbox = xbox;
    finished = false;
  }

  @Override
  public void initialize(){
    finished = false;

  }

  @Override
  public void execute(){
    System.out.print(xbox.getRawButton(5));

    if(xbox.getRawButton(5)){
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finished;
  }
}