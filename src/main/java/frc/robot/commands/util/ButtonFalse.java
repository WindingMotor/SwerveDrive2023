// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ButtonFalse extends CommandBase {

  private boolean finished = false;
  private ButtonSubsystem btn;

  public ButtonFalse(ButtonSubsystem btn){
    addRequirements(btn);
    this.btn = btn;
  }

  @Override
  public void initialize(){
    finished = false;

  }

  @Override
  public void execute(){
    if(btn.isButtonNinePressed()){
      finished = false;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finished;
  }
}