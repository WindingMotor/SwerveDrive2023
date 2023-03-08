// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ButtonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private XboxController xbox;

  public ButtonSubsystem( XboxController xbox) {
    this.xbox = xbox;
  
  }

  public boolean isButtonNinePressed(){
    return(xbox.getRawButton(9));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
