// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Rumble extends SubsystemBase{

    private XboxController xbox;
    GrabberSubsystem grabberSubsystem;

    // Class constructor takes: joystick ID
    public Rumble(XboxController xbox, GrabberSubsystem grabberSubsystem){
        this.xbox = xbox;
        this.grabberSubsystem = grabberSubsystem;
    }

    @Override
    public void periodic() {

        if(grabberSubsystem.intakeMotor.getAppliedOutput() > 0.12){
            xbox.setRumble(RumbleType.kBothRumble, 0.65);
        }else{ 
            xbox.setRumble(RumbleType.kBothRumble, 0);
        }

    }


}
