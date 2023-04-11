// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Rumble extends SubsystemBase{

    private XboxController xbox;
    SwerveSubsystem swerveSubsystem;

    // Class constructor takes: joystick ID
    public Rumble(XboxController xbox, SwerveSubsystem swerveSubsystem){
        this.xbox = xbox;
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void periodic() {
        double amount = swerveSubsystem.getRumble();
        System.out.println(amount);
        amount = Math.abs(amount);
        if(amount > 1){amount = 1;}
        if(amount < 0){amount = 0;}
       // xbox.setRumble(RumbleType.kBothRumble, amount);
    }

}
