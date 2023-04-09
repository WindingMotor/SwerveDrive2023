// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Rumble extends SubsystemBase{

    // Joystick object
    // My Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2
    // My Transmitter max and min values: 0.700 and -0.700
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
