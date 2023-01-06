// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class Monitor {

    // Create PDP object
    PowerDistribution PDP = new PowerDistribution();

    // Class constructor
    public Monitor(){}

    public void updateGeneral(){

        double sysVoltage = PDP.getVoltage();
        double sysCurrent = PDP.getTotalCurrent();
        double sysTemperature = PDP.getTemperature();

        // Check voltage
        if(sysVoltage <= 9){DriverStation.reportError("CRIT VOLT: " + sysVoltage, true);}
        else if(sysVoltage <= 10){DriverStation.reportError("LOW VOLT: " + sysVoltage , true);}

        // Check current draw
        if(sysCurrent >= 115){DriverStation.reportError("ALERT! CURR DRAW: " + sysCurrent, true);}
        
        // Check temperature
        if(sysTemperature >= 100){DriverStation.reportWarning("ALERT! PDP TEMP: " + sysTemperature, true);}

        // Report data to smart dashboard
        SmartDashboard.putNumber("Voltage", sysVoltage);
        SmartDashboard.putNumber("Current", sysCurrent);
        SmartDashboard.putNumber("PDP Temp", sysTemperature);

    }

    public void updateDrive(SwerveSubsystem swerveSubsystem){

        // Set to legible names
        SwerveModule frontLeft = swerveSubsystem.getFrontLeft();
        SwerveModule frontRight = swerveSubsystem.getFrontRight();
        SwerveModule backLeft = swerveSubsystem.getBackLeft();
        SwerveModule backRight = swerveSubsystem.getBackRight();

        // Check motor temp in celsius. Defualt: 33C is 91.4F
    
        // Check drive motor temps
        if(frontLeft.getMotorsTemp()[0] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("FL DR TEMP!: " + frontLeft.getMotorsTemp()[0], true);}
        if(frontRight.getMotorsTemp()[0] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("FR DR TEMP!: " + frontRight.getMotorsTemp()[0], true);}
        if(backLeft.getMotorsTemp()[0] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("BL DR TEMP!: " + backLeft.getMotorsTemp()[0], true);}
        if(backRight.getMotorsTemp()[0] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("BR DR TEMP!: " + backRight.getMotorsTemp()[0], true);}

        // Check turning motor temps
        if(frontLeft.getMotorsTemp()[1] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("FL TU TEMP!: " + frontLeft.getMotorsTemp()[1], true);}
        if(frontRight.getMotorsTemp()[1] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("FR TU TEMP!: " + frontRight.getMotorsTemp()[1], true);}
        if(backLeft.getMotorsTemp()[1] >=  Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("BL TU TEMP!: " + backLeft.getMotorsTemp()[1], true);}
        if(backRight.getMotorsTemp()[1] >= Constants.DriveConstants.kMaxDriveMotorTemp){DriverStation.reportError("BR TU TEMP!: " + backRight.getMotorsTemp()[1], true);}



    }

}