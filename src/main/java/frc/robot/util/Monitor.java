// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.util;
public class Monitor {


/* 
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
    
        double averageDriveMotorTemp = 0;
        double averageTurnMotorTemp = 0;
        
        // Check drive motor temps
        frontLeft.getMotorsTemp()[0] += averageDriveMotorTemp;
        frontRight.getMotorsTemp()[0] += averageDriveMotorTemp;
        backLeft.getMotorsTemp()[0]  += averageDriveMotorTemp;
        backRight.getMotorsTemp()[0] += averageDriveMotorTemp;

        // Check turning motor temps
        frontLeft.getMotorsTemp()[1] += averageTurnMotorTemp;
        frontRight.getMotorsTemp()[1] += averageTurnMotorTemp;
        backLeft.getMotorsTemp()[1] += averageTurnMotorTemp;
        backRight.getMotorsTemp()[1] += averageTurnMotorTemp;

        averageDriveMotorTemp /= 4;
        averageTurnMotorTemp /= 4;

        SmartDashboard.putNumber("Drive Motor Average C*", averageDriveMotorTemp);
        SmartDashboard.putNumber("Turn Motor Average C*", averageTurnMotorTemp);


    }
*/
}


