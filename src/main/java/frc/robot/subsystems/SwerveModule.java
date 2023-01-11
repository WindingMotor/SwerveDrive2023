// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;


public class SwerveModule extends SubsystemBase {
 
  // Create empty variables for reassignment
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;
  private SparkMaxPIDController builtinTurningPidController;

  private final DutyCycleEncoder absoluteEncoder;

  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;

  private String moduleName;

  // Special UI variables for swerve simulation
  private MechanismLigament2d simTurn;
  private MechanismLigament2d simDirection;

  private MechanismLigament2d simTurn2;
  private MechanismLigament2d simDirection2;


  // Class constructor where we assign default values for variable
   public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoLuteEncoderReversed, String name) {

    // Set offsets for absolute encoder in RADIANS!!!!!
    absoluteEncoderOffsetRad = absoluteEncoderOffset;
    absoluteEncoderReversed = absoLuteEncoderReversed;

    moduleName = name;

    // Create absolute encoder
    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);

    // Set duty cycle range of encoder of ABE encoder
    absoluteEncoder.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

    // Create drive and turning motor
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    // Set reverse state of drive and turning motor
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    // Set drive and turning motor encoder values
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    // Change drive motor conversion factors
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // Change conversion factors for neo turning encoder - should be in radians!
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    //-----ROBO-RIO-PID-----//

    // Create PID controller on ROBO RIO
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);

    // Tell PID controller that it is a *wheel*
    turningPidController.enableContinuousInput(0, 2*Math.PI);

    //-----SPARK-MAX-PID-----//

    builtinTurningPidController = turningMotor.getPIDController();

    // Set PID values for the simulated Spark max PID
    builtinTurningPidController.setP(ModuleConstants.kPTurning);
    builtinTurningPidController.setI(ModuleConstants.kITurning);
    builtinTurningPidController.setD(ModuleConstants.kDTurning);
    builtinTurningPidController.setIZone(0.0);
    builtinTurningPidController.setFF(0.0);
    builtinTurningPidController.setOutputRange(-1, 1);
    turningMotor.burnFlash();

    // Call resetEncoders
    resetEncoders();

    // Thanks to Alec for this code! 
    //>-----------S-I-M------------<//

    // Create the mechanism 2d canvas and get the root
    Mechanism2d mod = new Mechanism2d(6,6);
    MechanismRoot2d root = mod.getRoot("climber", 3, 3);

    // Add simTurn to the root, add direction to turn, then add it to smart dashboard
    simTurn = root.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirection = simTurn.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName+" commanded Turn", mod);

    //------------//

    // Do the same thing but for the real module state
    Mechanism2d mod2 = new Mechanism2d(6,6);
    MechanismRoot2d root2 = mod2.getRoot("climber2", 3, 3);

    simTurn2 = root2.append(new MechanismLigament2d("Swerve Turn", 2, 1.75));
    simDirection2 = simTurn2.append(new MechanismLigament2d("Wheel direction", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData(moduleName+"  real Turn", mod);

    //>-------------------------------<//

  }

  public void update(){

    // BUG WITH THESE 3 LINES, BREAKS SWERVE MODULE CODE
    SmartDashboard.putNumber(moduleName + "Absolute-Position", absoluteEncoder.getAbsolutePosition());
    //SmartDashboard.putNumber(moduleName + "Radians-Raw" , absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI);
    //SmartDashboard.putNumber(moduleName + "Radians", getAbsoluteEncoderRad());

    //SmartDashboard.putNumber(moduleName + " Drive Position", getDrivePosition());
    SmartDashboard.putNumber(moduleName + " Turning Position", getTurningPosition());

    //SmartDashboard.putNumber(moduleName + " Drive Velocity", getDriveVelocity());
    //SmartDashboard.putNumber(moduleName + " Turning Velocity", getTurningVelocity());

  }

  // Helpful get methods
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
      return turningEncoder.getPosition();
    }

  public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

  public double getTurningVelocity() {
      return turningEncoder.getVelocity();
    }
    
  public SwerveModulePosition getPosition(){
    return( new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad())));
  }

  /* Convert absolute value of the encoder to radians and then subtract the radian offset
  then check if the encoder is reversed.*/
  public double getAbsoluteEncoderRad(){

    //  Make angle variable
    double angle;

    // Get encoder absolute position goes from 1 to 0
    angle = absoluteEncoder.getAbsolutePosition();

    // Convert into radians
    angle *= 2.0 * Math.PI;

    // Apply magnetic offsets in radians
    angle -= absoluteEncoderOffsetRad;

    /*
    if(angle < 0){
      angle = 2.0 * Math.PI + angle ;
    } 
    */

    angle = Math.abs(angle);

    // Make negative if set
    angle *= ( absoluteEncoderReversed ? -1.0 : 1.0);
    
    // Report setting to driver station
    //DriverStation.reportError(moduleName + " called getAbsoluteEncoderRad: " + angle + "  " + absoluteEncoderOffsetRad, true);

    // Return angle in radians for neo turning motor encoder
    return angle;
    
  }

  // Set turning encoder to match absolute encoder value with gear offsets applied
  public void resetEncoders(){
    driveEncoder.setPosition(0);
    REVLibError error = turningEncoder.setPosition(getAbsoluteEncoderRad());
    //DriverStation.reportError("RESET ENCODER" + getAbsoluteEncoderRad() + " ", true);
    if(error.value != 0){
      //DriverStation.reportError(moduleName + " reset encoders error!: " + error.value, true);
    }
    else if(error.value == 0){
      //DriverStation.reportWarning(moduleName + " reset encoders has been ran without errors: " + getAbsoluteEncoderRad(), true);
    }
  }

  // Get swerve module current state, aka velocity and wheel rotation
  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    
    // Check if new command has high driving power 
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }

    // Optimize swerve module state to do fastest rotation movement, aka never rotate more than 90*
   state = SwerveModuleState.optimize(state, getState().angle);

    // Scale velocity down using robot max speed
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    // Use PID to calculate angle setpoint
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    //turningMotor.set(builtinTurningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    

    simTurn.setAngle(state.angle); // .plus(Rotation2d.fromDegrees(90))
    simDirection.setAngle(state.speedMetersPerSecond>0? 0:180);

    simTurn2.setAngle(absoluteEncoder.getAbsolutePosition()); // +90
    simDirection2.setAngle(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond >0 ? 0:180);
    SmartDashboard.putString("Swerve["+moduleName+"] state", state.toString());

  }

  // Stop all motors on module 
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }


  // Motor and SparkMax methods for Monitor 
  public double[] getMotorsCurrent(){
    return(new double[]{driveMotor.getOutputCurrent(),turningMotor.getOutputCurrent()});
  }

  public double[] getMotorsTemp(){
    return(new double[]{driveMotor.getMotorTemperature(),turningMotor.getMotorTemperature()});
  }

  public void setSmartCurrentLimiter(int driveLimit, int turningLimit){
    driveMotor.setSmartCurrentLimit(driveLimit);
    turningMotor.setSmartCurrentLimit(driveLimit);
  }



}
