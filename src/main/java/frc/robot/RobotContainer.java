// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.manuals.Forward2M;
import frc.robot.auto.routines.AutoOne;
import frc.robot.auto.routines.AutoThree;
import frc.robot.auto.routines.AutoTwo;
import frc.robot.auto.routines.TestRoutine;
import frc.robot.commands.elevator.ElevatorApriltag;
import frc.robot.commands.elevator.ElevatorDownSetpoint;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorJoystick;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorStop;
import frc.robot.commands.elevator.ElevatorUpSetpoint;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberStop;
import frc.robot.commands.intake.IntakeForward;
import frc.robot.commands.intake.IntakeHold;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.intake.IntakeStop;
import frc.robot.commands.routines.loading.ConeFloor;
import frc.robot.commands.routines.loading.ConePlatform;
import frc.robot.commands.routines.scoring.ConeBottom;
import frc.robot.commands.routines.scoring.ConeTop;
import frc.robot.commands.routines.util.ObjectLaunch;
import frc.robot.commands.routines.util.SetLedPurple;
import frc.robot.commands.routines.util.SetLedYellow;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.angle.GrabberTrigger;
import frc.robot.commands.swerve.SwerveAlignBasic;
import frc.robot.commands.swerve.SwerveJoystick;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.SwerveReset;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.LightStrip;
import frc.robot.util.Transmitter;
import frc.robot.util.UltrasonicRangefinder;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class RobotContainer {

  // Converted to 2023 wpiblib

  //------------------------------------O-B-J-E-C-T-S-----------------------------------//

  // Deprecated joysticks
  //private final Joystick leftJoystick = new Joystick(IOConstants.kLeftJoystick);
  //private final Joystick rightJoystick = new Joystick(IOConstants.kRightJoystick);

  // Create tx16s transmitter
  private final Joystick tx16s = new Joystick(4);
  private final CommandJoystick tx16sCOMD = new CommandJoystick(4);

  // Create led strips
  private final LightStrip strips = new LightStrip(tx16s);

  // Create ultrasonic sensor 
  private final UltrasonicRangefinder ultrasonic = new UltrasonicRangefinder(strips);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  // Create vision subsystem
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // Create grabber subsystem
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  // Create elevator subsystem
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // Create PID controllers for trajectory tracking
  public final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  //public final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
  public final ProfiledPIDController thetaController = new ProfiledPIDController(4, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);

  // Create a non profiled PID controller for path planner
  private final PIDController ppThetaController = new PIDController(0, 0, 0);

  // Create xbox controller
  //private final XboxController xbox = new XboxController(3);
  private final CommandXboxController xbox = new CommandXboxController(3);

  //private final ButtonSubsystem btn = new ButtonSubsystem(xbox);

  //--------------------------P-A-T-H-S----------------------------//

  // Manual path 
  //private final PathPlannerTrajectory pathOne = PathPlanner.loadPath("forward1M", new PathConstraints(0.25, 0.25)); 
//  private Command autoForward = new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true);

  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    CameraServer.startAutomaticCapture();
    PathPlannerServer.startServer(5811);
    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    // Joystick Numbers 0 = LEFT : 1 = RIGHT
    // Joystick Axises: 0 = left/right : 1 = forward/backwards : 2 = dial
    // OLD-> Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2

  //>--------------T-R-A-N-S-----------------//
    
/* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> tx16s.getRawAxis(0), // X-Axis
    () -> -tx16s.getRawAxis(1), // Y-Axis
    () -> tx16s.getRawAxis(3), // R-Axis
    () -> tx16s.getRawButton(0), // Field oriented -does nothing right now
    () -> swerveSubsystem.getHeading(), // Navx heading
    () -> tx16s.getRawButton(4))); // Flick offset button, should be toggle!
*/

swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem, ultrasonic,
() -> -tx16sCOMD.getRawAxis(0), // X-Axis
() -> tx16sCOMD.getRawAxis(1), // Y-Axis
() -> tx16sCOMD.getRawAxis(3), // R-Axis
() -> tx16s.getRawButton(2), // Field oriented -does nothing right now
() -> swerveSubsystem.getHeading(), // Navx heading
() -> falseFunct(), // Flick offset button, should be toggle!
() -> tx16s.getRawButton(4))); // Ultrasonic mode

  //elevatorSubsystem.setDefaultCommand(new ElevatorJoystick(elevatorSubsystem,
  //() -> xbox.getRawAxis(1)));

  //grabberSubsystem.setDefaultCommand(new GrabberTrigger(grabberSubsystem,
  //  () -> xbox.getRawAxis(3)));

  //>----------S-E-N-D-E-R----------<//

    // Run button binding method
    configureButtonBindings();

  }

  //------------------------------------D-E-B-U-G------------------------------------//

  private double zeroFunct(){return 0;}

  private boolean trueFunct(){return true;}

  private boolean falseFunct(){return false;}

  //------------------------------------B-U-T-T-O-N-S------------------------------------//

  // Create button bindings
  private void configureButtonBindings(){

    // ELEVATOR SOLENOID
    new JoystickButton(tx16s, 3).onTrue(new ElevatorSolenoid(elevatorSubsystem));

    // 4 Y - Garbber Solenoid
    xbox.y().onTrue(new GrabberSolenoid(grabberSubsystem));

    // 1 A - Grabber Intake Forward
    xbox.a().onTrue(new GrabberForward(grabberSubsystem));
    xbox.a().toggleOnFalse(new GrabberHold(grabberSubsystem));

    // 2 B - Grabber Intake Reverse
    xbox.b().onTrue(new GrabberReverse(grabberSubsystem));
    xbox.b().toggleOnFalse(new GrabberHold(grabberSubsystem));

    // 3 X - Elevator Zero
    xbox.x().onTrue(new ElevatorZero(elevatorSubsystem, grabberSubsystem));

    // 5 LB - Low Score
    xbox.leftBumper().onTrue(new ConeBottom(elevatorSubsystem, grabberSubsystem));

    // 6 RB - High Score
    xbox.rightBumper().onTrue(new ConeTop(elevatorSubsystem, grabberSubsystem));

    // 9 LJ - Loading station
    xbox.button(9).onTrue(new ConePlatform(elevatorSubsystem, grabberSubsystem));

    // 10 RJ - Grabber angle zero
    xbox.button(10).onTrue(new GrabberDegrees(grabberSubsystem, 0));

    // EMG RESET
   // tx16sCOMD.button(5).toggleOnTrue(new SwerveReset(swerveSubsystem));

    // 10 RJ - Reset Odometry
   // xbox.button(10).onTrue(new ResetOdometry(swerveSubsystem, new Pose2d()));
   // xbox.button(10).onTrue(new ResetOdometry(swerveSubsystem, new Pose2d()));

    // Manual grabber angle test code
   // xbox.axisGreaterThan(1, 0.55).onTrue(new GrabberTrigger(grabberSubsystem, () -> xbox.getRawAxis(1)));
    
    // D-Pad POV testing - elevator setpoint
    xbox.povUp().toggleOnTrue(new SetLedYellow(strips));
    xbox.povDown().toggleOnTrue(new SetLedPurple(strips));

    //tx16sCOMD.axisGreaterThan(1, 50.0).toggleOnTrue(new ElevatorZero(elevatorSubsystem, grabberSubsystem));

    // Homing
    //new JoystickButton(xbox, 1).onTrue(new ElevatorHome(elevatorSubsystem));
    // Apriltag
    // new JoystickButton(xbox, 2).onTrue(new ElevatorApriltag(elevatorSubsystem, visionSubsystem));
    // Meters
    // new JoystickButton(xbox, 3).onTrue(new ElevatorMeters(elevatorSubsystem, 1.0));

    //--------------// Auto Bindings

    // Apriltag
    new JoystickButton(tx16s, 8).onTrue(new SwerveAlignBasic(swerveSubsystem, visionSubsystem,
       () -> swerveSubsystem.getHeading(), () -> tx16s.getRawButton(8), () -> tx16s.getRawAxis(5)));
    
    // Run autonmous command during teleop
    //new JoystickButton(tx16s, 3).onTrue(new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true));
    //new JoystickButton(tx16s, 7).onTrue(new ElevatorManual(elevatorSubsystem, 0.5));
    //new JoystickButton(tx16s, 7).onFalse(new ElevatorStop(elevatorSubsystem));
  }

  //------------------------------------R-E-F-E-R-R-E-R-S------------------------------------//

    public void containerResetAllEncoders(){ swerveSubsystem.resetAllEncoders();}

  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  
  // Return the command to run during auto
  public Command getAutonomousCommand(){

  // PathPlannerTrajectory back = PathPlanner.loadPath("backwards0.5M", new PathConstraints(4, 2)); 
    // Command to run
      // Routine
   Command autoOne = new AutoOne(swerveSubsystem, elevatorSubsystem, grabberSubsystem, xController, yController, ppThetaController);
   Command autoTwo = new AutoTwo(swerveSubsystem, elevatorSubsystem, grabberSubsystem, xController, yController, ppThetaController);
   Command autoThree = new AutoThree(swerveSubsystem, elevatorSubsystem, grabberSubsystem, xController, yController, thetaController);
   // Command auto = swerveSubsystem.auto(back, true);

    return autoOne;
  }

}