// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import javax.xml.namespace.QName;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.manuals.Forward2M;
import frc.robot.auto.routines.AutoOne;
import frc.robot.auto.routines.TestRoutine;
import frc.robot.commands.elevator.ElevatorApriltag;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.ElevatorJoystick;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorStop;
import frc.robot.commands.grabber.GrabberDegrees;
import frc.robot.commands.grabber.GrabberIntake;
import frc.robot.commands.grabber.GrabberIntakeReverse;
import frc.robot.commands.grabber.GrabberIntakeStop;
import frc.robot.commands.grabber.GrabberTrigger;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.swerve.SwerveAlignBasic;
import frc.robot.commands.swerve.SwerveJoystick;
import frc.robot.commands.swerve.SwerveRotator;
import frc.robot.commands.swerve.SwerveThrottledJoystick;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.LightStrip;
import frc.robot.util.Transmitter;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class RobotContainer {

  // Converted to 2023 wpiblib

  //------------------------------------O-B-J-E-C-T-S-----------------------------------//

  // Create joysticks
  //private final Joystick leftJoystick = new Joystick(IOConstants.kLeftJoystick);
  //private final Joystick rightJoystick = new Joystick(IOConstants.kRightJoystick);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  // Create vision subsystem
  //private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // Create grabber subsystem
  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  // Create elevator subsystem
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  // Create PID controllers for trajectory tracking
  public final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  public final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);

  // Create a non profiled PID controller for path planner
  private final PIDController ppThetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

  // Create xbox controller
  private final XboxController xbox = new XboxController(3);

  // Create tx16s transmitter
  private final Joystick tx16s = new Joystick(4);

 // private final LightStrip strips = new LightStrip();

  //--------------------------P-A-T-H-S----------------------------//

  // Manual path 
  private final PathPlannerTrajectory pathOne = PathPlanner.loadPath("forward1M", new PathConstraints(0.25, 0.25)); 
  private Command autoForward = new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true);

  // Routine
  private Command autoOne = new AutoOne(swerveSubsystem, xController, yController, ppThetaController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    // Joystick Numbers 0 = LEFT : 1 = RIGHT
    // Joystick Axises: 0 = left/right : 1 = forward/backwards : 2 = dial
    // OLD-> Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2

  //>--------------T-R-A-N-S-----------------//
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> tx16s.getRawAxis(0), // X-Axis
    () -> -tx16s.getRawAxis(1), // Y-Axis
    () -> tx16s.getRawAxis(3), // R-Axis
    () -> tx16s.getRawButton(0), // Field oriented -does nothing right now
    () -> swerveSubsystem.getHeading(), // Navx heading
    () -> tx16s.getRawButton(4))); // Flick offset button, should be toggle!

   elevatorSubsystem.setDefaultCommand(new ElevatorJoystick(elevatorSubsystem,
   () -> xbox.getRawAxis(1)));

   grabberSubsystem.setDefaultCommand(new GrabberTrigger(grabberSubsystem,
   () -> xbox.getRawAxis(3)));

  //>----------S-E-N-D-E-R----------<//

  // Add auto commands to selector
    autoChooser.setDefaultOption("Forward", autoForward);
    autoChooser.addOption("AutoOne", autoOne);

  // Add auto chooser to smart dashboard
    SmartDashboard.putData(autoChooser);
  
  //>------------------------------<//

    // Run button binding method
    configureButtonBindings();

  }

  //------------------------------------D-E-B-U-G------------------------------------//

  private double zeroFunct(){return 0;}

  private boolean trueFunct(){return true;}

  //------------------------------------B-U-T-T-O-N-S------------------------------------//

  // Create button bindings
  private void configureButtonBindings(){

    new JoystickButton(tx16s, 2).onTrue(new ElevatorSolenoid(elevatorSubsystem));

    new JoystickButton(xbox, 4).onTrue(new GrabberSolenoid(grabberSubsystem));

    new JoystickButton(xbox, 2).onTrue(new GrabberIntake(grabberSubsystem));
    new JoystickButton(xbox, 2).toggleOnFalse(new GrabberIntakeStop(grabberSubsystem));

    new JoystickButton(xbox, 1).onTrue(new GrabberIntakeReverse(grabberSubsystem));
    new JoystickButton(xbox, 1).toggleOnFalse(new GrabberIntakeStop(grabberSubsystem));

   // new JoystickButton(xbox, 9).onTrue(new ElevatorMeters(elevatorSubsystem, 0.5));

    // Homing
    //new JoystickButton(xbox, 1).onTrue(new ElevatorHome(elevatorSubsystem));
    // Apriltag
   // new JoystickButton(xbox, 2).onTrue(new ElevatorApriltag(elevatorSubsystem, visionSubsystem));
    // Meters
   // new JoystickButton(xbox, 3).onTrue(new ElevatorMeters(elevatorSubsystem, 1.0));

    //--------------// Auto Bindings

    // Apriltag
    // new JoystickButton(tx16s, 8).onTrue(new SwerveAlignBasic(swerveSubsystem, visionSubsystem,
    //   () -> swerveSubsystem.getHeading(), () -> tx16s.getRawButton(8), () -> tx16s.getRawAxis(5)));
    
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

    // Command to run
    Command autoCommand = null;

    return autoChooser.getSelected();
  }

}