package frc.robot;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ButtonBindings;
import frc.robot.commands.ElevatorTargetCommand;
import frc.robot.commands.LimeLightCommands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    public DriveSubsystem m_robotDrive;
    public ElevatorSubsystem m_robotElevator;
    public ClimberSubsystem m_robotClimber;
    public IntakeSubsystem m_robotIntake;
    public EndEffectorSubsystem m_robotEndEffector;
    
    public Joystick m_driverController;
    public Joystick m_ButtonController;
    public XboxController m_XboxDriverController;
    
    public ButtonBindings buttons;
    private static RobotContainer instance;
    public LimeLightCommands LLCom;
    public ElevatorTargetCommand EleCom;

    public RobotContainer() {
        instance = this;
        // Instantiate subsystems and controllers.
        initiateSubsystems();
        // Instantiate button bindings.
        LLCom = new LimeLightCommands(this);

        buttons = new ButtonBindings(this);
        buttons.configureButtonBindings();
        // Now register commands that depend on button bindings.
        registerNamedCommands();
        // Instantiate LimeLight commands.
        // Set default command for drive.
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), false), m_robotDrive));    
    }
     
    public static RobotContainer getInstance() {
        return instance;
    }
    
    public Joystick getDriverController() {
        return m_driverController;
    }

    private double C1Y() {
        return -MathUtil.applyDeadband(m_driverController.getY() * LiftSlider(), OIConstants.kDriveDeadband);
    }

    private double C1X() {
        return -MathUtil.applyDeadband(m_driverController.getX() * LiftSlider(), OIConstants.kDriveDeadband);
    }

    private double C1Z() {
        return -MathUtil.applyDeadband(m_driverController.getZ() * LiftSlider(), OIConstants.kDriveDeadband);
    }
    
    private double LiftSlider() {
        return ((m_driverController.getRawAxis(5) + 1) / 2);
    }

    public void initiateSubsystems() {
        m_robotDrive = new DriveSubsystem();
        m_robotElevator = new ElevatorSubsystem();
        m_robotClimber = new ClimberSubsystem();
        m_robotIntake = new IntakeSubsystem();
        m_robotEndEffector = new EndEffectorSubsystem();

        m_driverController = new Joystick(OIConstants.kDriverControllerPort);
        m_ButtonController = new Joystick(OIConstants.kButtonControllerPort);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ElevatorBall1", buttons.elevatorBall1);
        NamedCommands.registerCommand("ElevatorBall2", buttons.elevatorBall2);
        NamedCommands.registerCommand("ElevatorL2", buttons.elevatorL2);
        NamedCommands.registerCommand("ElevatorL3", buttons.elevatorL3);
        NamedCommands.registerCommand("ElevatorL4", buttons.elevatorL4);
        NamedCommands.registerCommand("Shoot", new RunCommand(() -> m_robotEndEffector.Shoot(0.45)));
        NamedCommands.registerCommand("Stop Shooting", new RunCommand(() -> m_robotEndEffector.Shoot(0)));
    }

    // Autonomous command loading methods remain unchanged...
    private Command loadPathAndFollow(String pathName) {
        PathPlannerPath path;
        try {
            // Load the PathPlanner path
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException | FileVersionException e) {
            System.err.println("🚨 Error: Failed to load PathPlanner path '" + pathName + "'! Reason: " + e.getMessage());
            e.printStackTrace();
            return new InstantCommand(() -> System.out.println("⚠️ Running fallback autonomous: No path loaded."));
        }
        Translation2d startPosition = path.getPoint(0).position;
        Pose2d startingPose = new Pose2d(startPosition, new Rotation2d(0));
        m_robotDrive.resetOdometry(startingPose);
        return AutoBuilder.followPath(path)
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));
    }


    private Command loadAuto(String autoName) {
    List<PathPlannerPath> autoPaths;
    try {
        // Loads the entire auto (a list of paths) from a .auto file
        autoPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    } catch (IOException | ParseException | FileVersionException e) {
        System.err.println("Error loading auto '" + autoName + "': " + e.getMessage());
        return new InstantCommand(() -> System.out.println("Fallback: No auto loaded."));
    }

    // For example, build a SequentialCommandGroup of each path
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    for (PathPlannerPath path : autoPaths) {
        autoCommand.addCommands(AutoBuilder.followPath(path));
    }

    // Optionally reset odometry to the first path's start
    if (!autoPaths.isEmpty()) {
        Translation2d startPos = autoPaths.get(0).getPoint(0).position;
        Pose2d startPose = new Pose2d(startPos, new Rotation2d(0));
        m_robotDrive.resetOdometry(startPose);
    }

    // Return the combined auto, plus a stop command at the end
    return autoCommand.andThen(new InstantCommand(() -> m_robotDrive.drive(0,0,0,false), m_robotDrive));
}


    // Autonomous Commands
    public Command Forward() {
        return loadPathAndFollow("Example Path");
    }
    public Command LeftAuto() {
        return loadAuto("Left Auto");
    }
    public Command CenterAuto() {
        return loadAuto("Center Auto");
    }
    public Command RightAuto() {
        return loadAuto("Right Auto");
    }
}
    
