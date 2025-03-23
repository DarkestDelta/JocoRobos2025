package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Vision.LimelightHelpers;
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
    public final ElevatorTargetCommand LLelevatorL3; // Your elevator command

    public RobotContainer() {
        instance = this;
        // Instantiate subsystems and controllers first.
        initiateSubsystems();

        // Now that m_robotElevator is initialized, create the elevator command.
        LLelevatorL3 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 1.5, 0.1);

        // Instantiate LimeLight commands.
        LLCom = new LimeLightCommands(this);
        
        // Instantiate button bindings.
        buttons = new ButtonBindings(this);
        buttons.configureButtonBindings();
        
        // Register named commands that depend on button bindings.
        registerNamedCommands();
        
        // Set default command for drive.
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), false), m_robotDrive)
        );    
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

    // Autonomous command loading methods
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

        // Build a SequentialCommandGroup of each path.
        SequentialCommandGroup autoCommand = new SequentialCommandGroup();
        for (PathPlannerPath path : autoPaths) {
            autoCommand.addCommands(AutoBuilder.followPath(path));
        }

        // Optionally reset odometry to the first path's start.
        if (!autoPaths.isEmpty()) {
            Translation2d startPos = autoPaths.get(0).getPoint(0).position;
            Pose2d startPose = new Pose2d(startPos, new Rotation2d(0));
            m_robotDrive.resetOdometry(startPose);
        }

        // Return the combined auto, plus a stop command at the end.
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

    // Class to hold filtered vision data.
    public class State {
        public double filteredTX = 0.0;
        public double filteredTY = 0.0;
    }

    public Command LLSeeker() {
        // Run BasicAutoFollow until the Limelight target area (TA) exceeds 38.
        Command followCommand = BasicAutoFollow().until(() -> LimelightHelpers.getTA("") > 38);

        // Elevator command (using the command created in the constructor)
        Command elevatorCommand = LLelevatorL3;

        // Stop drive command.
        Command stopDriveCommand = new InstantCommand(
            () -> m_robotDrive.drive(0, 0, 0, false), 
            m_robotDrive
        );

        // Create a BooleanSupplier for the WaitUntilCommand condition.
        BooleanSupplier elevatorReached = () -> LLelevatorL3.isTargetReached();

        // Shooting command triggered when the elevator command is finished.
        Command shootCommand = new WaitUntilCommand(elevatorReached)
            .andThen(
                new RunCommand(() -> m_robotEndEffector.Shoot(0.15), m_robotEndEffector)
                    .withTimeout(1.25)
                    .andThen(new InstantCommand(() -> m_robotEndEffector.Shoot(0), m_robotEndEffector))
            );

        // Combine all commands into a sequential command group.
        return new SequentialCommandGroup(
            followCommand,
            elevatorCommand,
            stopDriveCommand,
            shootCommand
        );
    }

    public Command BasicAutoFollow() {
        // PID for forward/backward speed (ty)
        PIDController yController = new PIDController(0.02, 0, 0.001);
        // PID for lateral (strafe) speed (tx)
        PIDController xController = new PIDController(0.025, 0, 0.001);
        // Profiled PID for rotation (using tx)
        ProfiledPIDController thetaController = new ProfiledPIDController(
            0.04, // Reduced P gain for smoother turning
            0, 
            0.002,
            new TrapezoidProfile.Constraints(2.0, 3.0)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // Threshold (in degrees) to start combining turning with strafing.
        final double TY_THRESHOLD = 8.0;
    
        // Create a State object to hold variables.
        State state = new State();
    
        return new RunCommand(() -> {
            // Get Limelight data.
            double tx = LimelightHelpers.getTX("");
            double ty = LimelightHelpers.getTY("");
            boolean tv = LimelightHelpers.getTV("");
    
            if (tv) {
                // Apply filtering and deadband.
                tx = Math.abs(tx) < 1.5 ? 0 : tx;
                ty = Math.abs(ty) < 1.5 ? 0 : ty;
                state.filteredTX = 0.4 * tx + 0.6 * state.filteredTX;
                state.filteredTY = 0.4 * ty + 0.6 * state.filteredTY;
    
                // Calculate base speeds.
                double ySpeed = yController.calculate(state.filteredTY, 0) + 0.15;
                double xSpeed = xController.calculate(state.filteredTX, 0);
                double thetaSpeed = 0;
    
                // Add turning when close to target.
                if (state.filteredTY >= TY_THRESHOLD) {
                    thetaSpeed = -thetaController.calculate(Math.toRadians(state.filteredTX), 0);
                }
    
                // Clamp outputs for safety.
                ySpeed = MathUtil.clamp(ySpeed, -0.4, 0.4);
                xSpeed = MathUtil.clamp(xSpeed, -0.4, 0.4);
                thetaSpeed = MathUtil.clamp(thetaSpeed, -0.4, 0.4);
    
                // Drive the robot (field-relative).
                m_robotDrive.drive(ySpeed, xSpeed, thetaSpeed, true);
            } else {
                // When target is lost, stop moving.
                m_robotDrive.drive(0, 0, 0, false);
            }
        }, m_robotDrive).finallyDo(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
