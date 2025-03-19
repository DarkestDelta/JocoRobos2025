package frc.robot;

import java.io.IOException;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.commands.ButtonBindings;
import frc.robot.commands.LimeLightCommands;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;




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
    public LimeLightCommands LLCom;
    private static RobotContainer instance;


    public RobotContainer() {

        instance = this;
        initiateSubsystems();
        LLCom = new LimeLightCommands(this); // Initialize LimeLightCommands
        buttons = new ButtonBindings(this);
        buttons.configureButtonBindings();
        
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), false), m_robotDrive));

            
        
        // ✅ FIX: Configure AutoBuilder BEFORE using followPath()
       // ✅ FIX: Correctly configure AutoBuilder for Holonomic (swerve)
       
        
            
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
        return -MathUtil.applyDeadband(
            // m_driverController.getRawAxis(3)
            m_driverController.getZ()
             * LiftSlider(), OIConstants.kDriveDeadband);
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

        buttons = new ButtonBindings(this);
    }



    public Command m_autonomousCommand() {
        PathPlannerPath path;
    
        try {
            // Attempt to load the PathPlanner path
            path = PathPlannerPath.fromPathFile("Example Path");
        } catch (IOException | ParseException | FileVersionException e) {
            // Print error message to console
            System.err.println("🚨 Error: Failed to load PathPlanner path! Reason: " + e.getMessage());
            e.printStackTrace();  // Print full error for debugging
            
            // Return a safe fallback command (e.g., do nothing)
            return new Command() {
                @Override
                public void initialize() {
                    System.out.println("⚠️ Running fallback autonomous: No path loaded.");
                }
            };
        }
    
        // Get the first waypoint's position
        Translation2d startPosition = path.getPoint(0).position;
    
        // Convert it to a full Pose2d by adding a default rotation
        Pose2d startingPose = new Pose2d(startPosition, new Rotation2d(0));
    
        // Reset odometry to match the path's starting pose
        m_robotDrive.resetOdometry(startingPose);
    
        // Use AutoBuilder to follow the path
        Command pathCommand = AutoBuilder.followPath(path);
    
        // Run the command and stop at the end
        return AutoBuilder.followPath(path)
            .andThen(new InstantCommand(() -> m_robotDrive.drive(0,0,0,false), m_robotDrive)); // ✅ Ensures the robot stops at the end
    }
    
}
