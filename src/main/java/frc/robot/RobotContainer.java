package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class RobotContainer {
    public DriveSubsystem m_robotDrive;
    public ElevatorSubsystem m_robotElevator;
    public ClimberSubsystem m_robotClimber;
    public IntakeSubsystem m_robotIntake;
    public EndEffectorSubsystem m_robotEndEffector;
    
    public Joystick m_driverController;
    public Joystick m_ButtonController;
    
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
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), true), m_robotDrive));

            
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

        buttons = new ButtonBindings(this);
    }



    public Command m_autonomousCommand() {
        return new RunCommand(() -> m_robotDrive.drive(0.25, 0.0, 0.0, true), m_robotDrive).withTimeout(4).alongWith(
            new InstantCommand(() ->  m_robotEndEffector.SetBallHolderPivotMotor(-.1), m_robotEndEffector))
        
        .andThen(
               new InstantCommand(() -> m_robotEndEffector.Shoot(.15), m_robotEndEffector).withTimeout(3));
    }
    
public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
}
}
