package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ButtonBindings;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    // Declare subsystems as fields
    public DriveSubsystem m_robotDrive;
    public ElevatorSubsystem m_robotElevator;
    public ClimberSubsystem m_robotClimber;
    public IntakeSubsystem m_robotIntake;
    public EndEffectorSubsystem m_robotEndEffector;
    
    // Declare controllers as fields
    public Joystick m_driverController;
    public Joystick m_ButtonController;
    
    // Declare ButtonBindings as a field
    public ButtonBindings buttons;

    public RobotContainer() {
        initiateSubsystems();
        buttons.configureButtonBindings();

        // Set default command for the drive subsystem
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), true), m_robotDrive)
        );
    }

    // Helper methods for joystick input
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
        // Instantiate subsystems
        m_robotDrive = new DriveSubsystem();
        m_robotElevator = new ElevatorSubsystem();
        m_robotClimber = new ClimberSubsystem();
        m_robotIntake = new IntakeSubsystem();
        m_robotEndEffector = new EndEffectorSubsystem();

        // Instantiate controllers
        m_driverController = new Joystick(OIConstants.kDriverControllerPort);
        m_ButtonController = new Joystick(OIConstants.kButtonControllerPort);

        // Instantiate ButtonBindings with dependency injection using this RobotContainer
        buttons = new ButtonBindings(this);
    }
}
