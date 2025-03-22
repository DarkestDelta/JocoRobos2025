package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.co/mands.ButtonBindings;
// import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    public Timer timer = new Timer();

    private static final String Forward = "Forward";
    private static final String Left = "Left";
    private static final String Middle = "Middle";
    private static final String Right = "Right";

    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    public Robot() {
        m_chooser.setDefaultOption("Forward", Forward);
        m_chooser.addOption("Left", Left);
        m_chooser.addOption("Middle", Middle);
        m_chooser.addOption("Right", Right);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        m_autoSelected = m_chooser.getSelected();

        if (m_autoSelected == null) {
            System.out.println("Warning: No auto mode selected, defaulting to Forward.");
            m_autoSelected = Forward;
        }

        System.out.println("Auto selected: " + m_autoSelected);

        if (m_robotContainer == null) {
            System.out.println("Error: RobotContainer is null!");
        } else if (m_robotContainer.m_robotDrive == null) {
            System.out.println("Error: RobotDrive is null in RobotContainer!");
        }

        SmartDashboard.putBoolean("Is Autonomous", isAutonomous());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putString("Event Name", DriverStation.getEventName());
        SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
        SmartDashboard.putString("Game Message", DriverStation.getGameSpecificMessage());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
public void autonomousInit() {
    timer.reset();
    timer.start();

    switch (m_autoSelected) {
        case Left:
            m_autonomousCommand = m_robotContainer.LeftAuto()
                .andThen(waitForElevatorAndShoot());
            break;
        case Middle:
            m_autonomousCommand = m_robotContainer.CenterAuto()
                .andThen(waitForElevatorAndShoot());
            break;
        case Right:
            m_autonomousCommand = m_robotContainer.RightAuto()
                .andThen(waitForElevatorAndShoot());
            break;
        default:
            m_autonomousCommand = m_robotContainer.Forward()
                .andThen(waitForElevatorAndShoot());
            break;
    }

    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
}


    @Override
    public void autonomousPeriodic() {

    


    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationPeriodic() {}
    


    private Command waitForElevatorAndShoot() {
    return new RunCommand(() -> {
        if (m_robotContainer.buttons.elevatorL3.isTargetReached() || m_robotContainer.buttons.elevatorL2.isTargetReached()) {
            m_robotContainer.buttons.m_robotEndEffector.Shoot(0.15);
        } else if (m_robotContainer.buttons.elevatorL4.isTargetReached()) {
            m_robotContainer.buttons.m_robotEndEffector.Shoot(0.05);
        }
    }, m_robotContainer.m_robotEndEffector)
    .withTimeout(1.25) // Ensures the command doesn't run forever
    .andThen(new InstantCommand(() -> m_robotContainer.buttons.m_robotEndEffector.Shoot(0), m_robotContainer.buttons.m_robotEndEffector)); // Stop shooting after timeout
}

}
