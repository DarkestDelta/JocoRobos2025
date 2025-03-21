package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.commands.PathPlannerCommands;

public class ButtonBindings {
    private final ClimberSubsystem m_robotClimber;
    private final DriveSubsystem m_robotDrive;
    private final EndEffectorSubsystem m_robotEndEffector;
    private final ElevatorSubsystem m_robotElevator;
    private final IntakeSubsystem m_robotIntake;
    private final Joystick m_ButtonController;
    private final Joystick m_driverController;
    private final LimeLightCommands LLComand;
    // private final PathPlannerCommands PathPlanComan;

    private ArmHoldCommand armHoldCommand;
    private boolean armHoldActive = false;

    public ButtonBindings(RobotContainer container) {
        this.m_robotClimber = container.m_robotClimber;
        this.m_robotDrive = container.m_robotDrive;
        this.m_robotEndEffector = container.m_robotEndEffector;
        this.m_robotElevator = container.m_robotElevator;
        this.m_robotIntake = container.m_robotIntake;
        this.m_ButtonController = container.m_ButtonController;
        this.m_driverController = container.m_driverController;
        this.LLComand = container.LLCom;

        // this.PathPlanComan = container.m_robPathPlannerCommands;

        elevatorL1 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 1.5, 0.1);
        elevatorL2 = new ElevatorTargetCommand(m_robotElevator, 14, 0.5, 1.5, 0.1);
        elevatorL3 = new ElevatorTargetCommand(m_robotElevator, 35, 0.5, 1.5, 0.1); // is prolly right
        elevatorL4 = new ElevatorTargetCommand(m_robotElevator, 91, 0.5, 1.5, 0.1);
        elevatorBall1 = new ElevatorTargetCommand(m_robotElevator, 30, 0.5, 5, 0.1);
        elevatorBall2 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 5, 0.1);
    }

    private boolean servoState = true;

    private final ElevatorTargetCommand elevatorL1;
    private final ElevatorTargetCommand elevatorL2;
    private final ElevatorTargetCommand elevatorL3;
    private final ElevatorTargetCommand elevatorL4;
    private final ElevatorTargetCommand elevatorBall1;
    private final ElevatorTargetCommand elevatorBall2;

    public void configureButtonBindings() {
        ElevatorBindings();



        new Trigger(() -> m_driverController.getRawButton(7))
        .whileTrue(new RunCommand(() -> {
            LLComand.updateVisionData();
            LLComand.seekAndAlign();
        }, m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true), m_robotDrive));


        new Trigger(() -> m_driverController.getRawButton(5))
        .onTrue(new RunCommand(() -> {
            double LRAxis = Math.signum(m_ButtonController.getRawAxis(7));

            m_robotDrive.drive(0, .25 * LRAxis, 0, true);
            
        }, m_robotDrive).withTimeout(.22));

        new Trigger(() -> m_driverController.getRawButton(1))
        .whileTrue(new RunCommand(() -> m_robotEndEffector.SetLLServo(180), m_robotEndEffector))
        .onFalse(new InstantCommand(() -> m_robotEndEffector.SetLLServo(15), m_robotEndEffector));

        new Trigger(() -> m_ButtonController.getRawButtonPressed(15)) 
        .onTrue(new InstantCommand(() -> {
            if (!armHoldActive) {
                armHoldCommand = new ArmHoldCommand(m_robotEndEffector, m_robotEndEffector.armUpEncoderValue, 0.2, 0.1);
                armHoldCommand.schedule();
                armHoldActive = true;
            } else {
                armHoldCommand.cancel();
                armHoldActive = false;
            }
        }, m_robotEndEffector));

        new Trigger(() -> m_driverController.getRawButton(4))
        .whileTrue(new RunCommand(() -> System.out.println(m_robotEndEffector.getArmEncoder()), m_robotEndEffector));


        new Trigger(() -> m_ButtonController.getRawButton(16))
        .whileTrue(new RunCommand(() -> m_robotElevator.lift(-.1), m_robotElevator))
        .whileFalse(new InstantCommand(() -> m_robotElevator.lift(0), m_robotElevator));

        new Trigger(() -> m_ButtonController.getRawButton(3))
        .whileTrue(new RunCommand(() -> {
            double DirectionAxis = Math.signum(m_ButtonController.getRawAxis(7));
            m_robotEndEffector.Shoot(-.8 * DirectionAxis);

        }, m_robotEndEffector))
        .whileFalse(new InstantCommand(() -> m_robotEndEffector.Shoot(0.0), m_robotEndEffector));

        new Trigger(() -> m_ButtonController.getRawButton(5))
            .whileTrue(new RunCommand(() -> m_robotEndEffector.Shoot(.45), m_robotEndEffector))
            .whileFalse(new InstantCommand(() -> m_robotEndEffector.Shoot(.0), m_robotEndEffector));

        new Trigger(() -> m_driverController.getRawButton(8))
            .onTrue(new InstantCommand(() -> {
                servoState = !servoState;
                m_robotClimber.ServoSet(servoState);
            }, m_robotClimber));


        new Trigger(() -> m_ButtonController.getRawButton(18))
            .whileTrue(new RunCommand(() -> {
                double DirectionAxis = Math.signum(m_ButtonController.getRawAxis(7));

                m_robotEndEffector.SetBallHolderGrabMotor(-0.5 );
                m_robotEndEffector.SetBallHolderPivotMotor(0.25 * DirectionAxis);
            }, m_robotEndEffector))
            .whileFalse(new InstantCommand(() -> {
                m_robotEndEffector.SetBallHolderGrabMotor(0);
                m_robotEndEffector.SetBallHolderPivotMotor(0);
            }, m_robotEndEffector));

        new Trigger(() -> m_ButtonController.getRawButton(19)) 
            .whileTrue(new RunCommand(() -> {
                double DirectionAxis = Math.signum(m_ButtonController.getRawAxis(7));

                m_robotIntake.RaiseIntake(0.15 * DirectionAxis);
            
            }, m_robotIntake))
            .whileFalse(new InstantCommand(() -> m_robotIntake.RaiseIntake(0.0), m_robotIntake));



            new Trigger(() -> m_ButtonController.getRawButton(17))
            .whileTrue(new RunCommand(() -> m_robotEndEffector.SetLLServo(180), m_robotEndEffector)
                .andThen(new RunCommand(() -> m_robotClimber.ServoSet(false), m_robotClimber))
                .withTimeout(1)
                .andThen(new RunCommand(() -> {
                    double axisValue = m_ButtonController.getRawAxis(7);
                    double direction = Math.signum(axisValue);
                    m_robotClimber.Climb(direction * -0.5);
                }, m_robotClimber)))
            .onFalse(new InstantCommand(() -> m_robotEndEffector.SetLLServo(0), m_robotEndEffector)
                .andThen(new InstantCommand(() -> {
                    m_robotClimber.Climb(0);
                    // m_robotClimber.ServoSet(true);
                }, m_robotClimber)));
    }

    public void ElevatorBindings() {
        new Trigger(() -> m_ButtonController.getRawButton(1)).onTrue(elevatorBall1);
        new Trigger(() -> m_ButtonController.getRawButton(2)).onTrue(elevatorBall2);
        new Trigger(() -> m_ButtonController.getRawButton(6)).onTrue(elevatorL2);
        new Trigger(() -> m_ButtonController.getRawButton(7)).onTrue(elevatorL3);
        new Trigger(() -> m_ButtonController.getRawButton(8)).onTrue(elevatorL4);

        new Trigger(() -> elevatorL3.isTargetReached() || elevatorL2.isTargetReached())
        .onTrue(new RunCommand(() -> m_robotEndEffector.Shoot(0.15), m_robotEndEffector)
            .withTimeout(1.25)
            .andThen(new InstantCommand(() -> m_robotEndEffector.Shoot(0), m_robotEndEffector)));

        new Trigger(() -> elevatorL4.isTargetReached())
            .onTrue(new RunCommand(() -> m_robotEndEffector.Shoot(0.05), m_robotEndEffector)
                .withTimeout(1.25)
                .andThen(new InstantCommand(() -> m_robotEndEffector.Shoot(0), m_robotEndEffector))
                
                );
    }
}