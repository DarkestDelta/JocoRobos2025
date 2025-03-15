package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ButtonBindings {
    private final ClimberSubsystem m_robotClimber;
    private final DriveSubsystem m_robotDrive;
    private final EndEffectorSubsystem m_robotEndEffector;
    private final ElevatorSubsystem m_robotElevator;
    private final IntakeSubsystem m_robotIntake;
    private final Joystick m_ButtonController;
    private final Joystick m_driverController;

    // Vision data fields
    private double tx = LimelightHelpers.getTX(""); // Horizontal offset in degrees
    private double ty = LimelightHelpers.getTY(""); // Vertical offset in degrees
    private double ta = LimelightHelpers.getTA(""); // Target area
    private boolean hasTarget = LimelightHelpers.getTV(""); // Valid target indicator

    private static final double kAlignmentSpeedFactor = 0.1;
    private static final double kTaFarThreshold = 20.0;
    private static final double kTaCloseThreshold = 40.0;
    private static final double kMinStrafeSpeed = 0.15;
    private static final double kMaxStrafeSpeed = 0.4;
    private static final double kForwardSpeedFactor = 0.08;
    private static final double kRotationSpeedFactor = 0.02;

    private boolean servoState = false;
    private int[] coralAprilTagIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    private int aprilTagID = -1;

    // Declare ElevatorTargetCommand fields without inline initialization
    private final ElevatorTargetCommand elevatorL1;
    private final ElevatorTargetCommand elevatorL2;
    private final ElevatorTargetCommand elevatorL3;
    private final ElevatorTargetCommand elevatorL4;
    private final ElevatorTargetCommand elevatorBall1;
    private final ElevatorTargetCommand elevatorBall2;

    public ButtonBindings(RobotContainer container) {
        // Initialize subsystems and controllers from RobotContainer
        this.m_robotClimber = container.m_robotClimber;
        this.m_robotDrive = container.m_robotDrive;
        this.m_robotEndEffector = container.m_robotEndEffector;
        this.m_robotElevator = container.m_robotElevator;
        this.m_robotIntake = container.m_robotIntake;
        this.m_ButtonController = container.m_ButtonController;
        this.m_driverController = container.m_driverController;

        // Now that m_robotElevator is initialized, create ElevatorTargetCommands
        elevatorL1 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 1.5, 0.1);
        elevatorL2 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 1.5, 0.1);
        elevatorL3 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 1.5, 0.1);
        elevatorL4 = new ElevatorTargetCommand(m_robotElevator, 95, 0.5, 1.5, 0.1);
        elevatorBall1 = new ElevatorTargetCommand(m_robotElevator, 30, 0.5, 3, 0.1);
        elevatorBall2 = new ElevatorTargetCommand(m_robotElevator, 44, 0.5, 3, 0.1);
    }

    public void configureButtonBindings() {
        // Example button binding for button 17
        new Trigger(() -> (m_ButtonController.getRawButton(17)))
            .whileTrue(
                new RunCommand(() -> m_robotEndEffector.SetLLServo(0), m_robotEndEffector)
                    .andThen(new RunCommand(() -> {
                        // Optionally disable climber servo here if needed:
                        // m_robotClimber.ServoSet(false);
                    }, m_robotClimber))
                    .withTimeout(1)
                    .andThen(new RunCommand(() -> {
                        m_robotClimber.Climb(m_ButtonController.getRawAxis(7) * -0.45);
                    }, m_robotClimber))
            )
            .onFalse(
                new InstantCommand(() -> m_robotEndEffector.SetLLServo(180), m_robotEndEffector)
                    .andThen(new InstantCommand(() -> {
                        m_robotClimber.Climb(0);
                        // Optionally re-enable climber servo here:
                        // m_robotClimber.ServoSet(true);
                    }, m_robotClimber))
            );

        new Trigger(() -> (m_ButtonController.getRawButton(16))) // Negative axis for up
            .whileTrue(new RunCommand(() -> m_robotIntake.RaiseIntake(-0.25), m_robotIntake))
            .whileFalse(new InstantCommand(() -> m_robotIntake.RaiseIntake(0.0), m_robotIntake));

        new Trigger(() -> m_driverController.getRawButton(8))
            .onTrue(new InstantCommand(() -> {
                servoState = !servoState;
                m_robotClimber.ServoSet(servoState);
            }, m_robotClimber));

        new Trigger(() -> m_ButtonController.getRawButton(4))
            .whileTrue(new RunCommand(() -> m_robotEndEffector.Shoot(0.15), m_robotEndEffector))
            .whileFalse(new InstantCommand(() -> m_robotEndEffector.Shoot(0.0), m_robotEndEffector));

        // Button 18 binding for ball holder motors (ensure you only have one of these)
        new Trigger(() -> m_ButtonController.getRawButton(18))
            .whileTrue(new RunCommand(() -> {
                m_robotEndEffector.SetBallHolderGrabMotor(-0.5);
                m_robotEndEffector.SetBallHolderPivotMotor(-0.15);
            }, m_robotEndEffector))
            .whileFalse(new InstantCommand(() -> {
                m_robotEndEffector.SetBallHolderGrabMotor(0);
                m_robotEndEffector.SetBallHolderPivotMotor(0);
            }, m_robotEndEffector));

        new Trigger(() -> m_driverController.getRawButton(7))
            .whileTrue(new RunCommand(() -> {
                updateVisionData();

                if (hasValidTarget()) {
                    if (ta < kTaFarThreshold) {
                        double rotationSpeed = MathUtil.clamp(tx * kRotationSpeedFactor, -0.4, 0.4);
                        m_robotDrive.drive(0.2, 0, rotationSpeed, true);
                    } else if (ta < kTaCloseThreshold) {
                        double strafeSpeed = calculateStrafeSpeed();
                        double forwardSpeed = 0.3;
                        m_robotDrive.drive(forwardSpeed, strafeSpeed, 0, true);
                    } else {
                        double strafeSpeed = calculateStrafeSpeed();
                        double forwardSpeed = calculateForwardSpeed();
                        m_robotDrive.drive(forwardSpeed, strafeSpeed, 0, true);
                    }
                } else {
                    m_robotDrive.drive(0, 0, 0, true);
                }
            }, m_robotDrive))
            .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true), m_robotDrive));
    }

    private double calculateStrafeSpeed() {
        double speedMultiplier = MathUtil.clamp(1 - (ta / kTaCloseThreshold), 0.2, 1.0);
        double rawSpeed = tx * kAlignmentSpeedFactor * speedMultiplier;
        return MathUtil.clamp(rawSpeed, -kMaxStrafeSpeed, kMaxStrafeSpeed) +
                Math.copySign(kMinStrafeSpeed, rawSpeed);
    }

    private double calculateForwardSpeed() {
        double speedMultiplier = MathUtil.clamp(1 - (ta / kTaCloseThreshold), 0.2, 1.0);
        return MathUtil.clamp(-ty * kForwardSpeedFactor * speedMultiplier, -0.4, 0.4);
    }

    private void updateVisionData() {
        tx = LimelightHelpers.getTX("") * -1;
        ty = LimelightHelpers.getTY(""); // Multiplying by 1 is redundant
        ta = LimelightHelpers.getTA("");
        hasTarget = LimelightHelpers.getTV("");
        aprilTagID = (int) LimelightHelpers.getFiducialID("");
    }

    private boolean hasValidTarget() {
        return hasTarget && isCoralAprilTag(aprilTagID);
    }

    private boolean isCoralAprilTag(int aprilTagID) {
        for (int id : coralAprilTagIDs) {
            if (id == aprilTagID) {
                return true;
            }
        }
        return false;
    }

    public void ElevatorBindings() {
        new Trigger(() -> m_ButtonController.getRawButton(1)).onTrue(elevatorBall1);
        new Trigger(() -> m_ButtonController.getRawButton(2)).onTrue(elevatorBall2);
        new Trigger(() -> m_ButtonController.getRawButton(5)).onTrue(elevatorL1);
        new Trigger(() -> m_ButtonController.getRawButton(6)).onTrue(elevatorL2);
        new Trigger(() -> m_ButtonController.getRawButton(7)).onTrue(elevatorL3);
        new Trigger(() -> m_ButtonController.getRawButton(8)).onTrue(elevatorL4);

        new Trigger(() -> elevatorL4.isTargetReached() || elevatorL3.isTargetReached() || elevatorL2.isTargetReached())
            .onTrue(
                new RunCommand(() -> m_robotEndEffector.Shoot(0.45), m_robotEndEffector)
                    .withTimeout(1.25)
                    .andThen(new InstantCommand(() -> m_robotEndEffector.Shoot(0), m_robotEndEffector))
            );
    }
}
