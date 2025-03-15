package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LimeLightCommands {
    private final ClimberSubsystem m_robotClimber;
    private final DriveSubsystem m_robotDrive;
    private final EndEffectorSubsystem m_robotEndEffector;
    private final ElevatorSubsystem m_robotElevator;
    private final IntakeSubsystem m_robotIntake;
    private final Joystick m_ButtonController;
    private final Joystick m_driverController;

    public LimeLightCommands(RobotContainer container) {
        this.m_robotClimber = container.m_robotClimber;
        this.m_robotDrive = container.m_robotDrive;
        this.m_robotEndEffector = container.m_robotEndEffector;
        this.m_robotElevator = container.m_robotElevator;
        this.m_robotIntake = container.m_robotIntake;
        this.m_ButtonController = container.m_ButtonController;
        this.m_driverController = container.m_driverController;
    }

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

    private int[] coralAprilTagIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    private int aprilTagID = -1;

    private double calculateStrafeSpeed() {
        if (ta == 0) return 0; // Avoid division by zero
        double speedMultiplier = MathUtil.clamp(1 - (ta / kTaCloseThreshold), 0.2, 1.0);
        double rawSpeed = tx * kAlignmentSpeedFactor * speedMultiplier;
        return MathUtil.clamp(rawSpeed, -kMaxStrafeSpeed, kMaxStrafeSpeed) +
                Math.copySign(kMinStrafeSpeed, rawSpeed);
    }

    private double calculateForwardSpeed() {
        if (ta == 0) return 0; // Avoid division by zero
        double speedMultiplier = MathUtil.clamp(1 - (ta / kTaCloseThreshold), 0.2, 1.0);
        return MathUtil.clamp(-ty * kForwardSpeedFactor * speedMultiplier, -0.4, 0.4);
    }

    public void LLSeek() {
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
    }

    public void updateVisionData() {
        tx = LimelightHelpers.getTX("") * -1;
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        hasTarget = LimelightHelpers.getTV("");
        aprilTagID = (int) LimelightHelpers.getFiducialID("");
    }

    public boolean hasValidTarget() {
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
}