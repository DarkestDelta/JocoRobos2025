package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.Vision.LimelightHelpers;

public class LimeLightCommands {
    private final DriveSubsystem m_robotDrive;
    private final EndEffectorSubsystem m_robotEndEffector;

    public LimeLightCommands(RobotContainer container) {
        this.m_robotDrive = container.m_robotDrive;
        this.m_robotEndEffector = container.m_robotEndEffector;
    }

    // Vision data fields
    private double tx, ty, ta, tagYaw, distance;
    private boolean hasTarget;
    private int aprilTagID;

    // Tuning constants
    private static final double kAlignmentSpeedFactor = 0.05;
    private static final double kForwardSpeedFactor = 0.2;
    private static final double kRotationSpeedFactor = 0.01; // Reduced gain for smoother rotation
    private static final double kMinStrafeSpeed = 0.0125;
    private static final double kMaxStrafeSpeed = 0.05;
    private static final double kMaxRotationSpeed = 0.3; // Limit rotation speed
    private static final double kMaxForwardSpeed = .4;

    // Deadband values to avoid oscillation
    private static final double kDeadbandTX = 5; // degrees
    private static final double kDeadbandYaw = 5; // degrees

    // Exponential smoothing factor for rotation error (between 0 and 1, higher means less smoothing)
    private double smoothingFactor = 0.4;
    private double filteredRotationError = 0;

    // Allowed AprilTag IDs for Coral
    private int[] coralAprilTagIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    /**
     * Updates all vision data from the Limelight, including 3D tracking values via the camtran array.
     */
    public void updateVisionData() {
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        hasTarget = LimelightHelpers.getTV("");
        aprilTagID = (int) LimelightHelpers.getFiducialID("");

        // Retrieve the camtran array directly from NetworkTables
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] camtran = limelightTable.getEntry("camtran").getDoubleArray(new double[0]);
        if (camtran.length >= 6) {
            tagYaw = camtran[4];   // Adjust index as needed for your configuration
            distance = camtran[2]; // Adjust index as needed for your configuration
        } else {
            tagYaw = 0;
            distance = 0;
        }
    }

    /**
     * Checks if the current target is valid (i.e., a Coral AprilTag).
     */
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

    /**
     * Applies a deadband to the value.
     */
    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    /**
     * Calculates the strafe (lateral) speed based on horizontal offset (tx) and target area.
     */
    private double calculateStrafeSpeed() {
        if (ta == 0) return 0;
        double speedMultiplier = MathUtil.clamp(1 - (ta / 14), 0.2, 1.0);
        double rawSpeed = tx * kAlignmentSpeedFactor * speedMultiplier;
        return MathUtil.clamp(rawSpeed, -kMaxStrafeSpeed, kMaxStrafeSpeed)
                + Math.copySign(kMinStrafeSpeed, rawSpeed);
    }

    /**
     * Calculates the rotation speed correction by combining the horizontal offset and tag yaw.
     * Applies deadbands and exponential smoothing to reduce oscillation.
     */
    private double calculateRotationSpeed() {
        // Apply deadbands to avoid small oscillations
        double effectiveTX = applyDeadband(tx, kDeadbandTX);
        double effectiveYaw = applyDeadband(tagYaw, kDeadbandYaw);

        // Calculate combined error and apply gains
        double rotationError = (effectiveTX + effectiveYaw) * kRotationSpeedFactor;

        // Apply exponential smoothing
        filteredRotationError = (smoothingFactor * rotationError) + ((1 - smoothingFactor) * filteredRotationError);
        
        return MathUtil.clamp(filteredRotationError, -kMaxRotationSpeed, kMaxRotationSpeed);
    }

    /**
     * Calculates the forward speed to drive toward the tag using the distance value.
     */
    private double calculateForwardSpeed() {
        double speed =  kForwardSpeedFactor;
        return MathUtil.clamp(speed, -kMaxForwardSpeed, kMaxForwardSpeed);
    }

    /**
     * Main command to align and drive toward the AprilTag.
     */
    public void seekAndAlign() {
        updateVisionData();
        m_robotEndEffector.SetLLServo(10);
        if (hasValidTarget()) {
            double strafeSpeed = calculateStrafeSpeed();
            double rotationSpeed = calculateRotationSpeed();
            double forwardSpeed = calculateForwardSpeed();
            m_robotDrive.drive(forwardSpeed, strafeSpeed, rotationSpeed, true);
        } else {
            // Reset filtered error when target is lost
            filteredRotationError = 0;
            m_robotDrive.drive(0, 0, 0, true);        }
    }
}
