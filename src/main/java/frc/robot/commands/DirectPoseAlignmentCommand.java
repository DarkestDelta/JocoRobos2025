package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Vision.LimelightHelpers;

public class DirectPoseAlignmentCommand extends Command {
    private final DriveSubsystem drive;
    private final PIDController xController = new PIDController(0.1, 0, 0);
    private final PIDController thetaController = new PIDController(0.05, 0, 0);
    
    private Pose2d targetPoseRobotSpace = new Pose2d();
    
    public DirectPoseAlignmentCommand(DriveSubsystem drive) {
        this.drive = drive;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        // Convert double array to Pose2d
        double[] poseArray = LimelightHelpers.getTargetPose_RobotSpace("");
        if (poseArray.length >= 6) {
            targetPoseRobotSpace = new Pose2d(
                poseArray[0],  // X (meters)
                poseArray[1],  // Y (meters)
                Rotation2d.fromRadians(poseArray[5])  // Rotation (Z-axis)
            );
        }

        double lateralError = targetPoseRobotSpace.getX();
        double angularError = targetPoseRobotSpace.getRotation().getRadians();
        
        double strafeSpeed = -xController.calculate(lateralError, 0);
        double rotationSpeed = -thetaController.calculate(angularError, 0);
        
        drive.drive(0, strafeSpeed, rotationSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetPoseRobotSpace.getX()) < 0.1 && 
               Math.abs(targetPoseRobotSpace.getRotation().getDegrees()) < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, false);
    }
}