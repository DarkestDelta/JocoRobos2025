// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;



public class DriveSubsystem extends SubsystemBase {






  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

     
      

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

 public void addVisionMeasurement(Pose2d botpose, double d, Vector<N3> fill){}

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
     // ✅ FIX: Ensure RobotConfig is properly initialized
        RobotConfig config;// ✅ FIX: Corrected method for loading from GUI;
        try {
            config = RobotConfig.fromGUISettings(); // ✅ FIX: Corrected method for loading from GUI
        } catch (Exception e) {
            e.printStackTrace();
            
            config = new RobotConfig(61, 8, null, null);
            m_headingPID.enableContinuousInput(-180.0, 180.0);

        }

        // ✅ FIX: Corrected `AutoBuilder.configure()` call
        AutoBuilder.configure(
            this::getPose, // ✅ Robot pose supplier
            this::resetOdometry, // ✅ Reset odometry function
            this::getChassisSpeeds, // ✅ ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
            this::driveWithChassisSpeeds, // ✅ Drive function
            new PPHolonomicDriveController( // ✅ Corrected PathPlanner Holonomic Controller
              new PIDConstants(2.0, 0.0, 0.0), // translation
              new PIDConstants(4.0, 0.0, 0.0)  // rotation
            ),
            config, // ✅ FIX: Uses correct RobotConfig
            () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red, // ✅ FIX: Mirrors for Red Alliance
            this // ✅ Reference to this subsystem (sets command requirements)
        );

        System.out.println("✅ AutoBuilder successfully configured.");
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // At the top of your DriveSubsystem:
private final edu.wpi.first.math.controller.PIDController m_headingPID =
new edu.wpi.first.math.controller.PIDController(.015, 0.025, 0.05);

private double m_targetHeadingDegrees = 0.0;

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // 1) Convert joystick inputs to real-world speeds
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get current gyro heading in degrees (assuming getHeading() returns -180..180)
    double currentHeading = getHeading();

    // We'll decide what to do with 'rot' (the driver’s rotation command)
    double rotDelivered;
    double rotationDeadband = 0.05;  // Tweak for your controller's “no-touch” zone

    if (Math.abs(rot) > rotationDeadband) {
        // 2) Driver is actively rotating -> pass it directly
        rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        // Update the target heading to the robot's current heading
        m_targetHeadingDegrees = currentHeading;

        // Reset the PID so it doesn't cause a jump when driver releases stick
        m_headingPID.reset();
    } else {
        // 3) Driver is NOT rotating -> hold heading with PID
        // m_headingPID.calculate(current, setpoint) returns output in degrees/sec if getHeading() is in degrees
        double pidOutput = m_headingPID.calculate(currentHeading, m_targetHeadingDegrees);

        // If your drivetrain expects rad/sec, convert
        rotDelivered = Math.toRadians(pidOutput);

        // Optionally clamp to the max turn rate
        rotDelivered = Math.max(-DriveConstants.kMaxAngularSpeed,
                        Math.min(DriveConstants.kMaxAngularSpeed, rotDelivered));
    }

    // 4) Construct ChassisSpeeds (field-relative or robot-relative)
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeedDelivered,
              ySpeedDelivered,
              rotDelivered,
              Rotation2d.fromDegrees(currentHeading)
          )
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // 5) Convert to SwerveModuleState[] and desaturate
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        DriveConstants.kMaxSpeedMetersPerSecond
    );

    // 6) Finally, command each swerve module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}




  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
}
public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to SwerveModuleStates
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Normalize wheel speeds to be within max speed limits
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set the desired states to each swerve module
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // Add these methods to DriveSubsystem
public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    );
}

public void driveRobotRelative(ChassisSpeeds speeds) {
    driveWithChassisSpeeds(speeds);
}
  

}
