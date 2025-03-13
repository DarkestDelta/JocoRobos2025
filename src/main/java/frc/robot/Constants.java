// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 3;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 5;

    public static final boolean kGyroReversed = false;
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class EndEffectorConstants {
    public static final int kLeftOutTakeMotorCanId = 14;
    public static final int kRightOutTakeMotorCanId = 12;
    public static final int kBallHolderMotorCanId = 13;
    public static final int kBallGrabberMotorCanId = 11;
    public static final int kBallBolderEncoderDIOPort = 7; // Likely Absolute, which only needs one port
  }

  public static final class IntakeConstants {
    public static final int kIntakeRotationMotorCanId = 9;
    public static final int kIntakeEncoderDIOPort = 4;  // Likely Absolute, which only needs one port
    

  }
  public static final class ElevatorConstants {
    public static final double kElevatorChainRatio = 10;
    public static final int kMaxElevatorHeight = 0;
    public static final int kElevatorDriveMotor1CanId = 15;
    public static final int kElevatorDriveMotor2CanId = 16;
    public static final int kElevatorEncoderDIOPort1 =  3;
    public static final int kElevatorEncoderDIOPort2 =  2;
    public static final int kLowerLimitSwitchDIOPort = 0;
    public static final int kUpperLimitSwitchDIOPort = 1;
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorCanId = 10;
    public static final int kClimberServoPWMPort = 4;
    public static final double kClimberChainRatio = 10;
    public static final int kClimberMaxLength = 10;

  }

}
