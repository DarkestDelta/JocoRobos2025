// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< Updated upstream
<<<<<<< Updated upstream


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Vision.LimelightHelpers;
=======
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
=======
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.OIConstants;
>>>>>>> Stashed changes
import frc.robot.commands.ButtonBindings;
import frc.robot.commands.LimeLightCommands;
>>>>>>> Stashed changes
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
import frc.robot.util.ElasticMessages;
=======
=======
>>>>>>> Stashed changes
// import frc.robot.commands.PathPlannerCommands;

>>>>>>> Stashed changes



public class RobotContainer {
<<<<<<< Updated upstream
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorSubsystem m_robotElevator = new ElevatorSubsystem();
  private final ClimberSubsystem m_robotClimber = new ClimberSubsystem();
=======
    public DriveSubsystem m_robotDrive;
    public ElevatorSubsystem m_robotElevator;
    public ClimberSubsystem m_robotClimber;
    public IntakeSubsystem m_robotIntake;
    public EndEffectorSubsystem m_robotEndEffector;
    // public PathPlannerCommands m_robPathPlannerCommands;

    
    public Joystick m_driverController;
    public Joystick m_ButtonController;
    public XboxController m_XboxDriverController;
    
    public ButtonBindings buttons;
    public LimeLightCommands LLCom;
    private static RobotContainer instance;
    // private final PathPlannerCommands pathPlannerCommands;
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

  
  // Pass that instance to ElasticMessages
  private final ElasticMessages elasticMessages = new ElasticMessages(m_robotElevator);
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  private final EndEffectorSubsystem m_robotEndEffector = new EndEffectorSubsystem();
  
  
  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

<<<<<<< Updated upstream
    // Configure default commands ``
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand( () -> m_robotDrive.drive(C1Y(), C1X(), C1Z(),true), m_robotDrive));
  }

  public Command DriveAuto(Double x, Double z) {
    double tx = LimelightHelpers.getTX("");
=======
        instance = this;
        initiateSubsystems();
        LLCom = new LimeLightCommands(this);
        buttons = new ButtonBindings(this);
        buttons.configureButtonBindings();
        // pathPlannerCommands = new PathPlannerCommands(m_robotDrive);

        
        m_robotDrive.setDefaultCommand(
            new RunCommand(() -> m_robotDrive.drive(C1Y(), C1X(), C1Z(), true), m_robotDrive));      
<<<<<<< Updated upstream
    }
>>>>>>> Stashed changes

    System.out.println("TY:"+ty); // Horizontal offset from crosshair to target in degrees
    System.out.println("TX:"+tx); // Horizontal offset from crosshair to target in degrees

    return new RunCommand( () -> m_robotDrive.drive(tx/50, ty/50,0.0, true), m_robotDrive);
  }

  public ElevatorSubsystem getElevatorSubsystem() {
    return m_robotElevator;
}

private boolean servoState = false; // Initial state (false = 0 degrees, true = 180 degrees)
private boolean intakeState = false; // Initial state (false = down, true = up)

double tx = LimelightHelpers.getTX("") * -1;  // Horizontal offset from crosshair to target in degrees
double ty = LimelightHelpers.getTY("") * -1;  // Vertical offset from crosshair to target in degrees
double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(1);
  // System.out.println("Button 1 pressed: " + buttonPressed); // Debug button state
  return buttonPressed;
}).whileTrue(
  new RunCommand(
      () -> {
          double axisValue = MathUtil.applyDeadband(m_driverController.getRawAxis(7), 0.1);
          // System.out.println("Axis 7 value (Elevator): " + axisValue); // Debug axis value
          m_robotElevator.Lift(1);
      },
      m_robotElevator
  )
).whileFalse(
  new InstantCommand(() -> {
       System.out.println("Button 1 released, stopping elevator"); // Debug stop
      m_robotElevator.Lift(0); // Explicitly stop the elevator
  }, m_robotElevator)
);

// Climber control with Button 2
new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(2);
  // System.out.println("Button 2 pressed: " + buttonPressed); // Debug button state
  return buttonPressed;
}).whileTrue( // Run while Button 2 is pressed
  new RunCommand(
      () -> {
          double axisValue = m_driverController.getRawAxis(7);
          m_robotClimber.Climb(axisValue * 0.5);
      },
      m_robotClimber // Subsystem requirement
  )
).onFalse( // Stop the climber when Button 2 is released
  new InstantCommand(() -> {
      // System.out.println("Button 2 released, stopping climber"); // Debug stop
      m_robotClimber.Climb(0); // Stop the climber
  }, m_robotClimber)
);
// .andThen( // Chain the ServoSet command
//       new InstantCommand(() -> {
//           m_robotClimber.ServoSet(false); // Set the servo to false
//       }, m_robotClimber) // Subsystem requirement
//   )


new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(8);
  return buttonPressed;
}).onTrue(new InstantCommand(() -> {
      servoState = !servoState;
      m_robotClimber.ServoSet(servoState);
  }, m_robotClimber) 
);
// Updated Trigger code (in RobotContainer)
new Trigger(() -> m_driverController.getRawButton(3))
    .whileTrue(new RunCommand(() -> {
      double axisValue = m_driverController.getRawAxis(7);
      m_robotIntake.RaiseIntake(axisValue);
    }, m_robotIntake)
      
  )
    .whileFalse(new RunCommand(() -> m_robotIntake.RaiseIntake(0)));




new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(4);
  return buttonPressed;
}).whileTrue(
  new RunCommand(
      () -> {
        m_robotEndEffector.Shoot(.25);
        
      },
      m_robotEndEffector
  )
).whileFalse(
  new InstantCommand(() -> {
      m_robotEndEffector.Shoot(.0); // Explicitly stop the elevator
  }, m_robotEndEffector)
);

new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(5);
  return buttonPressed;
}).whileTrue(
  new RunCommand(
      () -> {
        double axis =  m_driverController.getRawAxis(7);
        m_robotEndEffector.BallHolderPivotMotor(.25, axis);
      }, m_robotEndEffector)
).whileFalse(
  new InstantCommand(() -> {
    double axis =  m_driverController.getRawAxis(7);
      m_robotEndEffector.BallHolderPivotMotor(0, axis); 
  }, m_robotEndEffector)
);

new Trigger(() -> {
  boolean buttonPressed = m_driverController.getRawButton(6);
  return buttonPressed;
}).whileTrue(
  new RunCommand(
      () -> {
        m_robotEndEffector.BallHolderGrabMotor(.25);
        
      },
      m_robotEndEffector
  )
).whileFalse(
  new InstantCommand(() -> {
    m_robotEndEffector.BallHolderGrabMotor(0); // Explicitly stop the elevator
  }, m_robotEndEffector)
);




  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // Return the AlignHorizontalCommand for autonomous





public Command print(String message) {
    return new InstantCommand(() -> System.out.println(message));
}
  
  // }

  double C1Y()
  {
    double Controller1YValue = -MathUtil.applyDeadband(m_driverController.getY()  * LiftSlider(), OIConstants.kDriveDeadband);
    return Controller1YValue;
  }

  double C1X()
  {
    double Controller1YValue = -MathUtil.applyDeadband(m_driverController.getX()  * LiftSlider(), OIConstants.kDriveDeadband);
    return Controller1YValue;
  }

  double C1Z()
  {
    double Controller1YValue = -MathUtil.applyDeadband(m_driverController.getRawAxis(3) * LiftSlider(), OIConstants.kDriveDeadband);
    return Controller1YValue;
  }

  double LiftSlider()
  {
    Double Scaler = ((m_driverController.getRawAxis(7) + 1)/2);
    return Scaler;
  }

// Helper enum for direction tracking
private enum TargetDirection {
    LEFT, RIGHT, NONE
}

// State class to hold variables
private static class State {
    TargetDirection lastKnownDirection = TargetDirection.NONE;
    double lastValidTX = 0;
    double filteredTX = 0;
    double filteredTY = 0;
}



public Command BasicAutoFollow() {
    // PID for forward/backward speed (ty)
    PIDController yController = new PIDController(0.02, 0, 0.001);  // Reduced P gain
    // PID for lateral (strafe) speed (tx)
    PIDController xController = new PIDController(0.025, 0, 0.001); // Adjusted for strafing
    // Profiled PID for rotation (using tx)
    ProfiledPIDController thetaController = new ProfiledPIDController(
        0.04,  // Reduced P gain for smoother turning
        0, 
        0.002,
        new TrapezoidProfile.Constraints(2.0, 3.0) // Reduced max turn speed
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Threshold (in degrees) to start combining turning with strafing
    final double TY_THRESHOLD = 8.0;

    // Create a State object to hold variables
    State state = new State();

    return new RunCommand(() -> {
        // Get Limelight data
        double tx = LimelightHelpers.getTX("");
        double ty = LimelightHelpers.getTY("");
        boolean tv = LimelightHelpers.getTV("");

        if (tv) {
            // Update last known position
            state.lastValidTX = tx;
            updateLastKnownDirection(tx, state);

            // Apply filtering and deadband
            tx = Math.abs(tx) < 1.5 ? 0 : tx;
            ty = Math.abs(ty) < 1.5 ? 0 : ty;
            state.filteredTX = 0.4 * tx + 0.6 * state.filteredTX;
            state.filteredTY = 0.4 * ty + 0.6 * state.filteredTY;

            // Calculate base speeds
            double ySpeed = yController.calculate(state.filteredTY, 0) + 0.15; // Reduced constant
            double xSpeed = xController.calculate(state.filteredTX, 0); // Always strafe
            double thetaSpeed = 0;

            // Add turning when close to target
            if (state.filteredTY >= TY_THRESHOLD) {
                thetaSpeed = -thetaController.calculate(Math.toRadians(state.filteredTX), 0);
            }

            // Clamp outputs for safety
            ySpeed = MathUtil.clamp(ySpeed, -0.4, 0.4);
            xSpeed = MathUtil.clamp(xSpeed, -0.4, 0.4);
            thetaSpeed = MathUtil.clamp(thetaSpeed, -0.4, 0.4);

            // Drive the robot (field-relative)
            m_robotDrive.drive(ySpeed, xSpeed, thetaSpeed, true);
        } else {
            // When target is lost, use recovery behavior
            double[] recoverySpeeds = getRecoveryTurnSpeed(state);
            m_robotDrive.drive(
                recoverySpeeds[0],  // Forward
                recoverySpeeds[1],  // Strafe
                recoverySpeeds[2],  // Turn
                true
            );
        }
    }, m_robotDrive).finallyDo(() -> m_robotDrive.drive(0, 0, 0, false));
}

// Separate recovery logic function
private double[] getRecoveryTurnSpeed(State state) {
    final double FORWARD_SPEED = 0.1;  // Slow forward
    final double STRAFE_SPEED = 0.08;  // Gentle strafe
    final double TURN_SPEED = 0.15;    // Slow turn
    
    double xSpeed = 0;
    double thetaSpeed = 0;

    // Use last known position for recovery strafe and turn
    switch (state.lastKnownDirection) {
        case LEFT:
            xSpeed = STRAFE_SPEED;
            thetaSpeed = TURN_SPEED;
            break;
        case RIGHT:
            xSpeed = -STRAFE_SPEED;
            thetaSpeed = -TURN_SPEED;
            break;
        case NONE:
            thetaSpeed = TURN_SPEED;  // Default gentle turn
            break;
=======
>>>>>>> Stashed changes
    }

    return new double[] {FORWARD_SPEED, xSpeed, thetaSpeed};
}

// Helper to update direction memory
private void updateLastKnownDirection(double currentTX, State state) {
    // Apply deadband and update direction
    if (Math.abs(currentTX) > 2.5) {
        state.lastKnownDirection = (currentTX > 0) ? 
            TargetDirection.RIGHT : TargetDirection.LEFT;
    }
    
    // Low-pass filtering for TX value
    state.lastValidTX = 0.25 * currentTX + 0.75 * state.lastValidTX;
}

<<<<<<< Updated upstream
=======
    private double C1X() {
        return -MathUtil.applyDeadband(m_driverController.getX() * LiftSlider(), OIConstants.kDriveDeadband);
    }

    private double C1Z() {
        return -MathUtil.applyDeadband(
            // m_driverController.getRawAxis(3)
            m_driverController.getZ()
             * LiftSlider(), OIConstants.kDriveDeadband);
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

    // public SequentialCommandGroup examplepathCommandGroup()
    // {


    // }


   public Command examplePath() {
    PathPlannerPath path;

    try {
        path = PathPlannerPath.fromPathFile("New New Path");
    } catch (FileVersionException | IOException | ParseException e) {
        e.printStackTrace();
        throw new RuntimeException("Failed to load path", e);
    }

    // Get the first waypoint's position
    Translation2d startPosition = path.getPoint(0).position;

    // Convert it to a full Pose2d by adding a default rotation
    Pose2d startingPose = new Pose2d(startPosition, new Rotation2d(0));

    // Reset odometry to match the path's starting pose
    m_robotDrive.resetOdometry(startingPose);

    // Use AutoBuilder to follow the path
    Command pathCommand = AutoBuilder.followPath(path);

    return pathCommand;
}



    
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
}
