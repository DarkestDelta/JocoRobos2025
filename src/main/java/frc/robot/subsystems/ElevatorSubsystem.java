package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    Encoder elevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoderDIOPort1, ElevatorConstants.kElevatorEncoderDIOPort2);
    DigitalInput lowerLimitSwitch = new DigitalInput(ElevatorConstants.kLowerLimitSwitchDIOPort);
    DigitalInput upperLimitSwitch = new DigitalInput(ElevatorConstants.kUpperLimitSwitchDIOPort);
    SparkMax elevatorDriveMotor1 = new SparkMax(ElevatorConstants.kElevatorDriveMotor1CanId, MotorType.kBrushless);
    SparkMax elevatorDriveMotor2 = new SparkMax(ElevatorConstants.kElevatorDriveMotor2CanId, MotorType.kBrushless);

    

    public boolean isUpperLimitSwitchActive() {
        return !upperLimitSwitch.get(); // Inverted logic
    }

    public boolean isLowerLimitSwitchActive() {
        return !lowerLimitSwitch.get();
    }

    public double getElevatorPosition() {
        elevatorEncoder.setDistancePerPulse(.01);
        if (isLowerLimitSwitchActive()) {
            elevatorEncoder.reset();
            System.out.println("Lower limit switch triggered. Encoder reset.");
        }
        double position = elevatorEncoder.getDistance();
        System.out.println("Current Encoder Position: " + position);
        return position;
    }
    public void liftToTarget(double targetRotations, double baseSpeed) {
        double currentPos = getElevatorPosition();
        
        // Prevent upward movement if upper limit is triggered
        if (isUpperLimitSwitchActive()) {
            elevatorDriveMotor1.set(0);
            return;
        }
    
        // Calculate error (distance remaining to target)
        double error = targetRotations - currentPos;
    
        // Simple proportional control
        double computedSpeed = 0.5 * error;
    
        // Clamp speed to baseSpeed and minimum threshold
        computedSpeed = Math.min(baseSpeed, computedSpeed);
        computedSpeed = Math.max(computedSpeed, 0.1); // Minimum speed to move
    
        elevatorDriveMotor1.set(computedSpeed);
    }

    public void lift(double speed) {
        if (speed > 0 && isUpperLimitSwitchActive()) {
            elevatorDriveMotor1.set(0);
        } else {
            elevatorDriveMotor1.set(speed);
        }
    }

    @Override
    public void periodic() {
        // Optional: Add periodic logging or checks
    }
}
