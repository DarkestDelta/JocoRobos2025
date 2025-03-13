package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ElevatorTargetCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetRotations;
    private final double baseSpeed;
    private final double holdTime; // Time to hold (seconds)
    private final double holdVoltage; // Voltage to hold position (e.g., 0.1)
    private final Timer holdTimer = new Timer();
    private boolean targetReached = false;

    public ElevatorTargetCommand(
        ElevatorSubsystem elevator,
        double targetRotations,
        double baseSpeed,
        double holdTime,
        double holdVoltage
    ) {
        this.elevator = elevator;
        this.targetRotations = targetRotations;
        this.baseSpeed = baseSpeed;
        this.holdTime = holdTime;
        this.holdVoltage = holdVoltage;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        holdTimer.reset();
        targetReached = false;
        System.out.println("Starting ElevatorTargetCommand. Target: " + targetRotations);
    }

    @Override
    public void execute() {
        if (!targetReached) {
            // Move toward the target
            elevator.liftToTarget(targetRotations, baseSpeed);

            // Check if target is reached
            double currentPos = elevator.getElevatorPosition();
            double error = currentPos - targetRotations;
            if ((error <= 0.25 && error >= 0) || error >= .25) { // Tolerance of 0.1 rotations
                targetReached = true;
                holdTimer.start(); // Start the hold timer
                System.out.println("Target reached. Starting hold timer.");
            }
        } else {
            // Apply a small voltage to hold position
            elevator.lift(holdVoltage);        }
    }

    @Override
    public boolean isFinished() {
        // End the command after the hold time elapses
        return targetReached && holdTimer.hasElapsed(holdTime);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.lift(0); // Stop the motor
        holdTimer.stop();
        System.out.println("ElevatorTargetCommand ended.");
    }

    public boolean isTargetReached() {
        return targetReached;
    }
}