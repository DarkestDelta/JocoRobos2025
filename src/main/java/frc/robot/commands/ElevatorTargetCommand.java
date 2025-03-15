package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ElevatorTargetCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double targetRotations;
    private final double baseSpeed;
    private final double holdTime; // Time to hold (seconds)
    private final double holdSpeed; // Voltage to hold position (e.g., 0.1)
    private final Timer holdTimer = new Timer();
    private boolean targetReached = false;
    private long lastShotTime = 0; // Store last shot time

    public ElevatorTargetCommand(
        ElevatorSubsystem elevator,
        double targetRotations,
        double baseSpeed,
        double holdTime,
        double holdSpeed
    ) {
        this.elevator = elevator;
        this.targetRotations = targetRotations;
        this.baseSpeed = baseSpeed;
        this.holdTime = holdTime;
        this.holdSpeed = holdSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        holdTimer.reset();
        targetReached = false;

    }

    @Override
    public void execute() {
        if (!targetReached) {
            // Move toward the target
            elevator.liftToTarget(targetRotations, baseSpeed);

            // Check if target is reached
            double currentPos = elevator.getElevatorPosition();
  

            double error = currentPos - targetRotations;
            if ((error <= 0.25 && error >= 0) || error >= .25) { // Tolerance of 0.25 rotations
                targetReached = true;
                holdTimer.start(); // Start the hold timer
                
            }
        } else {
            // Apply a small voltage to hold position
            elevator.lift(holdSpeed);        }
    }

    
   

    @Override
public boolean isFinished() {
    boolean finished = holdTimer.get() >= holdTime;
    if (finished) {
        targetReached = false;  // Reset for the next cycle
        lastShotTime = System.currentTimeMillis(); // Store time when shot happens
        System.out.println("Elevator Command Finished. Resetting isTargetReached(). Cooldown started.");
    }
    return finished;
}

public boolean isTargetReached() {
    long cooldownPeriod = 2000; // 2-second cooldown before re-triggering
    boolean shouldTrigger = targetReached && (System.currentTimeMillis() - lastShotTime > cooldownPeriod);

    if (shouldTrigger) {
        System.out.println("Triggering Shoot - Cooldown Complete");
    }

    return shouldTrigger;
}



    @Override
    public void end(boolean interrupted) {
        elevator.lift(0); // Stop the motor
        holdTimer.stop();
        System.out.println("ElevatorTargetCommand ended.");
    }

    


    

}