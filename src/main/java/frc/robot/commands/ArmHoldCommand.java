package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ArmHoldCommand extends Command {
    private final EndEffectorSubsystem endEffector;
    private final double targetEncoderValue;
    private final double baseSpeed;
    private final double holdVoltage;
    private boolean targetReached = false;

    public ArmHoldCommand(EndEffectorSubsystem endEffector, double targetEncoderValue, double baseSpeed, double holdVoltage) {
        this.endEffector = endEffector;
        this.targetEncoderValue = targetEncoderValue;
        this.baseSpeed = baseSpeed;
        this.holdVoltage = holdVoltage;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        targetReached = false;
    }

    @Override
    public void execute() {
        if (!targetReached) {
            // Continue moving upward until the encoder reaches the target
            if (endEffector.getArmEncoder() < targetEncoderValue) {
                endEffector.SetBallHolderPivotMotor(baseSpeed);
            } else {
                targetReached = true;
            }
        } else {
            // Once reached, apply the hold voltage to maintain the position
            endEffector.SetBallHolderPivotMotor(holdVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        // This command is designed to run until canceled.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor when the command ends
        endEffector.SetBallHolderPivotMotor(0);
    }
}
