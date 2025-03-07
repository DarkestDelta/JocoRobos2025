package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
double FinalRaiseSpeed = 0;

    WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeRotationMotorCanId);
    DutyCycleEncoder IntakeEncoder = new DutyCycleEncoder(IntakeConstants.kIntakeEncoderDIOPort);

    private double encoderOffset = 0; // Track the offset manually
    private double Intakeposition = 0; // Current position relative to the offset

    public void RaiseIntake(int Direction) { // Positive int = raise, negative = lower, 0 = stop
        IntakeMotor.set(CalculateFinalRaiseSpeed(Direction));
    }

    public double CalculateFinalRaiseSpeed(int Direction) {
        // Update the intake position relative to the offset
        Intakeposition = IntakeEncoder.get() - encoderOffset;

        if (Direction > 0 && Intakeposition <= .235) {
            FinalRaiseSpeed = ((-4.237 * (Intakeposition)) + 1);
        } else if (Direction < 0 && Intakeposition >= 0) {
            FinalRaiseSpeed = (4.237 * (Intakeposition));
        } else {
            FinalRaiseSpeed = 0;
        }
        return FinalRaiseSpeed * .1;
    }

    public void ResetIntakeEncoder() {
        // Get the current absolute position of the encoder
        double currentPosition = IntakeEncoder.get();

        // Set the offset to make the current position the new zero
        encoderOffset = currentPosition;

        // Update the Intakeposition variable
        Intakeposition = 0; // Since we've reset the encoder, the position is now 0

        System.out.println("Intake Encoder Reset to: " + Intakeposition);
    }

    public double GetIntakePosition() {
        // Calculate the position relative to the offset
        Intakeposition = IntakeEncoder.get() - encoderOffset;
        return Intakeposition;
    }
}