package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
double FinalRaiseSpeed = 0;
boolean IntakeUp = false;

    WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeRotationMotorCanId);
    DutyCycleEncoder IntakeEncoder = new DutyCycleEncoder(IntakeConstants.kIntakeEncoderDIOPort);
    
    private double Intakeposition = 0; // Current position relative to the offset

    public void RaiseIntake(double Direction) { // Positive int = raise, negative = lower, 0 = stop
        
        IntakeMotor.set(-Direction);
        // if(GetIntakePosition() >= .65) 
        // {
        //     IntakeMotor.set(-Direction);
        // }
        // else{
        //     IntakeMotor.set(0); 
        // }
    }


    // public double CalculateFinalRaiseSpeed(double Direction) {
    //     // Update the intake position relative to the offset
    //     Intakeposition = IntakeEncoder.get();

    //     if (Direction > 0 && Intakeposition <= .235) {
    //         FinalRaiseSpeed = ((-4.237 * (Intakeposition)) + 1);
    //     } else if (Direction < 0 && Intakeposition >= 0) {
    //         FinalRaiseSpeed = (4.237 * (Intakeposition));
    //     } else {
    //         FinalRaiseSpeed = 0;
    //     }
    //     return FinalRaiseSpeed * .25;
    // }


    public double GetIntakePosition() {
        // Calculate the position relative to the offset
        Intakeposition = IntakeEncoder.get();
        return Intakeposition;
    }
}