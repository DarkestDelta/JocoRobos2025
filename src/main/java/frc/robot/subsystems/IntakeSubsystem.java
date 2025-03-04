package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem {

SparkMax IntakeMotor = new SparkMax(IntakeConstants.kIntakeRotationMotorCanId, MotorType.kBrushless);
DutyCycleEncoder IntakeEncoder = new DutyCycleEncoder(IntakeConstants.kIntakeEncoderDIOPort);

double FinalRaiseSpeed = 0;
double Intakeposition = IntakeEncoder.get();

private void RaiseIntake(int Direction) // Postive int  = raise, neg = lower, 0 = 0
{IntakeMotor.set(CalculateFinalRaiseSpeed(Direction));}

double CalculateFinalRaiseSpeed(int Direction)
{
    if (Direction > 0 && Intakeposition <= .235)
        {FinalRaiseSpeed = ((-4.237 * (Intakeposition)) + 1);}
    else if (Direction < 0 && Intakeposition >= 0)
        {FinalRaiseSpeed = (4.237 * (Intakeposition));}
    else
        {FinalRaiseSpeed = 0;}
        return FinalRaiseSpeed;
}

}
