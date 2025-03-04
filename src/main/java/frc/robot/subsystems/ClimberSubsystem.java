package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.ElasticMessages;

public class ClimberSubsystem {
SparkMax ClimbMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
IntakeSubsystem Intake = new IntakeSubsystem();
ElasticMessages Elastic = new ElasticMessages();


public void Climb(double ClimbingSpeed)
{ 
    if (Intake.Intakeposition > 1.5)
        {ClimbMotor.set(ClimbingSpeed);}
    else {
        ClimbMotor.set(0);
        Elastic.ClimberCantGoUp();
        }
}
}
