package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.ElasticMessages;

public class ClimberSubsystem extends SubsystemBase{
SparkMax ClimbMotor = new SparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);
Servo ClimbLock = new Servo(ClimberConstants.kClimberServoPWMPort);
// IntakeSubsystem Intake = new IntakeSubsystem();
// ElasticMessages Elastic = new ElasticMessages();


public void Climb(double ClimbingSpeed)
{ 
    ServoSet(true);
    ClimbMotor.set(ClimbingSpeed);


    // if (Intake.Intakeposition > 1.5)
    //     {ClimbMotor.set(ClimbingSpeed);}
    // else {
    //     ClimbMotor.set(0);
    //     Elastic.ClimberCantGoUp();
    //     }
}

public void ServoSet(boolean State)
{
if (State)
{ClimbLock.setAngle(180);}
else
{ClimbLock.setAngle(0);}

}
}
