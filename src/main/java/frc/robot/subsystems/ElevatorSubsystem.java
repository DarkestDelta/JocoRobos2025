package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ElevatorConstants;
import java.lang.Math;

public class ElevatorSubsystem {

Encoder ElevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoderDIOPorts[1], ElevatorConstants.kElevatorEncoderDIOPorts[2]);
DigitalInput LowerLimitSwitch = new DigitalInput(ElevatorConstants.kLowerLimitSwitchDIOPort);
DigitalInput UpperLimitSwitch = new DigitalInput(ElevatorConstants.kUpperLimitSwitchDIOPort);
SparkMax ElevatorDriveMotor1 = new SparkMax(ElevatorConstants.kElevatorDriveMotor1CanId, MotorType.kBrushless);
SparkMax ElevatorDriveMotor2 = new SparkMax(ElevatorConstants.kElevatorDriveMotor2CanId, MotorType.kBrushless);
// TODO: Set Motor2 to follow Motor 1 in REV

int LiftCase = 0;
double FinalLiftSpeed = 0;



public Boolean GetUpperLimitSwitch()
    {return UpperLimitSwitch.get();}

public Boolean GetLowerLimitSwitch()
    {return LowerLimitSwitch.get();}

public double ElevatorPosition()
    {
        ElevatorEncoder.setDistancePerPulse(ElevatorConstants.kElevatorChainRatio);
            if (GetLowerLimitSwitch())
                {ElevatorEncoder.reset();}
            else{}
        return ElevatorEncoder.getDistance();
    }

public void Lift(double LiftSpeed)
    {
    switch(GetLiftCase(LiftSpeed))
    {
    case 1:
        ElevatorDriveMotor1.set(FinalLiftSpeed);
        break;

    case 2:
        ElevatorDriveMotor1.set(0);
        break;
        //TODO: add a notif error using elastic. 

    case 3:
        ElevatorDriveMotor1.set(0);
        break;
        //TODO: add a notif error using elastic.

    case 4:
        ElevatorDriveMotor1.set(FinalLiftSpeed);
        break;

    case 5:
        ElevatorDriveMotor1.set(FinalLiftSpeed);
        break;
    }
    }

public int GetLiftCase(double LiftSpeed)
{
    if (GetLowerLimitSwitch()      && (LiftSpeed > 0))
        {LiftCase = 1;}
    else if (GetLowerLimitSwitch() && (LiftSpeed < 0))
        {LiftCase = 2;}
    else if (GetUpperLimitSwitch() && (LiftSpeed > 0))
        {LiftCase = 3;}
    else if (GetUpperLimitSwitch() && (LiftSpeed < 0))
        {LiftCase = 4;}
    else
        {LiftCase = 5;}
    return LiftCase; 
}

public double CalculateFinalLiftSpeed(double LiftSpeed)
    {
    double ElevatorPositionPercentage = (ElevatorPosition()/ElevatorConstants.kMaxElevatorHeight);
    if (LiftSpeed > 0)
    {FinalLiftSpeed = Math.pow((ElevatorPositionPercentage + 1), 2);}
    else if (LiftSpeed < 0)
    {FinalLiftSpeed = Math.pow(ElevatorPositionPercentage, 2);}
    else
    {FinalLiftSpeed = 0;}

    return FinalLiftSpeed;
    }


}
