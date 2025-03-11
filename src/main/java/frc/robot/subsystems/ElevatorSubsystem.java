package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{

Encoder ElevatorEncoder = new Encoder(ElevatorConstants.kElevatorEncoderDIOPort1, ElevatorConstants.kElevatorEncoderDIOPort2);
DigitalInput LowerLimitSwitch = new DigitalInput(ElevatorConstants.kLowerLimitSwitchDIOPort);
DigitalInput UpperLimitSwitch = new DigitalInput(ElevatorConstants.kUpperLimitSwitchDIOPort);
SparkMax ElevatorDriveMotor1 = new SparkMax(ElevatorConstants.kElevatorDriveMotor1CanId, MotorType.kBrushless);
SparkMax ElevatorDriveMotor2 = new SparkMax(ElevatorConstants.kElevatorDriveMotor2CanId, MotorType.kBrushless);
// Make sure Motor2 is set to follow Motor 1 in REV

int LiftCase = 0;
double FinalLiftSpeed = 0;

public Boolean GetUpperLimitSwitch()
    {return !UpperLimitSwitch.get();}

public Boolean GetLowerLimitSwitch()
    {return !LowerLimitSwitch.get();}

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
    // System.out.println("Lower:" + GetLowerLimitSwitch());
    // System.out.println("Upper:" + GetUpperLimitSwitch());

    // ElevatorDriveMotor1.set(LiftSpeed*.25);
    

    switch(GetLiftCase(LiftSpeed))
    {
    case 1:
        ElevatorDriveMotor1.set(.15);
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
        ElevatorDriveMotor1.set(.15);
        break;

    case 5:
        ElevatorDriveMotor1.set(.15);
        System.out.println("You got here!");
        break;
    }
    }

public int GetLiftCase(double LiftSpeed)
{
    System.out.println("Encoder: " + ElevatorEncoder.get());
    System.out.println("Position: " + ElevatorPosition());


    if (GetLowerLimitSwitch()      && (LiftSpeed > 0))
        {LiftCase = 1;
        System.out.println(LiftCase);}
    else if (GetLowerLimitSwitch() && (LiftSpeed < 0))
        {LiftCase = 2;
        System.out.println(LiftCase);
        }
    else if (GetUpperLimitSwitch() && (LiftSpeed > 0))
        {LiftCase = 3;
        System.out.println(LiftCase);
        }
    else if (GetUpperLimitSwitch() && (LiftSpeed < 0))
        {LiftCase = 4;
        System.out.println(LiftCase);
        }
    else
        {LiftCase = 5;
        System.out.println(LiftCase);
        }
    return LiftCase; 
}

public double CalculateFinalLiftSpeed(double LiftSpeed)
    {
    double ElevatorPositionPercentage = (ElevatorPosition()/ElevatorConstants.kMaxElevatorHeight);
    if (LiftSpeed > 0)
    {FinalLiftSpeed = (-.5 * (ElevatorPositionPercentage) + .5);}
    else if (LiftSpeed < 0)
    {FinalLiftSpeed = (.5 * (ElevatorPositionPercentage));}
    else
    {FinalLiftSpeed = 0;}

    return FinalLiftSpeed;
    }


}
