package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {

    SparkMax LeftOuttakeMotor = new SparkMax(EndEffectorConstants.kLeftOutTakeMotorCanId, MotorType.kBrushless);
    SparkMax RightOuttakeMotor = new SparkMax(EndEffectorConstants.kRightOutTakeMotorCanId, MotorType.kBrushless);
    SparkMax BallHolderPivotMotor = new SparkMax(EndEffectorConstants.kBallHolderMotorCanId, MotorType.kBrushless);
    SparkMax BallHolderGrabMotor = new SparkMax(EndEffectorConstants.kBallGrabberMotorCanId, MotorType.kBrushless);
    DutyCycleEncoder BallHolderPivotEncoder = new DutyCycleEncoder(EndEffectorConstants.kBallBolderEncoderDIOPort);

      public Servo LLServo = new Servo(0);
  

    public void Shoot(double Shootingspeed) {
        RightOuttakeMotor.set(Shootingspeed);
        LeftOuttakeMotor.set(-Shootingspeed);
    }

    public void L1Shoot(double Shootingspeed) {
        RightOuttakeMotor.set(Shootingspeed);
        LeftOuttakeMotor.set(-Shootingspeed * .25);
    }

    public void SetBallHolderGrabMotor(double Speed) {
        BallHolderGrabMotor.set(Speed);
    }

    public void SetBallHolderPivotMotor(double Speed) {
        BallHolderPivotMotor.set(Speed);
    }

    public void SetLLServo(double value)
    {
    // 
    LLServo.setAngle(value);
    // LLServo.setSpeed(1);

    }

}