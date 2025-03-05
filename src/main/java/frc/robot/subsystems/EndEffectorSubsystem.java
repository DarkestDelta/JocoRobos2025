// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.EndEffectorConstants;

// public class EndEffectorSubsystem extends SubsystemBase{

// SparkMax LeftOuttakeMotor = new SparkMax(EndEffectorConstants.kLeftOutTakeMotorCanId, MotorType.kBrushless);
// SparkMax RightOuttakeMotor = new SparkMax(EndEffectorConstants.kRightOutTakeMotorCanId, MotorType.kBrushless);
// SparkMax BallHolderPivotMotor = new SparkMax(EndEffectorConstants.kBallHolderMotorCanId, MotorType.kBrushless);
// SparkMax BallHolderGrabMotor = new SparkMax(EndEffectorConstants.kBallHolderMotorCanId, MotorType.kBrushless);
// DutyCycleEncoder BallHolderPivotEncoder = new DutyCycleEncoder(EndEffectorConstants.kBallBolderEncoderDIOPort);

// double BallHolderPivotPosition = BallHolderPivotEncoder.get();

// public void Shoot(double Shootingspeed)
// {
// RightOuttakeMotor.set(Shootingspeed);
// LeftOuttakeMotor.set(-Shootingspeed);
// }

// public void L1Shoot(double Shootingspeed)
// {
// RightOuttakeMotor.set(Shootingspeed);
// LeftOuttakeMotor.set(-Shootingspeed * .25);
// }

// public void BallHolderGrabMotor(double Speed)
// {BallHolderGrabMotor.set(Speed);}

// public void BallHolderPivotMotor(double Speed)
// {BallHolderPivotMotor.set(Speed);}

// public void BallHolderDrop(double Speed)
// {BallHolderPivotMotor.set(Speed);}
// }
