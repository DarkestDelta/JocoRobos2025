package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {

    private SparkMax LeftOuttakeMotor = new SparkMax(EndEffectorConstants.kLeftOutTakeMotorCanId, MotorType.kBrushless);
    private SparkMax RightOuttakeMotor = new SparkMax(EndEffectorConstants.kRightOutTakeMotorCanId, MotorType.kBrushless);
    private SparkMax BallHolderPivotMotor = new SparkMax(EndEffectorConstants.kBallHolderMotorCanId, MotorType.kBrushless);
    private SparkMax BallHolderGrabMotor = new SparkMax(EndEffectorConstants.kBallGrabberMotorCanId, MotorType.kBrushless);
    private DutyCycleEncoder BallHolderPivotEncoder = new DutyCycleEncoder(EndEffectorConstants.kBallBolderEncoderDIOPort);

    public Servo LLServo = new Servo(0);
    public double armUpEncoderValue = 1.0;  // Adjust this value as needed

    public EndEffectorSubsystem() {
    }

    public void Shoot(double shootingSpeed) {
        RightOuttakeMotor.set(-shootingSpeed);
    }

    public void L1Shoot(double shootingSpeed) {
        RightOuttakeMotor.set(shootingSpeed);
        LeftOuttakeMotor.set(-shootingSpeed * 0.25);
    }

    public void SetBallHolderGrabMotor(double speed) {
        BallHolderGrabMotor.set(speed);
    }

    public void SetBallHolderPivotMotor(double speed) {
        BallHolderPivotMotor.set(speed);
    }

    public void SetLLServo(double value) {
        LLServo.setAngle(value);
    }
    
    // Provide a getter for the pivot encoder value
    public double getArmEncoder() {
        return BallHolderPivotEncoder.get();
    }

    @Override
    public void periodic() {
        // Periodic code if needed.
    }
}
