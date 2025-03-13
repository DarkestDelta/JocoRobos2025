package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorSubsystem extends SubsystemBase {

    SparkMax LeftOuttakeMotor = new SparkMax(EndEffectorConstants.kLeftOutTakeMotorCanId, MotorType.kBrushless);
    SparkMax RightOuttakeMotor = new SparkMax(EndEffectorConstants.kRightOutTakeMotorCanId, MotorType.kBrushless);
    SparkMax BallHolderPivotMotor = new SparkMax(EndEffectorConstants.kBallHolderMotorCanId, MotorType.kBrushless);
    SparkMax BallHolderGrabMotor = new SparkMax(EndEffectorConstants.kBallGrabberMotorCanId, MotorType.kBrushless);
    DutyCycleEncoder BallHolderPivotEncoder = new DutyCycleEncoder(EndEffectorConstants.kBallBolderEncoderDIOPort);

    private double encoderOffset = 0; // Track the offset manually
    private double BallHolderPivotPosition = 0; // Current position relative to the offset

    public void Shoot(double Shootingspeed) {
        RightOuttakeMotor.set(Shootingspeed *.5);
        LeftOuttakeMotor.set(-Shootingspeed * 1.5);
    }

    public void L1Shoot(double Shootingspeed) {
        RightOuttakeMotor.set(Shootingspeed);
        LeftOuttakeMotor.set(-Shootingspeed * .25);
    }

    public void BallHolderGrabMotor(double Speed) {
        BallHolderGrabMotor.set(Speed);
    }

    public void BallHolderPivotMotor(double Speed, double position) {
        double wantedratio = (position / 1);
        if (Math.abs(BallHolderPivotPosition) < wantedratio)
            BallHolderPivotMotor.set(Speed * position);
    }

    public void ResetBallHolderEncoder() {
        // Get the current absolute position of the encoder
        double currentPosition = BallHolderPivotEncoder.get();

        // Set the offset to make the current position the new zero
        encoderOffset = currentPosition;

        // Update the BallHolderPivotPosition variable
        BallHolderPivotPosition = 0; // Since we've reset the encoder, the position is now 0

        System.out.println("Ball Holder Encoder Reset to: " + BallHolderPivotPosition);
    }

    public double GetBallHolderPivotPosition() {
        // Calculate the position relative to the offset
        BallHolderPivotPosition = BallHolderPivotEncoder.get() - encoderOffset;
        return BallHolderPivotPosition;
    }

}