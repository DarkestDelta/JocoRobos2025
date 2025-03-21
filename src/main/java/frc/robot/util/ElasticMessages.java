package frc.robot.util;
import edu.wpi.first.hal.PowerDistributionJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.subsystems.EndEffectorSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Elastic.Notification.NotificationLevel;





public class ElasticMessages {

    // Remove the direct instantiation of ElevatorSubsystem.
    private ElevatorSubsystem elevator;

    // Accept an ElevatorSubsystem instance in the constructor.
    public ElasticMessages(ElevatorSubsystem elevator) {
        this.elevator = elevator;
    }


// private EndEffectorSubsystem EndEffector = new EndEffectorSubsystem();
// private IntakeSubsystem Intake = new IntakeSubsystem();
// private ClimberSubsystem Climber = new ClimberSubsystem();

Elastic.Notification ClimberNotif = new Elastic.Notification();


public void DisplayAllMessages() // This is meant for a simple one time call. Do this unless there are issues. 
{
DisplayMatchInfo();
// DisplayLimitSwitchStates();
DisplayElevatorPosition();

}

void DisplayElevatorPosition()
{
SmartDashboard.putNumber("Elevator Position: ", elevator.getElevatorPosition());
}

void DisplayMatchInfo()
{
SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
SmartDashboard.putString("Event Name", DriverStation.getEventName());
SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
SmartDashboard.putString("Game Message", DriverStation.getGameSpecificMessage());
}

// void DisplayLimitSwitchStates()
// {
// SmartDashboard.putBoolean("Lower Limit Switch", elevator.GetLowerLimitSwitch());
// SmartDashboard.putBoolean("Upper Limit Switch", elevator.GetUpperLimitSwitch());
// }

public void ClimberCantGoUp()
{
    Elastic.sendNotification(ClimberNotif
        .withLevel(NotificationLevel.ERROR)
        .withTitle("Climber ERROR")
        .withDescription("Please raise the intake!!!")
        .withDisplaySeconds(2.5)
    );
}



}
