package frc.robot;

import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.Source;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.elevator.Elevator;

/** Put high level commands here */
public final class RobotCommands {
  public Command coralIntake(Elevator elevator, Carriage carriage) {
    return elevator
        .request(Source)
        .andThen(elevator.enableElevator())
        .until(elevator::atDesiredHeight)
        .andThen(carriage.intakeCoral())
        .withName("intakeCoral");
  }

  public static Command score(Elevator elevator, Carriage carriage) {
    return elevator
        .enableElevator()
        .until(elevator::atGoalHeight)
        .andThen(carriage.placeCoral())
        .andThen(elevator.disableElevator())
        .withName("score");
  }
}
