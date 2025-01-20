package frc.robot.subsystems.reef;

import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightHighAlgae;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL1;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL2;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightL3;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightLowAlgae;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightProcessor;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.desiredHeightSource;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.reef.elevator.Elevator;
import frc.robot.subsystems.reef.rollers.Rollers;

public class Reef extends SubsystemBase {
  private final Rollers rollers;
  private final Elevator elevator;

  public Reef(Rollers rollers, Elevator elevator) {
    this.rollers = rollers;
    this.elevator = elevator;
  }

  public void periodic() {}

  public boolean atDesiredHeight(double desiredHeight) {
    return elevator.atDesiredHeight(desiredHeight);
  }

  public Command setCoralIntake() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightSource)))
        .andThen(rollers.intakeCoral())
        .withName("intakeCoral");
  }

  public Command setScoreL1() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL1)))
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL1");
  }

  public Command setScoreL2() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL2)))
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL2");
  }

  public Command setScoreL3() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightL3)))
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("scoreL3");
  }

  public Command setDealgaeLow() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightLowAlgae)))
        .andThen(rollers.lowDealgaefy())
        .withName("dealgaefyLow");
  }

  public Command setDealgaeHigh() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightHighAlgae)))
        .andThen(rollers.placeCoralAndHighDealgify())
        .withName("dealgaefyHigh");
  }

  public Command setScoreProcessor() {
    return elevator
        .source()
        .alongWith(Commands.waitUntil(() -> atDesiredHeight(desiredHeightProcessor)))
        .andThen(rollers.scoreProcessor())
        .withName("");
  }
}
