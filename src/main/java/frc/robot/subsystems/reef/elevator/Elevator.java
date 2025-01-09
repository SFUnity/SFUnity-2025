package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxElevatorSpeed, ElevatorConstants.maxElevatorAcceleration));
  private final ElevatorFeedforward ffController =
      new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);
  ;

  private final ElevatorIO io;
  // TODO: Impliment or remove poisemanager
  // private final PoseManager poseManager;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io /*, PoseManager poseManager */) {
    this.io = io;
    // this.poseManager = poseManager;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void calculateDesiredPosition(double desiredPosition) {
    pid.setGoal(desiredPosition);
  }

  public void runElevator() {
    io.runVolts(
        pid.calculate(inputs.position.in(Rotation))
            + ffController.calculate(inputs.velocityRotsPerSec));
  }

  public void stop() {
    io.stop();
  }

  public Command L1() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightL1);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyL1");
  }
}
