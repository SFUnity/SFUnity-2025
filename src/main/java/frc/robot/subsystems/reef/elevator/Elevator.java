package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

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
        pid.calculate(inputs.position.in(Meters))
            + ffController.calculate(inputs.velocityMetersPerSec));
  }

  public void stop() {
    io.stop();
  }
  // TODO: fix desired height
  public boolean atDesiredHeight(double desiredHeight) {
    if (!Util.equalsWithTolerance(inputs.position.in(Meters), desiredHeight, 0.15)) {
      return true;
    } else {
      return false;
    }
  }

  public Command l1() {
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

  public Command l2() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightL2);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyL2");
  }

  public Command l3() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightL3);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyL3");
  }

  public Command lowAlgae() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightLowAlgae);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyLowAlgae");
  }

  public Command highAlgae() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightHighAlgae);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyHighAlgae");
  }

  public Command processor() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightProcessor);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readyProcessor");
  }

  public Command source() {
    return run(() -> {
          calculateDesiredPosition(ElevatorConstants.desiredHeightSource);
          runElevator();
        })
        .finallyDo(
            () -> {
              calculateDesiredPosition(ElevatorConstants.desiredHeightBottom);
              runElevator();
            })
        .withName("readySource");
  }
}
