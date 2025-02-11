package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kRed);
  private final IntakeVisualizer setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kBlue);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double filteredVelocity;
  private double filteredStatorCurrent;
  public static boolean simHasAlgae = false;
  private double positionSetpoint = raisedAngle.get();

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    filteredVelocity = velocityFilter.calculate(Math.abs(inputs.rollerVelocityRPM));
    filteredStatorCurrent = currentFilter.calculate(inputs.rollersCurrentAmps);

    // Logs
    measuredVisualizer.update(inputs.pivotCurrentPosition);
    setpointVisualizer.update(positionSetpoint);
    Logger.recordOutput("Intake/positionSetpointRadians", positionSetpoint.in(Radians));
    Util.logSubsystem(this, "Intake");
  }

  private void lower() {
    positionSetpoint = Degrees.of(loweredAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = Degrees.of(raisedAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runRollers(rollersSpeedIn.get());
  }

  private void rollersOut() {
    io.runRollers(-rollersSpeedOut.get());
  }

  private void rollersStopOrHold() {
    io.runRollers(algaeHeld() ? holdSpeedVolts.get() : 0);
  }

  public Command raiseAndStopOrHoldCmd() {
    return run(() -> {
          raise();
          rollersStopOrHold();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd() {
    return run(() -> {
          lower();
          rollersIn();
        })
        .until(this::algaeHeld)
        .withName("intake");
  }

  public Command poopCmd() {
    return run(() -> {
          raise();
          rollersOut();
        })
        .until(() -> !algaeHeld())
        .withName("poop");
  }

  @AutoLogOutput
  public boolean algaeHeld() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return simHasAlgae;
    }
    return (filteredVelocity <= algaeVelocityThreshold.get()
            && (filteredStatorCurrent >= algaeCurrentThreshold.get())
        || filteredStatorCurrent <= -2);
  }
}
