package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

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
  private double filteredVelocity;
  private double filteredStatorCurrent;
  public static boolean simHasAlgae = false;
  // In rotations
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Intake/loweredAngle", 55);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Intake/raisedAngle", 0);
  // In volts
  private static final LoggedTunableNumber rollersSpeedIn =
      new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 6);
  private static final LoggedTunableNumber rollersSpeedOut =
      new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", 4);
  private static final LoggedTunableNumber holdSpeedVolts =
      new LoggedTunableNumber("Carriage/holdSpeedVolts", 0.5);

  private double positionSetpoint = raisedAngle.get();

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Logs
    measuredVisualizer.update(Degrees.of(inputs.pivotCurrentPositionDeg));
    setpointVisualizer.update(Degrees.of(positionSetpoint));
    Logger.recordOutput("Intake/positionSetpointRadians", Units.degreesToRadians(positionSetpoint));
    Util.logSubsystem(this, "Intake");
  }

  private void lower() {
    positionSetpoint = loweredAngle.get();
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = raisedAngle.get();
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
