package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public Distance position = Distance.ofBaseUnits(0, Meters);

    public double velocityMetersPerSec;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVolts(double volts) {}
}
