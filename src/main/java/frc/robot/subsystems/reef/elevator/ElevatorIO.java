package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public Distance position;
    public double velocityMetersPerSec;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}
}

// Calculate
// updateInputs
// setHeight
// setPID
// setFF
