package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public Angle position;
    public double velocityRotsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runVolts(double volts) {}

  public default void stop() {}

  public default void getEncoder() {}
}

// Calculate
// updateInputs
// setHeight
// setPID
// setFF
