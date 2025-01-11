package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.*;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder encoder = elevatorMotor.getEncoder();

  public ElevatorIOSparkMax() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = Meters.of(encoder.getPosition());
    inputs.velocityMetersPerSec =
        (encoder.getVelocity()
                / ElevatorConstants.gearRatio
                * (2 * Math.PI * ElevatorConstants.wheelRadius))
            / 60;
    inputs.appliedVolts = elevatorMotor.getAppliedOutput() * elevatorMotor.getBusVoltage();
    inputs.currentAmps =
        new double[] {elevatorMotor.getOutputCurrent(), elevatorMotor.getOutputCurrent()};
  }

  @Override
  public void runVolts(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    elevatorMotor.stopMotor();
  }
}
