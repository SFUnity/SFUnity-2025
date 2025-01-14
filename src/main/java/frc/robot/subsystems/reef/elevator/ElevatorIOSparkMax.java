package frc.robot.subsystems.reef.elevator;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.reef.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Distance;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder encoder = elevatorMotor.getEncoder();
  private Distance prevoiusPosition;
  private long prevoiusTime;
  private long currentTime;
  private double deltaPosition = 0;
  private double deltaTime = 0;

  public ElevatorIOSparkMax() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    prevoiusTime = currentTime;
    currentTime = System.nanoTime();

    prevoiusPosition = inputs.position;
    inputs.position = Meters.of(encoder.getPosition());
    deltaPosition = inputs.position.in(Meters) - prevoiusPosition.in(Meters);
    deltaTime = (currentTime - prevoiusTime) / 1e9;
    inputs.velocityMetersPerSec = deltaPosition / deltaTime;

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
