package frc.robot.subsystems.reef.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.subsystems.reef.rollers.RollersConstants.*;  

public class RollersIOSparkMax implements RollersIO {
  private final SparkMax rollerMotor =
      new SparkMax(rollerMotorID, MotorType.kBrushless);
  private final SparkMaxConfig config = new SparkMaxConfig();
  private final RelativeEncoder encoder = rollerMotor.getEncoder();

  public RollersIOSparkMax() {}

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    inputs.positionRots = encoder.getPosition();
    inputs.velocityRotsPerSec = encoder.getVelocity();
    inputs.appliedVolts = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.currentAmps = rollerMotor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    rollerMotor.setVoltage(volts);
  }
}
