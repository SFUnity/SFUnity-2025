package frc.robot.subsystems.reef.rollers;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RollersIOSparkMax implements RollersIO {
  private final SparkMax rollerMotor =
      new SparkMax(RollersConstants.rollerMotorID, MotorType.kBrushless);
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

  @Override
  public void runMotorStraight() {
    config.inverted(false);
  }

  @Override
  public void reverseMotor() {
    config.inverted(true);
  }

  @Override
  public void stop() {
    rollerMotor.stopMotor();
  }
}
