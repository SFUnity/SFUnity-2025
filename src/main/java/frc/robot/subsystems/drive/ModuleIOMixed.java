// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOMixed implements ModuleIO {
  private final TalonFX driveTalon;
  private final SparkMax turnSparkMax;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder cancoder;

  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;

  private final double DRIVE_GEAR_RATIO = 6.12244897959;
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isDriveMotorInverted;
  private final boolean isTurnMotorInverted;
  private final boolean isCancoderInverted;
  private final double absoluteEncoderOffseRot; // TODO tune

  public ModuleIOMixed(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(0);
        turnSparkMax = new SparkMax(0, MotorType.kBrushless);
        cancoder = new CANcoder(0);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 1:
        driveTalon = new TalonFX(1);
        turnSparkMax = new SparkMax(1, MotorType.kBrushless);
        cancoder = new CANcoder(1);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 2:
        driveTalon = new TalonFX(2);
        turnSparkMax = new SparkMax(2, MotorType.kBrushless);
        cancoder = new CANcoder(2);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      case 3:
        driveTalon = new TalonFX(3);
        turnSparkMax = new SparkMax(3, MotorType.kBrushless);
        cancoder = new CANcoder(3);
        absoluteEncoderOffseRot = 0.0; // MUST BE CALIBRATED
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        isCancoderInverted = false;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(isTurnMotorInverted)
        .smartCurrentLimit(60);
    turnConfig.encoder
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(2);
    configureTurnSMax(turnConfig);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnRelativeEncoder.setPosition(0.0);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());
    CANcoderConfiguration encConfig = new CANcoderConfiguration();
    encConfig.MagnetSensor.SensorDirection =
        isCancoderInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    encConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffseRot;
    cancoder.getConfigurator().apply(encConfig);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent)
            .isOK();
    inputs.cancoderConnected = BaseStatusSignal.refreshAll(turnAbsolutePosition).isOK();

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = turnSparkMax.getOutputCurrent();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isDriveMotorInverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    configureTurnSMax(config);
  }

  private void configureTurnSMax(SparkMaxConfig config) {
    turnSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
