package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  public static final int elevatorMotorID = 0;
  public static final double maxElevatorSpeed = 5;
  public static final double maxElevatorAcceleration = 10;

  public static final double desiredHeightL1 = 0.5;
  public static final double desiredHeightL2 = 1.5;
  public static final double desiredHeightL3 = 2;
  public static final double desiredHeightHighAlgae = 1;
  public static final double desiredHeightLowAlgae = 1.75;
  public static final double desiredHeightProcessor = 0;
  public static final double desiredHeightSource = 2;

  public static final double carrageMassKg = Units.lbsToKilograms(15);
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4);
  public static final double minHeightInches = 0;
  public static final double maxHeightInches = 23.0;

  public static final double gearRatio = 0;
  public static final double wheelRadius = 1;

  public static final double elevatorMinLength = 0;

  public static final Translation2d elevatorOrigin = new Translation2d(0, 0);

  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kD;
  public static final LoggedTunableNumber kG;
  public static final LoggedTunableNumber kV;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 150.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 17.53);
        kG = new LoggedTunableNumber("Reef/Elevator/kG", 0.11591);
        kV = new LoggedTunableNumber("Reef/Elevator/kV", 11.3);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Reef/Elevator/kP", 40.0);
        kD = new LoggedTunableNumber("Reef/Elevator/kD", 0.0);
        kG = new LoggedTunableNumber("Reef/Elevator/kG", 0.06);
        kV = new LoggedTunableNumber("Reef/Elevator/kV", 12.6);
        break;
    }
  }
}
