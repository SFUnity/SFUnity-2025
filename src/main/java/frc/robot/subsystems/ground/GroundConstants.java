package frc.robot.subsystems.ground;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class GroundConstants {
  public static final LoggedTunableNumber kP;
  public static final double pivotPositionFactor = 0;
  public static final double minAngleRads = Units.degreesToRadians(18.39);
  public static final double maxAngleRads = Units.degreesToRadians(87.39);
  public static final double armLengthMeters = Units.inchesToMeters(15);

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Ground/kP", 2.0);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Ground/simkP", 4.82);
        break;
    }
  }
}
