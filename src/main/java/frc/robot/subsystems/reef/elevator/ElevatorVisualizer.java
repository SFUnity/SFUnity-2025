package frc.robot.subsystems.reef.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d elevator;
  private final String key;

  public ElevatorVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kBlack));
    MechanismRoot2d root = mechanism.getRoot("elevator", 1.5, 2.0);
    elevator =
        new MechanismLigament2d(
            "elevator", ElevatorConstants.elevatorMinLength, 0, 6, new Color8Bit(color));
    root.append(elevator);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double heightMeters) {
    // Log Mechanism2d
    elevator.setLength(heightMeters);
    // Logger.recordOutput("Reef/Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d elevator =
        new Pose3d(
            ElevatorConstants.elevatorOrigin.getX(),
            heightMeters,
            ElevatorConstants.elevatorOrigin.getY(),
            new Rotation3d(0, 0, 0.0));
    Logger.recordOutput("Reef/Elevator/Mechanism3d/" + key, elevator);
  }
}
