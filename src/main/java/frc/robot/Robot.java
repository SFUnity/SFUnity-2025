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

package frc.robot;

import static frc.robot.RobotCommands.*;
import static frc.robot.RobotCommands.IntakeState.*;
import static frc.robot.RobotCommands.ScoreState.*;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.reefName;
import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.sourceName;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.util.AllianceFlipUtil.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constantsGlobal.BuildConstants;
import frc.robot.constantsGlobal.Constants;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.carriage.CarriageIO;
import frc.robot.subsystems.carriage.CarriageIOSim;
import frc.robot.subsystems.carriage.CarriageIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOMixed;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  // Autos
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  // Alerts
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Carriage carriage;
  private final Intake intake;
  private final AprilTagVision vision;

  // Non-subsystems
  private final PoseManager poseManager = new PoseManager();
  private final Autos autos;

  // Controllers + driving
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.kWarning);

  public boolean slowMode = false;
  private final LoggedTunableNumber slowDriveMultiplier =
      new LoggedTunableNumber("Slow Drive Multiplier", 0.6);
  private final LoggedTunableNumber slowTurnMultiplier =
      new LoggedTunableNumber("Slow Turn Multiplier", 0.5);

  private final DriveCommandsConfig driveCommandsConfig =
      new DriveCommandsConfig(driver, () -> slowMode, slowDriveMultiplier, slowTurnMultiplier);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @SuppressWarnings("resource")
  public Robot() {
    super();

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        Logger.registerURCL(URCL.startExternal()); // Enables REV CAN logging !!! not replayable !!!
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        // Logger.addDataReceiver(new WPILOGWriter()); // for sim logging
        break;

      case REPLAY:
        // In this case when you "Simulate Robot Code" you will enter replay mode
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.

    // Start AdvantageKit logger
    Logger.start();

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    disabledTimer.restart();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOMixed(0),
                new ModuleIOMixed(1),
                new ModuleIOMixed(2),
                new ModuleIOMixed(3),
                poseManager,
                driveCommandsConfig);
        elevator = new Elevator(new ElevatorIOSparkMax());
        carriage = new Carriage(new CarriageIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        vision = new AprilTagVision(poseManager, new AprilTagVisionIOLimelight(reefName), new AprilTagVisionIOLimelight(sourceName));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseManager,
                driveCommandsConfig);
        elevator = new Elevator(new ElevatorIOSim());
        carriage = new Carriage(new CarriageIOSim());
        intake = new Intake(new IntakeIOSim());
        vision = new AprilTagVision(poseManager, new AprilTagVisionIO() {}, new AprilTagVisionIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                poseManager,
                driveCommandsConfig);
        elevator = new Elevator(new ElevatorIO() {});
        carriage = new Carriage(new CarriageIO() {});
        intake = new Intake(new IntakeIO() {});
        vision = new AprilTagVision(poseManager, new AprilTagVisionIO() {}, new AprilTagVisionIO() {});
        break;
    }

    autos = new Autos(drive, poseManager, elevator, carriage);

    // Configure the button bindings
    configureButtonBindings();

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.kInfo).set(true);
    }

    DriverStation.silenceJoystickConnectionWarning(true);

    // For tuning visualizations
    // Logger.recordOutput("ZeroedPose2d", new Pose2d());
    // Logger.recordOutput("ZeroedPose3d", new Pose3d[] {new Pose3d(), new Pose3d()});

    // Set up port forwarding for limelights so we can connect to them through the RoboRIO USB port
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // Check controllers
    driverDisconnected.set(isControllerConnected(driver));
    operatorDisconnected.set(isControllerConnected(operator));

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    // Logs
    Logger.recordOutput("Controls/intakeState", intakeState.toString());
    Logger.recordOutput("Controls/scoreState", scoreState.toString());
    Logger.recordOutput("Controls/dealgifyAfterPlacing", dealgifyAfterPlacing);
    Logger.recordOutput("Controls/allowAutoRotation", allowAutoRotation);
    Logger.recordOutput("Controls/goalPose", goalPose(poseManager).get());
  }

  private boolean isControllerConnected(CommandXboxController controller) {
    return controller.isConnected()
        && DriverStation.getJoystickIsXbox(
            controller.getHID().getPort()); // Should be an XBox controller
  }

  private boolean allowAutoRotation = true;

  // Consider moving to its own file if/when it gets big
  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Setup rumble
    new Trigger(() -> intake.algaeHeld())
        .onTrue(Commands.run(() -> driver.setRumble(RumbleType.kBothRumble, 0.5)).withTimeout(.5));

    // Default cmds
    drive.setDefaultCommand(drive.joystickDrive());
    elevator.setDefaultCommand(elevator.disableElevator());
    carriage.setDefaultCommand(carriage.stop());
    intake.setDefaultCommand(intake.raiseAndStopCmd());

    // Driver controls
    driver.leftTrigger().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driver
        .y()
        .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(0)).until(drive::thetaAtGoal));
    driver
        .b()
        .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(90)).until(drive::thetaAtGoal));
    driver
        .a()
        .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(180)).until(drive::thetaAtGoal));
    driver
        .x()
        .onTrue(drive.headingDrive(() -> Rotation2d.fromDegrees(270)).until(drive::thetaAtGoal));
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        poseManager.setPose(
                            new Pose2d(poseManager.getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driver
        .back()
        .onTrue(
            Commands.runOnce(() -> allowAutoRotation = !allowAutoRotation).ignoringDisable(true));

    driver.leftBumper().whileTrue(fullIntake(drive, carriage, intake, poseManager));
    driver
        .rightBumper()
        .whileTrue(
            fullScore(drive, elevator, carriage, intake, poseManager, driver.rightBumper())
                .beforeStarting(
                    () -> {
                      if (!intake.algaeHeld() && !carriage.algaeHeld() && !carriage.coralHeld())
                        scoreState = Dealgify;
                    }));

    // Operator controls
    operator.y().onTrue(elevator.request(L3));
    operator.x().onTrue(elevator.request(L2));
    operator.a().onTrue(elevator.request(L1));
    operator
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (intake.algaeHeld()) {
                    scoreState = ProcessorBack;
                  } else if (carriage.algaeHeld()) {
                    scoreState = ProcessorFront;
                  }
                }));
    operator.leftBumper().onTrue(Commands.runOnce(() -> scoreState = LeftBranch));
    operator.rightBumper().onTrue(Commands.runOnce(() -> scoreState = RightBranch));
    operator
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> dealgifyAfterPlacing = !dealgifyAfterPlacing));

    operator.povUp().onTrue(Commands.runOnce(() -> intakeState = Source));
    operator.povRight().onTrue(Commands.runOnce(() -> intakeState = Ice_Cream));
    operator.povDown().onTrue(Commands.runOnce(() -> intakeState = Ground));

    // State-Based Triggers
    new Trigger(carriage::coralHeld)
        .and(() -> allowAutoRotation)
        .whileTrue(drive.headingDrive(() -> poseManager.getHorizontalAngleTo(apply(reefCenter))));
    new Trigger(carriage::algaeHeld).onTrue(Commands.runOnce(() -> scoreState = ProcessorFront));
    new Trigger(intake::algaeHeld).onTrue(Commands.runOnce(() -> scoreState = ProcessorBack));

    // Sim fake gamepieces
    SmartDashboard.putData(
        "Toggle Coral in Carriage",
        Commands.runOnce(() -> Carriage.simHasCoral = !Carriage.simHasCoral));
    SmartDashboard.putData(
        "Toggle Algae in Carriage",
        Commands.runOnce(() -> Carriage.simHasAlgae = !Carriage.simHasAlgae));
    SmartDashboard.putData(
        "Toggle Algae in Intake", Commands.runOnce(() -> Intake.simHasAlgae = !Intake.simHasAlgae));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link Autos} class. */
  @Override
  public void autonomousInit() {
    autoCommand = autos.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
