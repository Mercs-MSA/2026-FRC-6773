// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutonCommands;
// import frc.robot.RobotManager.IntakeManagerState;
// import frc.robot.RobotManager.RobotScoringState;
// import frc.robot.commands.AutonCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopCommands;
// import frc.robot.commands.TeleopCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerKickerIOSim;
import frc.robot.subsystems.indexer.IndexerKickerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSpindexerIOSim;
import frc.robot.subsystems.indexer.IndexerSpindexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.ShooterFlywheelIOTalonFX;
import frc.robot.subsystems.shooter.ShooterHoodIOSim;
import frc.robot.subsystems.shooter.ShooterHoodIOTalonFX;
import frc.robot.subsystems.shooter.ShooterTurretIOSim;
import frc.robot.subsystems.shooter.ShooterTurretIOTalonFX;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.transfer.TransferIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Vision vision;
  private final Indexer indexer;
  private final Transfer transfer;
  private final Intake intake;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutonCommands autonCommands;
  private final TeleopCommands teleopCommands;

  //   private RobotManager manager;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.FrontLeft),
                new ModuleIOTalonFX(DriveConstants.FrontRight),
                new ModuleIOTalonFX(DriveConstants.BackLeft),
                new ModuleIOTalonFX(DriveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
        shooter =
            new Shooter(
                new ShooterFlywheelIOTalonFX(
                    ShooterConstants.flywheelHardware,
                    ShooterConstants.flywheelConfigs,
                    ShooterConstants.flywheelGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                new ShooterTurretIOTalonFX(
                    ShooterConstants.turretHardware,
                    ShooterConstants.turretConfigs,
                    ShooterConstants.turretGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                new ShooterHoodIOTalonFX(
                    ShooterConstants.hoodHardware,
                    ShooterConstants.hoodConfigs,
                    ShooterConstants.hoodGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                drive::getPose,
                drive::getFieldVelocity);
        intake =
            new Intake(
                new IntakeRollerIOTalonFX(
                    IntakeConstants.rollerHardware,
                    IntakeConstants.kRollerMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz),
                new IntakePivotIOTalonFX(
                    IntakeConstants.pivotHardware,
                    IntakeConstants.pivotGains,
                    IntakeConstants.kPivotMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz));
        indexer =
            new Indexer(
                new IndexerSpindexerIOTalonFX(
                    IndexerConstants.spindexerHardware,
                    IndexerConstants.spindexerGains,
                    IndexerConstants.spindexerTalonFXConfiguration,
                    IndexerConstants.statusSignalUpdateFrequencyHz),
                new IndexerKickerIOTalonFX(
                    IndexerConstants.kickerHardware,
                    IndexerConstants.kickerGains,
                    IndexerConstants.kickerTalonFXConfiguration,
                    IndexerConstants.statusSignalUpdateFrequencyHz));
        transfer =
            new Transfer(
                new TransferIOTalonFX(
                    TransferConstants.transferHardware,
                    TransferConstants.transferGains,
                    TransferConstants.transferTalonFXConfiguration,
                    TransferConstants.statusSignalUpdateFrequencyHz));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FrontLeft),
                new ModuleIOSim(DriveConstants.FrontRight),
                new ModuleIOSim(DriveConstants.BackLeft),
                new ModuleIOSim(DriveConstants.BackRight));

        shooter =
            new Shooter(
                new ShooterFlywheelIOSim(
                    0.02,
                    ShooterConstants.flywheelHardware,
                    ShooterConstants.shooterFlywheelSimConfig),
                new ShooterTurretIOSim(
                    0.02, ShooterConstants.turretHardware, ShooterConstants.shooterTurretSimConfig),
                new ShooterHoodIOSim(
                    0.02, ShooterConstants.hoodHardware, ShooterConstants.shooterHoodSimConfig),
                drive::getPose,
                drive::getFieldVelocity);
        intake =
            new Intake(
                new IntakeRollerIOSim(
                    0.02,
                    IntakeConstants.rollerHardware,
                    IntakeConstants.rollerSimulationConfiguration),
                new IntakePivotIOSim(
                    0.02,
                    IntakeConstants.pivotHardware,
                    IntakeConstants.pivotSimulationConfiguration));
        indexer =
            new Indexer(
                new IndexerSpindexerIOSim(
                    0.02,
                    IndexerConstants.spindexerHardware,
                    IndexerConstants.spindexerSimulationConfiguration),
                new IndexerKickerIOSim(
                    0.02,
                    IndexerConstants.kickerHardware,
                    IndexerConstants.kickerSimulationConfiguration));
        transfer =
            new Transfer(
                new TransferIOSim(
                    0.02,
                    TransferConstants.transferHardware,
                    TransferConstants.transferSimulationConfiguration));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                // RobotState.getInstance()::addVisionObservation,
                drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        intake = new Intake(null, null);
        indexer = new Indexer(null, null);
        transfer = new Transfer(null);
        shooter = new Shooter(null, null, null, null, null);
        break;
    }
    // manager = new RobotManager(drive, intake, indexer, transfer, shooter);
    teleopCommands = new TeleopCommands(drive, intake, indexer, transfer, shooter);
    autonCommands = new AutonCommands(drive, intake, indexer, transfer, shooter);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Left Path", autonCommands.getAutonomousSequence("LEFT_45"));
    autoChooser.addOption("Right Path", autonCommands.getAutonomousSequence("RIGHT_45"));
    autoChooser.addOption("Left Across Path", autonCommands.getAutonomousSequence("LEFT_FULL"));
    autoChooser.addOption("Right Across Path", autonCommands.getAutonomousSequence("RIGHT_FULL"));
    autoChooser.addOption("Right TEST Path", autonCommands.getAutonomousSequence("RIGHT_TEST"));
    autoChooser.addOption("Left TEST Path", autonCommands.getAutonomousSequence("LEFT_TEST"));
    autoChooser.addOption(
        "Right FULL TEST Path", autonCommands.getAutonomousSequence("RIGHT_FULL_TEST"));
    autoChooser.addOption(
        "Left FULL TEST Path", autonCommands.getAutonomousSequence("LEFT_FULL_TEST"));

    // autoChooser.addOption("Test Path", autonCommands.getPathCommand("TuningPath"));
    // autoChooser.addOption("Center Bump Path", autonCommands.getAutonomousSequence("CENTER"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DriveCommands.setDriveState(drive, DriveState.DRIVING);
    drive.setDriveState(DriveState.DRIVING);
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveXLock(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .leftStick()
        .onTrue(Commands.runOnce(() -> drive.setDriveState(DriveState.ALIGN)))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    drive.interpolateAngle(
                        new Pose2d(
                            drive.getPose().getX(), drive.getPose().getY(), Rotation2d.kZero),
                        new Pose2d(
                            AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX()),
                            AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.getY()),
                            Rotation2d.kZero))
                // .plus(new Rotation2d(Math.PI))
                ))
        .onFalse(Commands.runOnce(() -> drive.setDriveState(DriveState.DRIVING)));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // controller.rightTrigger().whileTrue(shooter.startShooter()).onFalse(shooter.idleShooter());
    controller
        .rightTrigger()
        .whileTrue(teleopCommands.shootCommand())
        .onFalse(teleopCommands.stopShootCommand());
    controller
        .leftTrigger()
        .onTrue(teleopCommands.intakeCommand(IntakeState.INTAKING))
        .onFalse(teleopCommands.intakeCommand(IntakeState.IDLE));

    // controller
    //     .rightTrigger(0.1)
    //     .onTrue(teleopCommands.runShoot())
    //     .onFalse(teleopCommands.stopShoot());

    // controller
    //     .leftStick()
    //     .onTrue(teleopCommands.toggleFixedShooting(true))
    //     .onFalse(teleopCommands.toggleFixedShooting(false));

    // controller
    //     .leftTrigger(0.1)
    //     .onTrue(teleopCommands.runIntake())
    //     .onFalse(teleopCommands.stopIntake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return drive;
  }

  //   public void updateManager() {
  //     manager.periodicManager();
  //   }

  //   public void resetState() {
  //     manager.robotState = RobotScoringState.IDLE;
  //     manager.intakeState = IntakeManagerState.IDLE;
  //   }
}
