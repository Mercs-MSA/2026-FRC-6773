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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
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
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.transfer.KickerIOTalonFX;
import frc.robot.subsystems.transfer.RegulatorIOTalonFX;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
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
  private Vision vision; // make final once we figure out sim
  private final Spindexer spindexer;
  private final Transfer transfer;
  private Intake intake;
  private Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final AutonCommands autonCommands;
  private final TeleopCommands teleopCommands;

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
                (visionRobotPose, timestamp, stds) -> {
                  drive.addVisionMeasurement(visionRobotPose, timestamp, stds);
                  RobotState.getInstance()
                      .addVisionObservation(visionRobotPose, timestamp, stds);
                },
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        intake =
            new Intake(
                new IntakePivotIOTalonFX(
                    IntakeConstants.kPivotMotorHardware,
                    IntakeConstants.kPivotMotorConfiguration,
                    IntakeConstants.kPivotGains,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz),
                new IntakeRollerIOTalonFX(
                    IntakeConstants.kRollerMotorHardware,
                    IntakeConstants.kRollerMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz));

        spindexer =
            new Spindexer(
                new SpindexerIOTalonFX(
                    SpindexerConstants.kSpindexerHardware,
                    SpindexerConstants.kSpindexerConfiguration,
                    SpindexerConstants.kSpindexerGains,
                    SpindexerConstants.kStatusSignalUpdateFrequencyHz));

        transfer =
            new Transfer(
                new KickerIOTalonFX(
                    TransferConstants.kTransferKickerHardware,
                    TransferConstants.kTransferConfiguration,
                    TransferConstants.kStatusSignalUpdateFrequencyHz),
                new RegulatorIOTalonFX(
                    TransferConstants.kTransferRegulatorHardware,
                    TransferConstants.kTransferConfiguration,
                    TransferConstants.kRegulatorGains,
                    TransferConstants.kStatusSignalUpdateFrequencyHz));
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
                drive);
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

        intake =
            new Intake(
                new IntakePivotIOSim(
                    0.02,
                    IntakeConstants.kPivotMotorHardware,
                    IntakeConstants.kPivotSimulationConfiguration,
                    IntakeConstants.kPivotGains),
                new IntakeRollerIOSim(
                    0.02,
                    IntakeConstants.kRollerMotorHardware,
                    IntakeConstants.kIntakeRollerSimulationConfiguration));
        spindexer =
            new Spindexer(
                new SpindexerIOSim(
                    0.01,
                    SpindexerConstants.kSpindexerHardware,
                    SpindexerConstants.kSimulationSpindexerGains,
                    SpindexerConstants.kSpindexerSimulationConfiguration));
        transfer =
            new Transfer(
                new TransferIOSim(
                    0.02,
                    TransferConstants.kTransferKickerHardware,
                    TransferConstants.kSimulationRegulatorGains,
                    TransferConstants.kTransferSimulationConfiguration),
                new TransferIOSim(
                    0.02,
                    TransferConstants.kTransferRegulatorHardware,
                    TransferConstants.kSimulationRegulatorGains,
                    TransferConstants.kTransferSimulationConfiguration));

        shooter =
            new Shooter(
                new ShooterFlywheelIOSim(
                    0.02, ShooterConstants.flywheelHardware, ShooterConstants.shooterSimConfig),
                new ShooterTurretIOSim(
                    0.02, ShooterConstants.turretHardware, ShooterConstants.shooterSimConfig),
                new ShooterHoodIOSim(
                    0.02, ShooterConstants.hoodHardware, ShooterConstants.shooterSimConfig),
                drive);
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
        vision = new Vision(RobotState.getInstance()::addVisionObservation, new VisionIO() {}, new VisionIO() {});

        intake = new Intake(null, null);
        transfer = new Transfer(null, null);
        spindexer = new Spindexer(new SpindexerIOSim(0, null, null, null));
        shooter = new Shooter(null, null, null, null);
        break;
    }
    teleopCommands = new TeleopCommands(intake, spindexer, transfer, shooter, controller);
    autonCommands = new AutonCommands(teleopCommands);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Test Path", autonCommands.getPathCommand("TuningPath"));
    autoChooser.addOption("Center Bump Path", autonCommands.getAutonomousSequence("CENTER"));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveXLock(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    shooter.setDefaultCommand(shooter.shooterDefaultCommand());

    // controller.axisLessThan(4, )

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () ->
                    drive
                        .interpolateAngle(
                            new Pose2d(
                                drive.getPose().getX(), drive.getPose().getY(), Rotation2d.kZero),
                            new Pose2d(4.626, 4.028, Rotation2d.kZero))
                        .plus(new Rotation2d(Math.PI))));

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

    controller
        .leftTrigger(0.25)
        .onTrue(teleopCommands.runIntakeFloorPickup())
        .onFalse(teleopCommands.runIntakeSlowRollers());

    controller
        .rightTrigger()
        .whileTrue(teleopCommands.startShoot())
        // .whileTrue(teleopCommands.whileShooting())
        .onFalse(teleopCommands.stopShoot());

    controller.x().whileTrue(teleopCommands.spinAlt());
    controller.x().whileTrue(teleopCommands.startKick());

    controller.x().onFalse(teleopCommands.spinStop());
    controller.x().onFalse(teleopCommands.stopKick());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
