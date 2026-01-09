// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOKraken;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.debugging.LoggedTunableNumber;
import static frc.robot.subsystems.drive.DriveConstants.*;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    // Define subsystems
    private final Drive robotDrive;
    private final Vision m_Vision;

    
    private Trigger intakeCoralTrigger;
    private Trigger manipulatorCoralTrigger;

    
    // Define other utility classes
    
    private LoggedDashboardChooser<String> autoChooser;
    private LoggedDashboardChooser<Double> speedChooser;
    private LoggedDashboardChooser<NeutralModeValue> bChooser;
    
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /* TODO: Set to true before competition
     please */

    private final boolean useCompetitionBindings = true;

    // Anshul said to use this because he loves event loops
    private final EventLoop teleopLoop = new EventLoop();

    private LoggedTunableNumber startPos = new LoggedTunableNumber("Auton/StartPos (0 = U, 1 = M, 2 = B)", 1);
    private LoggedTunableNumber startReefPos = new LoggedTunableNumber("Auton/StartReefPos (0 = Reef RUp, 1 = Reef R, 2 = Reef D)", 1);
    private LoggedTunableNumber sourcePref = new LoggedTunableNumber("Auton/SourcePref (0 = Source T, 1 = Source B)", 1);
    
    public Field2d goalPoseField = new Field2d();
    public Field2d currPoseField = new Field2d();

    public Pose2d getCurrPose() {
        return robotDrive.getPoseEstimate();
    }

    public RobotContainer() {
        
        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:

                

                robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOKraken(kFrontLeftHardware )),
                    new Module("FR", new ModuleIOKraken(kFrontRightHardware)),
                    new Module("BL", new ModuleIOKraken(kBackLeftHardware  )),
                    new Module("BR", new ModuleIOKraken(kBackRightHardware ))
                }, new GyroIOPigeon2(), null);

                m_Vision = new Vision(new CameraIO[]{
                    new VisionIOLimelight(VisionConstants.camera0Name, () -> robotDrive.getRobotRotation()),
                    // new VisionIOLimelight(VisionConstants.camera1Name, () -> robotDrive.getRobotRotation()),
                });

                robotDrive.setVision(m_Vision);
                break;
            case SIM:

               robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOSim()),
                    new Module("FR", new ModuleIOSim()),
                    new Module("BL", new ModuleIOSim()),
                    new Module("BR", new ModuleIOSim())
                }, new GyroIO() {}, null);

                m_Vision = new Vision(new CameraIO[]{
                    new VisionIOLimelight(VisionConstants.camera0Name, () -> robotDrive.getRobotRotation()),
                    // new VisionIOLimelight(VisionConstants.camera1Name, () -> robotDrive.getRobotRotation()),
                });
                robotDrive.setVision(m_Vision);

                break;
            default:
                
                robotDrive = new Drive( new Module[] {
                    new Module("FL", new ModuleIO() {}),
                    new Module("FR", new ModuleIO() {}),
                    new Module("BL", new ModuleIO() {}),
                    new Module("BR", new ModuleIO() {})
                }, new GyroIO() {}, null);

                m_Vision = new Vision(new CameraIO[]{
                    new VisionIOLimelight(VisionConstants.camera0Name, () -> robotDrive.getRobotRotation()),
                });

                
                break;
        }
        // autonCommands = new AutonCommands(robotDrive, m_Elevator, m_Intake, m_Manipulator);
        // teleopCommands = new TeleopCommands(m_Elevator, m_Intake, m_Manipulator, driverController);

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();
        createAutos();
        speedChooser = new LoggedDashboardChooser<Double>("Teleop/Acceleration Chooser");
        speedChooser.addDefaultOption("NORMAL", 1.0);
        speedChooser.addOption("SLOW", 1.0);
        
        bChooser = new LoggedDashboardChooser<NeutralModeValue>("Drive/BrakeModeChooser");
        bChooser.addDefaultOption("BRAKE", NeutralModeValue.Brake);
        bChooser.addOption("COAST", NeutralModeValue.Coast);


        robotDrive.setDefaultCommand(Commands.run(() -> robotDrive.setDriveState(DriveState.TELEOP), robotDrive));
        //m_Elevator.setDefaultCommand(Commands.run(() -> m_Elevator.setGoal(ElevatorGoal.kStow), m_Elevator));
        //m_Intake.setDefaultCommand(Commands.run(()-> m_Intake.setPivotGoal(IntakePivotGoal.kStow), m_Intake));

        // Pass subsystems to classes that need them for configuration
        robotDrive.acceptJoystickInputs(
            () -> - driverController.getLeftY(),
            () -> - driverController.getLeftX(),
            () -> driverController.getRightX() * (Constants.kCurrentMode == Mode.SIM ? -1 : 1),
            () -> driverController.getHID().getPOV());


        // Create any Dashboard choosers (LoggedDashboardChooser, etc)

        // Configure controls (drivebase suppliers, DriverStation triggers, Button and other Controller bindings)
        configureStateTriggers();
        configureButtonBindings();

        

        
    }

    /* Commands to schedule on telop start-up */
    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            robotDrive.setDriveStateCommand(DriveState.TELEOP)
        );
    }

    public void createAutos() {
        autoChooser = new LoggedDashboardChooser<String>("Autonomous/chooser");

        // Add autos to chooser
        autoChooser.addOption("LEFT", "LEFT");
        autoChooser.addOption("RIGHT", "RIGHT");
        autoChooser.addOption("CENTER", "CENTER");
        autoChooser.addOption("NOTHING", "NOTHING");


        SmartDashboard.putData("Autonomous/AutoChooser", autoChooser.getSendableChooser());
    }

    public Command getAutonomousCommand() {








        //autoChooser.addOption("3PieceTRREEF-Chicken-McNuggets", Commands.sequence(leftThreePiece));

        // autoCommand.addCommands(autonCommands.runAutonScoringSegment(ElevatorState.L4, "ST_TLREEF", SIDE.LEFT));

        // switch ((int)startPos.get()) {
        //     case 0: // U
        //         startCommandName += "STT_";
        //         break;
        //     case 1: // M
        //         startCommandName += "STM_";
        //         break;
        //     case 2: // B
        //         startCommandName += "STB_";
        //         break;
        //     default:
        //         startCommandName += "STM_";
        //         break;
        // }

        // switch ((int)startReefPos.get()) {
        //     case 0: // Reef RUp
        //         startCommandName += "TRREEF";
        //         break;
        //     case 1: // Reef R
        //         startCommandName += "RREEF";
        //         break;
        //     case 2: // Reef D
        //         startCommandName += "BRREEF";
        //         break;
        //     default:
        //         startCommandName += "RREEF";
        //         break;
        // }
        
        // autoCommand.addCommands(autonCommands.runAutonScoringSegment(ElevatorState.L4, startCommandName));
        // startCommandName = startCommandName.split("_")[1];

        // // switch ((int)sourcePref.get()) {
        // //     case 0: // Source T
        // //         startCommandName += "_ST";
        // //         break;
        // //     case 1: // Source B
        // //         startCommandName += "_SB";
        // //         break;
        // //     default:
        // //         startCommandName += "_ST";
        // //         break;
        // // }
        // // autoCommand.addCommands(
        // //     autonCommands.runAutonIntakeSegment(startCommandName)
        // // );

        // // startCommandName = startCommandName.split("_")[1];

        // // switch ((int)sourcePref.get()) {
        // //     case 0: // Source T
        // //         startCommandName += "_TLREEF";
        // //         break;
        // //     case 1: // Source B
        // //         startCommandName += "_BLREEF";
        // //         break;
        // //     default:
        // //         startCommandName += "_TRREEF";
        // //         break;
        // // }

        // // autoCommand.addCommands(autonCommands.runAutonScoringSegment(ElevatorState.L4, startCommandName));

        // // for (int i = 0 ; i < 10; i++) {
        // //     startCommandName = startCommandName.split("_")[1] + "_" + startCommandName.split("_")[0];
        // //     autoCommand.addCommands(autonCommands.followChoreoPath(startCommandName));
        // // }

        return null;//autonCommands.getAutonomousChosen(autoChooser.get());
    }

    public void getAutonomousExit() {
        robotDrive.setDriveState(DriveState.STOP);
    }

    private void configureStateTriggers() {
        /* Due to roborio start up times sometimes modules aren't reset properly, this accounts for that */
        new Trigger(DriverStation::isEnabled)
            .onTrue(
                /* Do not require robot drive or it will deschedule auto */
                Commands.runOnce(() -> robotDrive.resetModulesEncoders()));
    }

    private Command rumbleCommandOperator() {
        return Commands.startEnd(
            () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0), 
            () -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
    }

    private Command rumbleCommandDriver() {
        return Commands.startEnd(
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0), 
            () -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
    }

    private void configureButtonBindings() {

        // intakeCoralTrigger = new Trigger(() -> m_Intake.getCoralDetected());
        // manipulatorCoralTrigger = new Trigger(() -> m_Manipulator.getCoralDetected());
        
        ArrayList<Trigger> positionButtons = new ArrayList<Trigger>();
        positionButtons.add(operatorController.y());
        positionButtons.add(operatorController.b());
        positionButtons.add(operatorController.a());
        positionButtons.add(operatorController.x());




        if (useCompetitionBindings) {

        } 

        else {
            driverController.y().onTrue(Commands.runOnce(() -> robotDrive.resetGyro()));

            driverController.b()
                .onTrue(robotDrive.setDriveStateCommandContinued(DriveState.LINEAR_TEST))
                .onFalse(robotDrive.setDriveStateCommand(DriveState.TELEOP));
            
            
        }
    }

    public EventLoop getTeleopEventLoop() {
        return teleopLoop;
    }



    public void setDriveBrakeMode() {
        robotDrive.setBrakeMode(bChooser.get());
    }

    public void setGyroInit() {
        robotDrive.resetGyro();
    }
    
    public double[] getPoseArr() {
        return new double[]{robotDrive.getPoseEstimate().getMeasureX().baseUnitMagnitude(), robotDrive.getPoseEstimate().getMeasureX().baseUnitMagnitude(), robotDrive.getPoseEstimate().getMeasureX().baseUnitMagnitude()};
    }
}