package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleHardwareConfig;
import frc.robot.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ModuleIOKraken implements ModuleIO {
    private TalonFX driveMotor;
    private VelocityVoltage driveControl = new VelocityVoltage(0.0);
    private VoltageOut driveVoltageControl = new VoltageOut(0.0);
    private double driveAppliedVolts = 0.0;

    private StatusSignal<Angle> drivePositionM;
    private StatusSignal<AngularVelocity> driveVelocityMPS;
    private StatusSignal<Voltage> driveVoltage;
    private StatusSignal<Current> driveSupplyCurrent;
    private StatusSignal<Current> driveStatorCurrent;
    private StatusSignal<Current> driveTorqueCurrent;
    private StatusSignal<Temperature> driveTempCelsius;
    private StatusSignal<AngularAcceleration> driveAccelerationMPSS;

    private TalonFX azimuthMotor;
    private VoltageOut azimuthVoltageControl = new VoltageOut(0.0);
    private double azimuthAppliedVolts = 0.0;

    private StatusSignal<Angle> azimuthPosition;
    private StatusSignal<AngularVelocity> azimuthVelocity;
    private StatusSignal<Voltage> azimuthVoltage;
    private StatusSignal<Current> azimuthStatorCurrent;
    private StatusSignal<Current> azimuthSupplyCurrent;
    // private StatusSignal<Current> azimuthTorqueCurrent;
    private StatusSignal<Temperature> azimuthTemp;

    private CANcoder absoluteEncoder;
    private StatusSignal<Angle> absolutePositionSignal;
    private Rotation2d absoluteEncoderOffset;

    public ModuleIOKraken(ModuleHardwareConfig config) {
        /* DRIVE INSTANTIATION AND CONFIGURATION */
        driveMotor = new TalonFX(config.driveID(), Constants.kCanbusName);
        var driveConfig = new TalonFXConfiguration();
        
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = kDriveStatorAmpLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyAmpLimit;

        // foc
        // driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        // driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        // driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        driveConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        driveConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        driveConfig.MotorOutput.NeutralMode = DriveConstants.kNeutralVal;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearing / kWheelCircumferenceMeters;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearing;

        driveConfig.Slot0.kP = kModuleControllerConfigs.driveController().getP();
        driveConfig.Slot0.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot0.kD = kModuleControllerConfigs.driveController().getD();
        drivePositionM = driveMotor.getPosition();
        driveVelocityMPS = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        driveStatorCurrent = driveMotor.getStatorCurrent();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();
        driveTempCelsius = driveMotor.getDeviceTemp();
        driveAccelerationMPSS = driveMotor.getAcceleration();

        driveMotor.getConfigurator().apply(driveConfig);

        /* CANCODER INSTANTIATION AND CONFIGURATION */
        absoluteEncoderOffset = config.offset();
        absoluteEncoder = new CANcoder(config.encoderID(), Constants.kCanbusName);
        absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        var encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        //encoderConfig.MagnetSensor.withMagnetOffset(absoluteEncoderOffset.getRotations());

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePositionSignal);
        absoluteEncoder.optimizeBusUtilization();

        /* AZIMUTH INSTANTIATION AND CONFIGURATION */
        azimuthMotor = new TalonFX(config.azimuthID(), Constants.kCanbusName);
        var turnConfig = new TalonFXConfiguration();
        azimuthMotor.getConfigurator().apply(turnConfig);

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = kAzimuthStatorAmpLimit;

        turnConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        turnConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = kTurnMotorInvert ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
            
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.Feedback.RotorToSensorRatio = 1.0;
        turnConfig.Feedback.SensorToMechanismRatio = kAzimuthMotorGearing;

        turnConfig.Slot0.kP = kModuleControllerConfigs.azimuthController().getP();
        turnConfig.Slot0.kD = kModuleControllerConfigs.azimuthController().getD();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Configured but FOC not used on azimuth, just drive motors */
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = kAzimuthFOCAmpLimit;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kAzimuthFOCAmpLimit;

        resetAzimuthEncoder();

        absoluteEncoder.getConfigurator().apply(encoderConfig);
        azimuthMotor.getConfigurator().apply(turnConfig);

        azimuthPosition = azimuthMotor.getPosition();
        azimuthVelocity = azimuthMotor.getVelocity();
        azimuthVoltage = azimuthMotor.getMotorVoltage();
        azimuthStatorCurrent = azimuthMotor.getStatorCurrent();
        azimuthSupplyCurrent = azimuthMotor.getSupplyCurrent();
        // azimuthTorqueCurrent = azimuthMotor.getTorqueCurrent();
        azimuthTemp = azimuthMotor.getDeviceTemp();

    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.isDriveConnected = BaseStatusSignal.refreshAll(
                driveVelocityMPS,
                drivePositionM,
                driveVoltage,
                driveSupplyCurrent,
                driveStatorCurrent,
                driveTorqueCurrent,
                driveTempCelsius).isOK();

        inputs.drivePositionM = (drivePositionM.getValueAsDouble());
        inputs.driveVelocityMPS = (driveVelocityMPS.getValueAsDouble()  * DriveConstants.kWheelCircumferenceMeters);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveMotorVolts = driveVoltage.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
        inputs.driveTemperatureCelsius = driveTempCelsius.getValueAsDouble();
        inputs.driveAccelerationMPSS = driveAccelerationMPSS.getValueAsDouble();

        inputs.isAzimuthConnected = BaseStatusSignal.refreshAll(
            azimuthVelocity,
            azimuthVoltage,
            azimuthStatorCurrent,
            azimuthSupplyCurrent,
            azimuthTemp,
            azimuthPosition).isOK();
        inputs.azimuthPosition = Rotation2d.fromRotations(azimuthPosition.getValueAsDouble());
        inputs.azimuthVelocity = Rotation2d.fromRotations(azimuthVelocity.getValueAsDouble());
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
        inputs.azimuthStatorCurrentAmps = azimuthStatorCurrent.getValueAsDouble();
        inputs.azimuthSupplyCurrentAmps = azimuthSupplyCurrent.getValueAsDouble();
        // inputs.azimuthTorqueCurrentAmps = azimuthTorqueCurrent.getValueAsDouble();
        inputs.azimuthTemperatureCelsius = azimuthTemp.getValueAsDouble();

        inputs.isCancoderConnected = BaseStatusSignal.refreshAll(absolutePositionSignal).isOK();
        inputs.azimuthAbsolutePosition = Rotation2d.fromRotations(absolutePositionSignal.getValueAsDouble()).minus(absoluteEncoderOffset);
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        /* Sets drive voltage inbetween kPeakVoltage and -kPeakVoltage */
        driveMotor.setControl(driveVoltageControl.withOutput(volts));
    }

    @Override
    public void setDriveAmperage(double amps) {
        /* Sets drive amperage inbetween kDriveFOCAmpLimit and -kDriveFOCAmpLimit */
        driveMotor.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        /* Uses FOC PID with a arbitrary FF on the with Slot 0 gains */
        driveMotor.setControl(driveControl
            .withVelocity((velocityMPS / kWheelCircumferenceMeters) * DriveConstants.kDriveMotorGearing) //TODO: THIS MIGHT BE THE PROBLEM!!!!  Understanding Mechanism Rotations vs Rotor Rotations
            .withFeedForward(feedforward));
    }

    /* Sets azimuth PID values on slot 0, used for tunable numbers */
    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        driveMotor.getConfigurator().apply(slotConfig);
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        /* Sets azimuth encoder rotation using CANCoder */
        azimuthMotor.setPosition(Rotation2d.fromRotations(
            absoluteEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(absoluteEncoderOffset).getRotations());
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        /* Sets azimuth voltage inbetween kPeakVoltage and -kPeakVoltage */
        azimuthMotor.setControl(azimuthVoltageControl.withOutput(volts));
    }

    @Override
    public void setAzimuthPosition(Rotation2d rotation, double feedforward) {   
        /* Uses voltage PID with a arbitrary FF on the with Slot 0 gains */
        azimuthMotor.setControl(new PositionVoltage(rotation.getRotations())
            .withFeedForward(0)
            .withSlot(0));
    }

    /* Sets azimuth PID values on slot 0, used for tunable numbers */
    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        azimuthMotor.getConfigurator().apply(slotConfig);
    }

    
    public void setNeutralMode(NeutralModeValue neutralMode) {
        driveMotor.setNeutralMode(neutralMode);
    }
}
