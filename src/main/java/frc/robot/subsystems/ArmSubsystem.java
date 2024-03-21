// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import static java.util.Map.entry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    // Initialize control requests
    // Position 0, at rest, no FOC, no FF, use config slot 0, always brake when in
    // deadband, no limiting
    private final PositionVoltage voltagePosition = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
    // Brake request to set when neutral

    private double kP = 0.1;
    private double kI = 0.0001;
    private double kD = 0.005;
    private double setpoint = 0;

    private TalonFXConfiguration configs = new TalonFXConfiguration();

    private final TalonFX motor1 = new TalonFX(11);
    private final TalonFX motor2 = new TalonFX(12);
    private final TalonFX motor3 = new TalonFX(13);

    private final TalonSRX sensorTalon = new TalonSRX(14);

    // TODO: test and optimize these
    private static final Map<ArmStates, Double> ANGLES = Map.ofEntries(
        entry(ArmStates.INTAKE, 17.0),
        entry(ArmStates.SHOOT, 24.0),
        entry(ArmStates.STOW, 75.0));

    /** Creates a new ExampleSubsystem. */
    public ArmSubsystem() {
        
        double motorToArm = .2155; // motor turns needed for the arm to turn fully

        configs.Feedback.SensorToMechanismRatio = motorToArm;

        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);

        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 3;
        configs.Voltage.PeakReverseVoltage = -3;


        // Peak output of 130 amps
        // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // configs.CurrentLimits.SupplyCurrentThreshold = 1;
        // configs.CurrentLimits.SupplyTimeThreshold = .01;
        // configs.CurrentLimits.SupplyCurrentLimit = 0;

        // Set motors 2 and 3 to follow motor 1
        motor2.setControl(new Follower(motor1.getDeviceID(), false));
        motor2.setNeutralMode(NeutralModeValue.Brake);
        motor3.setControl(new Follower(motor1.getDeviceID(), false));
        motor3.setNeutralMode(NeutralModeValue.Brake);
        
        SmartDashboard.putNumber("regressionA", -6.58156);
        SmartDashboard.putNumber("regressionB", 41.555);
        SmartDashboard.putNumber("regressionC", -1.55917);

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor1.getConfigurator().apply(configs);
            status = motor2.getConfigurator().apply(configs);
            status = motor3.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        // Figure out our starting position in degrees, and set the Falcon's onboard encoders to correspond
        sensorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        sensorTalon.configFeedbackNotContinuous(true, 0);
        double startingPosition = (sensorTalon.getSelectedSensorPosition() - Constants.ARM_ENCODER_ZERO) * 360 / 4096;
        motor1.setPosition(startingPosition);

        // Set the arm setpoint to be the same as the starting position so that the arm doesn't move until we explicitly tell it to go somewhere else
        setpoint = startingPosition;
        SmartDashboard.putNumber("setpoint", setpoint);
    }

    public double getArmAngle(double dist) {
        double a = SmartDashboard.getNumber("regressionA", -2.34149);
        double b = SmartDashboard.getNumber("regressionB", 20.0303);
        double c = SmartDashboard.getNumber("regressionC", 4.88624);
        double angle = a * Math.pow(dist, 2) + b * dist + c;
        if (angle > 19 && angle < 110) {
            setpoint = angle;
            return angle;
        }
        else {
            return 75;
        }
    }

    public Command runPIDCommand() {
        return runOnce(() -> {
            motor1.setControl(voltagePosition.withPosition(setpoint));
        });
    }

    public void autoSetAngle(DoubleSupplier poseX, DoubleSupplier poseY) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        double speakerX = (ally.get() == Alliance.Blue) ? 0.0 : 16.579342;
        double dist = Math.hypot(poseY.getAsDouble() - 5.547868, poseX.getAsDouble() - speakerX);
        SmartDashboard.putNumber("distFromSpeaker", dist);
        motor1.setControl(voltagePosition.withPosition(getArmAngle(dist)));
    }

    public void runPID() {
        motor1.setControl(voltagePosition.withPosition(setpoint));
    }

    public Command increaseSetpointCommand() {
        return runOnce(() -> {
            this.setpoint += 5; // one more motor turn
        });
    }

    public Command decreaseSetpointCommand() {
        return Commands.runOnce(() -> {
            this.setpoint -= 5; // one more motor turn
        });
    }

    public void setAngle(Double angle) {
        this.setpoint = angle;
    }

    public Command setAngleCommand(Double angle) {
        return Commands.runOnce(() -> {
            this.setpoint = angle;
        });
        // don't use this.runOnce because it implicitly requires this, which is not what
        // we want (don't stop the loop to change the setpt)
    }

    public Command setAngleCommand(ArmStates state){
        return Commands.runOnce(() -> {
            this.setpoint = ANGLES.get(state);
        });
        // don't use this.runOnce because it implicitly requires this, which is not what
        // we want (don't stop the loop to change the setpt)
    }
    public Command setAngleCommand(DoubleSupplier angle) {
        return Commands.runOnce(() -> {
            this.setpoint = angle.getAsDouble();
        });
        // don't use this.runOnce because it implicitly requires this, which is not what
        // we want (don't stop the loop to change the setpt)
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Pull PD values from dashboard
        this.kP = SmartDashboard.getNumber("kP", kP);
        this.kI = SmartDashboard.getNumber("kI", kI);
        this.kD = SmartDashboard.getNumber("kD", kD);
        SmartDashboard.putNumber("setpoint", setpoint);

        // this.setpoint = SmartDashboard.getNumber("setpoint", setpoint);

        // Update our PID controller
        configs.Slot0.kP = this.kP;
        configs.Slot0.kI = this.kI;
        configs.Slot0.kD = this.kD;
        // motor1.getConfigurator().apply(configs);
        //motor1.getConfigurator().apply(configs.Slot0);

        SmartDashboard.putNumberArray("motor current draw", new double[] { motor1.getTorqueCurrent().getValue(),
                motor2.getTorqueCurrent().getValue(), motor3.getTorqueCurrent().getValue() });

        SmartDashboard.putNumber("sensor value in turns", sensorTalon.getSelectedSensorPosition() * (1.0 / 4096.0));
        SmartDashboard.putNumber("raw sensor value", sensorTalon.getSelectedSensorPosition());
        SmartDashboard.putNumber("falcon position", motor1.getPosition().getValue());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * States the arm can be in
     */
    public enum ArmStates {
        /** Lowered for intaking */
        INTAKE,
        /** Stowed - entirely within frame */
        STOW,
        /** Raised to shoot in speaker */
        SHOOT
    }
}
