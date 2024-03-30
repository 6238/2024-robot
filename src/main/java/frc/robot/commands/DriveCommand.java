package frc.robot.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {

    private final SwerveSubsystem subsys;

    private final DoubleSupplier vX, vY;
    private final DoubleSupplier rotationSpeed;
    private final DoubleSupplier radians;
    private final BooleanSupplier angleControl;

    public DriveCommand(SwerveSubsystem subsys, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rotationSpeed, BooleanSupplier angleControl, DoubleSupplier radians) {
        this.subsys = subsys;

        this.vX = vX;
        this.vY = vY;

        this.rotationSpeed = rotationSpeed;

        this.angleControl = angleControl;
        this.radians = radians;

        addRequirements(this.subsys);

        SmartDashboard.putNumber("aiming_velocity_multiplier", 2);
    }

    @Override
    public void execute() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        double sign = (ally.get() == Alliance.Blue) ? 1.0 : -1.0;
        SmartDashboard.putNumber("yawSetpoint", radians.getAsDouble());
        SmartDashboard.putBoolean("angleControl", angleControl.getAsBoolean());
        // Read from joysticks
        double driveY = Math.pow(vY.getAsDouble(), 1) * OperatorConstants.JOYSTICK_SCALE;
        double driveX = Math.pow(vX.getAsDouble(), 1) * OperatorConstants.JOYSTICK_SCALE;
        double multiplier = SmartDashboard.getNumber("aiming_velocity_multiplier", 1);
        double rotation = angleControl.getAsBoolean() == false ? rotationSpeed.getAsDouble() * Constants.MAX_ANGULAR_VELOCITY : multiplier * subsys.headingCalculate(radians.getAsDouble());

        Translation2d translation = new Translation2d(sign * driveX * subsys.maximumSpeed, sign * driveY * subsys.maximumSpeed);
        subsys.drive(translation, rotation, true);   
    }
}
