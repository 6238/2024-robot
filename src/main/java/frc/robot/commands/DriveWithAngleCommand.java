package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithAngleCommand extends Command {

    private final SwerveSubsystem subsys;

    private final DoubleSupplier vX, vY;
    private final double radians;


    public DriveWithAngleCommand(SwerveSubsystem subsys, DoubleSupplier vX, DoubleSupplier vY, double radians) {
        this.subsys = subsys;

        this.vX = vX;
        this.vY = vY;

        this.radians = radians;

        addRequirements(this.subsys);
    }

    @Override
    public void execute() {
        // Read from joysticks
        double driveY = Math.pow(vY.getAsDouble(), 1) * OperatorConstants.JOYSTICK_SCALE;
        double driveX = Math.pow(vX.getAsDouble(), 1) * OperatorConstants.JOYSTICK_SCALE;

        subsys.driveFieldOriented(subsys.getTargetSpeeds(driveX * subsys.maximumSpeed,
                                                        driveY * subsys.maximumSpeed,
                                                        new Rotation2d(radians)));                                        
    }
}
