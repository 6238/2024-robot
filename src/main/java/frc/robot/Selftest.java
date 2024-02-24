package frc.robot;

import java.util.stream.IntStream;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DataLogManager;

public class Selftest {

    // prevent this class from being instantiated, per good OOP style
    private Selftest() {}

    /** 
     * Runs the self-test sequence.
     * @return true if no issues found, otherwise false.
     */
    public static boolean run() {
        boolean good = true;
        // Are all swerve SparkMaxes present, and not reporting faults?
        if (! checkSwerveMotors(IntStream.rangeClosed(20, 27).toArray())) {
            good = false;
        }
       
        // Are the swerve encoders present?

        // Are all three arm TalonFX's present, and not reporting faults?
        if (!checkArmTalons(IntStream.rangeClosed(11, 13).toArray())) {
            good = false;
        }
        // Is the TalonSRX that reads the arm encoder present? Is it reporting any faults?
        if (!checkArmSensorTalon(14)) {
            good = false;
        }
        // Report results
        reportResults(good);
        return good;
    }

    private static boolean checkSwerveMotors(int[] ids) {
        boolean good = true;
        for (int id : ids) {
            DataLogManager.log("Trying to set up swerve motor " + id);
            CANSparkBase motor = new CANSparkMax(id, MotorType.kBrushless);
            REVLibError e = motor.clearFaults();
            if (e != REVLibError.kOk) {
                DataLogManager.log("Error checking swerve motor " + id + ": " + e);
                good = false;
            }

            short faults = motor.getFaults();
            if (faults != 0) {
                DataLogManager.log("Swerve motor " + id + " has faults. Bitstring: " + faults);
                good = false;
            }

            motor.close();
        }
        return good;
    }

    private static boolean checkSwerveEncoders(int[] pins) {
        boolean good = true;
        // TODO write this if we switch to CAN encoders
        return good;
    }

    private static boolean checkArmTalons(int[] ids) {
        boolean good = true;

        for (int id : ids) {
            TalonFX motor = new TalonFX(id);
            if (!motor.isAlive()) {
                DataLogManager.log("Arm TalonFX " + id + " isn't alive!");
                good = false;

                // are there specific sticky faults we want to check for?
                motor.close();
            }
        }

        return good;
    }

    private static boolean checkArmSensorTalon(int id) {
        boolean good = true;
        TalonSRX talon = new TalonSRX(id);
        if (talon.clearStickyFaults() != ErrorCode.OK) {
                DataLogManager.log("Sensor TalonSRX " + id + " isn't alive!");
                good = false;

                // are there specific sticky faults we want to check for?
                talon.DestroyObject();
            }
        return good;
    }

    private static void reportResults(boolean good) {
        CANdle candle = new CANdle(30);
        // TalonFX band = new TalonFX(13);
        // Orchestra orchestra = new Orchestra();
        // orchestra.addInstrument(band);

        if (good) {
            candle.animate(new RainbowAnimation());
        } else {
            candle.animate(new SingleFadeAnimation(255, 0, 0));
        }

        candle.destroyObject();
        // band.close();
        // orchestra.close();
    }
}
