// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    /**
     * Scale factor for the drive joystick inputs. Can be reduced for practice.
     */
    public static final double JOYSTICK_SCALE = 1.0;
  }

  public static final double MAX_ANGULAR_VELOCITY = 2 * Math.PI; // Math.PI radians/sec

  public static final String PHOTONCAMERA_NAME = "Arducam_A";
  public static final String PHOTONCAMERA_NAME2 = "Arducam_B";

  public static class IDs {
    public static final int INTAKE_MOTOR = 2;
    public static final int OUTTAKE_TOP_MOTOR = 3;
    public static final int OUTTAKE_BOTTOM_MOTOR = 4;
    public static final int ANGLE_MOTOR = 31;
    public static final int ROLLER_MOTOR = 30;
  }

  public static class Speeds {
    public static final double INTAKE_SPEED = -2000;
    public static final double OUTTAKE_SPEED = 3000; 
    public static final double SHOOTER_TRANSFER_SPEED = 350; 
    public static final double AMP_TRANSFER_SPEED = -.4; 
  }
  public static final double MIN_DELTA_V = 0;
  public static final double MIN_V = 60;
  public static final double SHOOT_DELAY = 1;
  public static final double NUDGE_SPEED = 2000;

  public static final int ARM_ENCODER_ZERO = 3212;

  public static class OuttakeGains {
    public static final double kP = 1e-4;
    public static final double kI = 1e-7;
    public static final double kD = 1e-5;
    public static final double kIz = 0; 
    public static final double kFF = 0.000125; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700.0;
  }
  public static class OuttakeGainsAmp {
    public static final double kP = 0.009;
    public static final double kI = 1e-6;
    public static final double kD = 0.01;
    public static final double kIz = 0.0; 
    public static final double kFF = 0.0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700.0;
  }

  public static final Matrix<N3, N1> VISION_STDDEV = new Matrix<N3, N1>(N3.instance, N1.instance, new double[] {2.0, 2.0, 2.5});
}
