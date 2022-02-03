// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final DifferentialDriveKinematics kTrackWidthMeters = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    public static double maxVolts = 28;
    public static int maxVelocity = 9;
    public static double kPDriveVelocity = 1.5294;
    public static final int kEncoderCPR = 1024;
    public static final double ksVolts = 0.49701;
    public static final double kvVoltsSecondsPerMeter = 4.8933;
    public static final double kaVoltsSecondSquaredPerMeter = 0.35959;
    public static final double kpDriveVel = 8.5;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kEncoderMeterperPulse = .1524 * Math.PI / 2048 / 10.71; //pulse * diameter * pi / pulse per rotation / 10.71 (gear reduction)
    
}
