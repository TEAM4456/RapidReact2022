// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;



public class Drive extends SubsystemBase {
  private Pose2d pose = new Pose2d();

  private final WPI_TalonFX leftDrive;
  private final WPI_TalonFX rightDrive;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d(0, 0, new Rotation2d()));

  public Drive(WPI_TalonFX leftMaster, WPI_TalonFX rightMaster) {
    leftDrive = leftMaster;
    rightDrive = rightMaster;
  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("NavX Angle", RobotMap.navx.getAngle());
    SmartDashboard.putNumber("Left Encoder", leftDrive.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", rightDrive.getSelectedSensorPosition());
    
    pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-RobotMap.navx.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds((leftDrive.getSelectedSensorVelocity() / 10.71 *2*Math.PI * Units.inchesToMeters(3)) / 60, (rightDrive.getSelectedSensorVelocity() / 10.71 *2*Math.PI * Units.inchesToMeters(3)) / 60);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.set(ControlMode.PercentOutput, leftVolts / Constants.maxVolts);
    rightDrive.set(ControlMode.PercentOutput, rightVolts / Constants.maxVolts);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    //resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading().getDegrees()));
  }

  public void zeroHeading(){
    RobotMap.navx.reset();
  }
  
}
