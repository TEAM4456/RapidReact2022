// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;



public class Drive extends SubsystemBase {
  private Pose2d pose = new Pose2d();

  public WPI_TalonFX leftDrive;
  public WPI_TalonFX rightDrive;

  public AHRS gyro;

  

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
  private final DifferentialDriveOdometry odometry;
  


  public Drive(WPI_TalonFX leftTalon, WPI_TalonFX rightTalon) {
    //drive.setDefaultCommand(new RunCommand(() -> diffDrive.arcadeDrive(controller.getRawAxis(0), -controller.getRawAxis(1), controller.getRightStickButtonPressed()), drive));
    leftDrive = leftTalon;
    rightDrive = rightTalon;
    //setDefaultCommand(defaultCommand);
    

    gyro = new AHRS(Port.kMXP);

    rightDrive.setSensorPhase(false);

    //diffDrive = new DifferentialDrive(leftDrive, rightDrive);

 
    odometry = new DifferentialDriveOdometry(getHeading());
  }

  public void driveWithArcade(double speed, double rotation) {
    RobotContainer.diffDrive.arcadeDrive(speed, rotation*-1);
  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("NavX Angle", RobotMap.navx.getAngle());
    SmartDashboard.putNumber("Left Position", leftDrive.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Velocity", leftDrive.getSelectedSensorVelocity());
    
    SmartDashboard.putNumber("Output", leftDrive.getMotorOutputPercent());
    SmartDashboard.putNumber("Distance", leftDrive.getSelectedSensorPosition() * Constants.kEncoderMeterperPulse);

    SmartDashboard.putNumber("pose get X", pose.getX());
    SmartDashboard.putNumber("pose get Y", pose.getY());   
   
    SmartDashboard.putNumber("Right Position", rightDrive.getSelectedSensorPosition());


    odometry.update(getHeading(), -1*getMotorPositionsinMeters(leftDrive), getMotorPositionsinMeters(rightDrive));
   // odometry.update(RobotMap.navx.get)
   // pose = odometry.update(getHeading(), getSpeeds().leftMetersPerSecond, getSpeeds().rightMetersPerSecond);
    // This method will be called once per scheduler run
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-RobotMap.navx.getAngle());
  }

  public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftDrive.getSelectedSensorVelocity()*Constants.kEncoderMeterperPulse * 10, -1*rightDrive.getSelectedSensorVelocity()*Constants.kEncoderMeterperPulse * 10);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(-rightVolts);
    RobotContainer.diffDrive.feed();  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics(){
    return kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    //resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }
  public void zeroHeading(){
    RobotMap.navx.reset();
  }

  public double getMotorPositionsinMeters(WPI_TalonFX motor) {
    return motor.getSelectedSensorPosition() * Constants.kEncoderMeterperPulse;
  } 
}
