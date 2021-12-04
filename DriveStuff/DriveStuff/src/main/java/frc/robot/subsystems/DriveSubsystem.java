// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private static final WPI_TalonFX leftMotor = RobotMap.leftDriveMotor;
  private static final WPI_TalonFX rightMotor = RobotMap.rightDriveMotor;

  private static XboxController driverController = Robot.m_robotContainer.driverController;

  private static final double IN_TO_M = .0254;

  private static final int MOTOR_ENCODER_CODES_PER_REV = 2048; //4096 for CTRE Mag Encoders, 2048 for the Falcons
  private static final double DIAMETER_INCHES = 5.0; // Flex wheels on Everybot
    
	private static final double WHEEL_DIAMETER = DIAMETER_INCHES * IN_TO_M; // in meters
  private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  // This is for a output gear side motor encoder
	// private static final double TICKS_PER_METER = MOTOR_ENCODER_CODES_PER_REV / WHEEL_CIRCUMFERENCE;

  private static final double GEAR_RATIO = 12.75;
  //private static final double RIGHT_GEAR_RATIO = 10.71;
  // This is for an encoder mounted to the motor
  private static final double TICKS_PER_METER = (MOTOR_ENCODER_CODES_PER_REV * GEAR_RATIO) / (WHEEL_CIRCUMFERENCE);
  private static final double METERS_PER_TICKS = 1 / TICKS_PER_METER;

  public DriveSubsystem() {
    resetEncoders();

    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
    leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);
    
    rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    rightMotor.configVelocityMeasurementWindow(16);//1,2,4,8,16,32,64(default)
    rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5, 10);

    leftMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setNeutralMode(NeutralMode.Coast);
        
    leftMotor.setInverted(false); //--> enable if we determine they are not even
    rightMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setModePercentVoltage() {
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  // basic, no frills, no PID, no nothing drivetrain
  public static void drive(double throttle, double rotate) {
    leftMotor.set(throttle + rotate);
    rightMotor.set(throttle - rotate);
  }
  
  public double distanceTravelledinTicks() {
		return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
  }
    
	  public double getLeftEncoderPosition() {
		  return leftMotor.getSelectedSensorPosition();
	  }

    public double getRightEncoderPosition() {
      return rightMotor.getSelectedSensorPosition();
    }

  public double getLeftEncoderVelocityMetersPerSecond() {
    //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double leftVelocityMPS = (leftMotor.getSelectedSensorVelocity()*10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    leftVelocityMPS = leftVelocityMPS * METERS_PER_TICKS;
    return (leftVelocityMPS);
  }
  
  public double getRightEncoderVelocityMetersPerSecond() {
    //getQuadVelocity is in 100 ms so we have to divide it by 10 to get seconds
    double rightVelocityMPS = (rightMotor.getSelectedSensorVelocity()*10); // /10
    // since getQuadVelocity is in encoder ticks, we have to convert it to meters
    //Need to have a negative for right velocity since the motors are reversed on the opposite side
    rightVelocityMPS = rightVelocityMPS * METERS_PER_TICKS;
    return (rightVelocityMPS);
  }

  public double leftDistanceTravelledInMeters() {
    double left_dist = getLeftEncoderPosition() * METERS_PER_TICKS;
    return left_dist;
  }

  public double rightDistanceTravelledInMeters() {
    double right_dist = getRightEncoderPosition() * METERS_PER_TICKS;
    return right_dist;
  }
  
    public double distanceTravelledinMeters() {
      // left distance is negative because the encoder value on the 
      // left is negative when dev bot is pushed forward 2/15/20
      // Code Tested on Dev Bot, Works on 2/15/20
      double distanceTravelled = (leftDistanceTravelledInMeters() + rightDistanceTravelledInMeters()) / 2;
      return distanceTravelled;
    }

    public void resetEncoders() {
      leftMotor.setSelectedSensorPosition(0);
      rightMotor.setSelectedSensorPosition(0);
    }
    
  public void stop() {
    drive(0, 0);
  }
}
