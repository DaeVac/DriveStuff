package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RobotMap {
    public static final int LEFT_DRIVE_PORT = 1;
    public static final int RIGHT_DRIVE_PORT = 2;

    public static WPI_TalonFX leftDriveMotor = new WPI_TalonFX(LEFT_DRIVE_PORT);
    public static WPI_TalonFX rightDriveMotor = new WPI_TalonFX(RIGHT_DRIVE_PORT);
}
