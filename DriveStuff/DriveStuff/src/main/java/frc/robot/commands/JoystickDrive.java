// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  /** Creates a new JoystickDrive. */
  private final DriveSubsystem driveSubsystem;
  private final static XboxController driverController = RobotContainer.driverController;
  
  public JoystickDrive(DriveSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = driverController.getY(Hand.kLeft);
    double rotate = driverController.getX(Hand.kRight);

    if((throttle > 0 && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
      throttle = 0;
    } else {
      throttle = (Math.tan(.465 * (throttle * Math.PI))) / 3;
    }

    if((rotate > 0 && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
        rotate = 0;
    }
    
    // Slow mode
    if (driverController.getTriggerAxis(Hand.kLeft) > 0.25) {
      throttle = Math.signum(throttle) * 0.75;
    }
    // Fast Mode
    else if (driverController.getTriggerAxis(Hand.kRight) > 0.25) {
        throttle = (throttle*1.7);
    }
    // Normal Driving Speed
    else {
        throttle = (throttle*1.2);
    }
  

    driveSubsystem.drive(throttle, rotate);

    // rotation speed scaler
    rotate = 2*rotate;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
