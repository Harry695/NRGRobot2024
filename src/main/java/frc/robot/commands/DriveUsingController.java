/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class DriveUsingController extends Command {
  private static final double DEADBAND = 0.08;
  private static final double LOCK_ORIENTATION_DELAY = 0.5; // Seconds
  private static final double AUTO_ORIENTATION_SPEED_THRESHOLD = 0.05; // Meters per second

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KP =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kP", 1.0);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KI =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kI", 0);

  @RobotPreferencesValue(column = 2, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KD =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kD", 0);

  private final SwerveSubsystem drivetrain;
  private final CommandXboxController xboxController;
  private ProfiledPIDController controller;

  private double previousRInput = 0;
  private final Timer lockOrientationTimer = new Timer();

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    this.xboxController = xboxController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // We initialize the PID controller and PID constant in initialize. To make preferences changes
    // take effect immediately, press the "interrupt all" button.
    controller =
        new ProfiledPIDController(
            AUTO_ORIENT_KP.getValue(),
            AUTO_ORIENT_KI.getValue(),
            AUTO_ORIENT_KD.getValue(),
            SwerveSubsystem.getRotationalConstraints());
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setIZone(Math.toRadians(5));
    controller.reset(drivetrain.getOrientation().getRadians());

    drivetrain.lockDesiredOrientation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rInput;
    double xInput = -xboxController.getLeftY();
    double yInput = -xboxController.getLeftX();
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xInput = MathUtil.applyDeadband(xInput, DEADBAND) * inputScalar;
    yInput = MathUtil.applyDeadband(yInput, DEADBAND) * inputScalar;

    Optional<Rotation2d> targetOrientation = drivetrain.getTargetOrientation();

    rInput = -xboxController.getRightX();
    rInput = MathUtil.applyDeadband(rInput, DEADBAND) * inputScalar;

    if (targetOrientation.isEmpty()) {
      if (rInput == 0) {
        if (previousRInput != 0) {
          lockOrientationTimer.reset();
          lockOrientationTimer.start();
        } else if (lockOrientationTimer.get() > LOCK_ORIENTATION_DELAY) {
          drivetrain.lockDesiredOrientation();
          lockOrientationTimer.stop();
          targetOrientation = drivetrain.getTargetOrientation();
        }
      }
    } else if (rInput != 0) {
      if (previousRInput == 0) {
        drivetrain.unlockDesiredOrientation();
      }
    }

    previousRInput = rInput;

    double rSpeed = rInput;

    // If the orientation is locked, maintain the orientation using PID.
    if (targetOrientation.isPresent()
        && drivetrain.getSpeed() >= AUTO_ORIENTATION_SPEED_THRESHOLD) {
      double currentOrientation = drivetrain.getOrientation().getRadians();
      double feedback =
          controller.calculate(currentOrientation, targetOrientation.get().getRadians());

      rSpeed =
          feedback
              + (controller.getSetpoint().velocity
                  / SwerveSubsystem.getRotationalConstraints().maxVelocity);
    }

    drivetrain.drive(xInput, yInput, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
