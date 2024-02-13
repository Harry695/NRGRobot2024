// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public class AdjustAngleShooting extends Command {
  SwerveSubsystem drivetrain;
  ArmSubsystem arm;

  /** Creates a new AdjustAngleShooting. */
  public AdjustAngleShooting(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;
    this.arm = subsystems.armSubsystem;

    addRequirements(drivetrain, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose3d SPEAKER_POSE = DriverStation.getAlliance().get().equals(Alliance.Blue) ? FieldConstants.BLUE_SPEAKER_TOP
        : FieldConstants.RED_SPEAKER_TOP;
    double goalAngle = Math.atan2(
        (SPEAKER_POSE.getZ() - RobotConstants.APRILTAG_CAMERA_TO_ROBOT.getZ()),
        Math.hypot(
            SPEAKER_POSE.getX() - drivetrain.getPosition().getX(),
            SPEAKER_POSE.getY() - drivetrain.getPosition().getY()));
    arm.setGoalAngle(goalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
