/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteVisionSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class AlignToAmp extends Command {
  private static final double SPEED_FACTOR = 0.5;

  private SwerveSubsystem drivetrain;
  private Optional<NoteVisionSubsystem> noteVision;

  ProfiledPIDController controller =
      new ProfiledPIDController(
          1.0,
          0,
          0,
          new Constraints(
              SwerveSubsystem.getMaxSpeed() * SPEED_FACTOR, SwerveSubsystem.getMaxAcceleration()));

  /** Creates a new AlignToAmp. Robot need to be aligned rotationally. */
  public AlignToAmp(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;
    this.noteVision = subsystems.noteVision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(0);
    noteVision.get().setApriltagPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = noteVision.get().getAngleToBestTarget();
    double xSpeed = controller.calculate(angle);
    drivetrain.drive(xSpeed, 0, 0, true);
    // drivetrain.drive(0, xSpeed, 0, false); //alternate driving that uses
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    noteVision.get().setNotePipeline();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
