// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;

public class NoteCommands {
  /**
   * Returns a sequence of commands to intake the note into the indexer.
   * 
   * This command should be bound to a button using whileTrue.
   * 
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command intake(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexerSubsystem;
    return Commands.race(
        Commands.runOnce(intake::in, intake),
        Commands.runOnce(indexer::intake, indexer))
        .andThen(Commands.idle(intake, indexer))
        .until(indexer::isNoteDetected)
        .andThen(Commands.race(
            Commands.runOnce(intake::disable, intake),
            Commands.runOnce(indexer::disable, indexer)));
  }

  /**
   * Returns a sequence of commands to outtake the note.
   * 
   * Assumes this command will be bound to a button using whileTrue.
   * 
   * @param subsystems The subsystems container.
   * @return The command sequence.
   */
  public static Command outtake(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexerSubsystem;
    return Commands.race(
        Commands.runOnce(intake::out, intake),
        Commands.runOnce(indexer::outtake, indexer))
        .andThen(Commands.idle(intake, indexer));
  }

  /**
   * Returns a command sequence to shoot a note if one is in the indexer otherwise
   * it will do nothing.
   * 
   * @param subsystems The subsystems container.
   * @param rpm The RPM to shoot the note.
   * @return The command sequence.
   */
  public static Command shoot(Subsystems subsystems, double rpm) {
    IndexerSubsystem indexer = subsystems.indexerSubsystem;
    ShooterSubsystem shooter = subsystems.shooter;
    return Commands.either(
        Commands.sequence(
            ShooterCommands.setAndWaitForRPM(subsystems, rpm),
            Commands.runOnce(indexer::feed, indexer),
            Commands.idle(shooter, indexer).until(() -> !indexer.isNoteDetected()),
            Commands.waitSeconds(0.5),
            Commands.runOnce(shooter::disable, shooter)),
        Commands.none(),
        indexer::isNoteDetected);
  }
}