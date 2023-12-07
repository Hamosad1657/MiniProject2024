package frc.robot.commands

import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.collectCommand(): Command {
	return withName("collect") { IntakeSubsystem.run { collect() } }
}

fun IntakeSubsystem.stopIntakeCommand(): Command {
	return withName("stopIntake") { IntakeSubsystem.runOnce(this::stopIntake) }
}