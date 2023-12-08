package frc.robot.commands

import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.IntakeSubsystem

fun IntakeSubsystem.collectCommand(): Command =
	withName("collect") { run(this::collect) }


fun IntakeSubsystem.stopIntakeCommand(): Command =
	withName("stopIntake") { runOnce(this::stop) }
