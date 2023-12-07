package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.loader.LoaderSubsystem

fun LoaderSubsystem.loadCommand(ballsPerSecs: () -> Double): Command {
	return withName("load") {
		LoaderSubsystem.run { load(ballsPerSecs()) } andThen
			LoaderSubsystem.runOnce { motor.stopMotor() }
	}
}
