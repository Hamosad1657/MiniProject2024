package frc.robot.commands

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.hood.HoodSubsystem

fun HoodSubsystem.getToAngleCommand(desiredAngleSupplier: () -> Rotation2d): Command {
	return withName("getToAngle") {
		run {
			getToAngle(desiredAngleSupplier())
		} finallyDo {
			stopHood()
		}
	}
}

fun HoodSubsystem.stopCommand(): Command {
	return withName("stopHood") { runOnce { stopHood() } }
}