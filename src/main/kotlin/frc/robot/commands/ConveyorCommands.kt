package frc.robot.commands

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.conveyor.ConveyorSubsystem
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

fun ConveyorSubsystem.loadWhenReadyToShootCommand(): Command =
	withName("loadWhenReadyToShoot") {
		loadWhen {
			ShooterSubsystem.withinTolerance() &&
				HoodSubsystem.withinTolerance() &&
				TurretSubsystem.withinTolerance()
		}
	}


fun ConveyorSubsystem.loadWhenShooterAndHoodReady(): Command =
	withName("loadWhenHoodAndShooterReady") {
		loadWhen {
			ShooterSubsystem.withinTolerance() &&
				HoodSubsystem.withinTolerance()
		}
	}


fun ConveyorSubsystem.loadWhen(condition: () -> Boolean): Command {
	return withName("loadWhen") {
		run {
			if (condition()) {
				val ballsPerSec = ShooterSubsystem.ballsPerSec
				runConveyor(ballsPerSec)
				runLoader(ballsPerSec)
			} else {
				stopLoader()
				stopConveyor()
			}
		} finallyDo {
			stopLoader()
			stopConveyor()
		}
	}
}


