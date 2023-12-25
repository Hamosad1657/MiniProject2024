package frc.robot.commands

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.hood.HoodSubsystem
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.turret.TurretSubsystem

fun LoaderSubsystem.loadWhenReadyToShootCommand(): Command =
	withName("loadWhenReadyToShoot") {
		loadWhen {
			ShooterSubsystem.withinTolerance() &&
				HoodSubsystem.withinTolerance() &&
				TurretSubsystem.withinTolerance()
		}
	}


fun LoaderSubsystem.loadWhenShooterAndHoodReady(): Command =
	withName("loadWhenHoodAndShooterReady") {
		loadWhen {
			ShooterSubsystem.withinTolerance() &&
				HoodSubsystem.withinTolerance()
		}
	}


fun LoaderSubsystem.loadWhen(condition: () -> Boolean): Command =
	withName("loadWhen") {
		run {
			if (condition()) {
				val ballsPerSec = ShooterSubsystem.ballsPerSec
				runMagazine(ballsPerSec)
				runConveyor(ballsPerSec)
			} else {
				stopMagazine()
				stopConveyor()
			}
		} finallyDo {
			stopMagazine()
			stopConveyor()
		}
	}


fun LoaderSubsystem.runOpenLoop() =
	run {
		setConveyor(1.0)
		setMagazine(1.0)
	} finallyDo {
		stopConveyor()
		stopMagazine()
	}


