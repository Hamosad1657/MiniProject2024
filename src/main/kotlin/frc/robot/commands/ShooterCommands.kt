package frc.robot.commands

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterSubsystem

fun ShooterSubsystem.shootBallsCommand(ballsPerSec: Double): Command {
	val rotationsPerSec = ballsPerSec * ShooterConstants.SHOOTER_BALLS_PER_ROTATION
	val angularVelocity = AngularVelocity.fromRps(rotationsPerSec)
	return shootBallsCommand { angularVelocity }
}

fun ShooterSubsystem.shootBallsCommand(angularVelocitySupplier: () -> AngularVelocity): Command =
	withName("shootBalls") {
		run {
			getToVelocity(angularVelocitySupplier())
		} finallyDo {
			stopShooter()
		}
	}

fun ShooterSubsystem.stopCommand(): Command =
	withName("stopShooter") {
		runOnce { stopShooter() }
	}