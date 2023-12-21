package frc.robot.commands

import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterSubsystem
import kotlin.experimental.ExperimentalTypeInference

@OptIn(ExperimentalTypeInference::class)
@OverloadResolutionByLambdaReturnType
@JvmName("shootBallsCommandBallsPerSec")
fun ShooterSubsystem.shootBallsCommand(ballsPerSecSupplier: () -> Double): Command {
	return withName("ShootBalls") {
		val angularVelocitySupplier: () -> AngularVelocity = {
			val rotationsPerSec = ballsPerSecSupplier() * ShooterConstants.SHOOTER_BALLS_PER_ROTATION
			AngularVelocity.fromRps(rotationsPerSec)
		}
		shootBallsCommand(angularVelocitySupplier)
	}
}

@JvmName("shootBallsCommandAngularVelocity")
fun ShooterSubsystem.shootBallsCommand(angularVelocitySupplier: () -> AngularVelocity): Command =
	withName("shootBalls") {
		run {
			getToVelocity(angularVelocitySupplier())
		}
	}

fun ShooterSubsystem.stopCommand(): Command =
	withName("stopShooter") {
		runOnce { stopShooter() }
	}