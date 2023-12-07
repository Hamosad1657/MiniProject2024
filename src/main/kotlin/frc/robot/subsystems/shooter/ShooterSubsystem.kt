package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem : SubsystemBase() {
	val motor = HaTalonFX(RobotMap.Shooter.MOTOR_ID).apply {
		config_kP(0, ShooterConstants.PID_GAINS.kP)
		config_kI(0, ShooterConstants.PID_GAINS.kI)
		config_kD(0, ShooterConstants.PID_GAINS.kD)
	}

	val ballsPerSecs: Double
		get() = angularVelocity.rps * ShooterConstants.SHOOTER_BALLS_PER_ROTATION

	val angularVelocity: AngularVelocity
		get() = AngularVelocity.fromFalconTicksPer100ms(motor.selectedSensorVelocity)

	private fun setSetpoint(velocity: AngularVelocity) {
		motor.set(ControlMode.Velocity, angularVelocity.degPs)
	}

	fun shootBallsCommand(angularVelocity: AngularVelocity): Command {
		return withName("shootBalls") {
			ShooterSubsystem.run { setSetpoint(angularVelocity) } andThen
				ShooterSubsystem.runOnce { motor.stopMotor() }
		}
	}
}