package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Shooter.MOTOR_ID)

	init {
		motor.config_kP(0, ShooterConstants.PID_GAINS.kP)
		motor.config_kI(0, ShooterConstants.PID_GAINS.kI)
		motor.config_kD(0, ShooterConstants.PID_GAINS.kD)
	}

	fun shootBallsCommand(angularVelocity: AngularVelocity): Command {
		return run { set(angularVelocity) }.finallyDo { motor.set(0.0) }
	}

	private fun set(angularVelocity: AngularVelocity) {
		motor.set(ControlMode.Position, angularVelocity.degPs)
	}

	val ballsPerSecs: Double
		get() = angularVelocity.rps * ShooterConstants.SHOOTER_BALLS_PER_ROTATION

	val angularVelocity: AngularVelocity
		get() = AngularVelocity.fromFalconTicksPer100ms(motor.selectedSensorVelocity)
}