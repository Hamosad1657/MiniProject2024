package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.shooter.ShooterConstants as Constants

object ShooterSubsystem : SubsystemBase() {
	val motor = HaTalonFX(RobotMap.Shooter.MOTOR_ID).apply {
		configPIDGains(Constants.PID_GAINS)
	}

	val ballsPerSecs: Double get() = angularVelocity.rps * Constants.SHOOTER_BALLS_PER_ROTATION

	private val angularVelocity get() = AngularVelocity.fromFalconTicksPer100ms(motor.selectedSensorVelocity)

	fun getToVelocity(velocity: AngularVelocity) {
		motor.set(ControlMode.Velocity, velocity.degPs)
	}

	fun stopShooter() {
		motor.stopMotor()
	}
}