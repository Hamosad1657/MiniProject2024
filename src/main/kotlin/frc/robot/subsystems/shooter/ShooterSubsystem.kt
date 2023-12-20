package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import kotlin.math.abs
import frc.robot.subsystems.shooter.ShooterConstants as Constants

object ShooterSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Shooter.MOTOR_ID).apply {
		inverted = false // TODO: verify positive output is shooting
		configPIDGains(Constants.PID_GAINS)
	}

	val ballsPerSec: Double get() = angularVelocity.rps * Constants.SHOOTER_BALLS_PER_ROTATION

	private val angularVelocity get() = AngularVelocity.fromFalconTicksPer100ms(motor.selectedSensorVelocity)

	private var setpoint = AngularVelocity.fromRpm(0.0)

	init {
		SmartDashboard.putData(this)
	}

	fun getToVelocity(velocity: AngularVelocity) {
		setpoint = velocity
		motor.set(ControlMode.Velocity, velocity.degPs)
	}

	fun stopShooter() {
		motor.stopMotor()
	}

	fun withinTolerance(): Boolean {
		val error = setpoint - angularVelocity
		return abs(error.rpm) <= Constants.VELOCITY_TOLERANCE.rpm
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("ShooterSubsystem")
		builder.addDoubleProperty("Balls per sec", { ballsPerSec }, null)
		builder.addDoubleProperty("Deg per sec", { angularVelocity.degPs }, null)
		builder.addDoubleProperty("Setpoint deg per sec", { setpoint.degPs }, null)
		builder.addBooleanProperty("Within tolerance", ::withinTolerance, null)
	}
}