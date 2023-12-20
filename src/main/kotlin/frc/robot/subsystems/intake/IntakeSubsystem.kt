package frc.robot.subsystems.intake

import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase(), AutoCloseable {

	private val motor = HaTalonFX(RobotMap.Intake.MOTOR_ID).apply {
		inverted = false // TODO: verify positive output is collecting
	}

	fun collect() = motor.set(Constants.MOTOR_OUTPUT)
	fun stop() = motor.stopMotor()

	val running: Boolean
		get() = motor.get() != 0.0

	init {
		name = "IntakeSubsystem"
		SmartDashboard.putData(this)
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("IntakeSubsystem")
		builder.addBooleanProperty("Running", { running }, null)
	}

	override fun close() {
		motor.close()
	}

	/** FOR TESTING */
	val currentOutput
		get() = motor.get()
}