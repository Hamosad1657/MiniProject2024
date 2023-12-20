package frc.robot.subsystems.conveyor

import com.hamosad1657.lib.motors.HaCANSparkMax
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMax.ControlType
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.conveyor.ConveyorConstants as Constants

object ConveyorSubsystem : SubsystemBase() {

	private val loaderMotor = HaCANSparkMax(RobotMap.Conveyor.LOADER_MOTOR_ID).apply {
		inverted = false // TODO: verify positive output is loading and not unloading
		idleMode = CANSparkMax.IdleMode.kBrake
	}
	private val loaderEncoder = loaderMotor.encoder
	private val loaderPIDController = loaderMotor.pidController

	private val conveyorMotor = HaCANSparkMax(RobotMap.Conveyor.CONVEYOR_MOTOR_ID).apply {
		inverted = false // TODO: verify positive output is loading and not unloading
		idleMode = CANSparkMax.IdleMode.kBrake
	}
	private val conveyorEncoder = conveyorMotor.encoder
	private val conveyorPIDController = conveyorMotor.pidController

	val conveyorBallsPerSec: Double
		get() = conveyorEncoder.velocity * Constants.CONVEYOR_VELOCITY_RATIO

	val loaderBallsPerSec: Double
		get() = loaderEncoder.velocity * Constants.LOADER_VELOCITY_RATIO

	init {
		SmartDashboard.putData(this)
	}

	fun runConveyor(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.CONVEYOR_VELOCITY_RATIO
		conveyorPIDController.setReference(setpointRPM, ControlType.kVelocity)
	}

	fun stopConveyor() {
		conveyorMotor.stopMotor()
	}

	fun runLoader(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.LOADER_VELOCITY_RATIO
		loaderPIDController.setReference(setpointRPM, ControlType.kVelocity)
	}


	fun stopLoader() {
		loaderMotor.stopMotor()
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("ConveyorSubsystem")
		builder.addDoubleProperty("Conveyor balls per sec", { conveyorBallsPerSec }, null)
		builder.addDoubleProperty("Loader balls per sec", { loaderBallsPerSec }, null)
	}
}