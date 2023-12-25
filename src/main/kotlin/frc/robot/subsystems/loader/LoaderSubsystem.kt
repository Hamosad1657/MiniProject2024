package frc.robot.subsystems.loader

import com.hamosad1657.lib.motors.HaCANSparkMax
import com.revrobotics.CANSparkMax.ControlType
import com.revrobotics.CANSparkMax.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.loader.LoaderConstants as Constants

object LoaderSubsystem : SubsystemBase() {
	private val magazineMotor = HaCANSparkMax(RobotMap.Conveyor.LOADER_MOTOR_ID).apply {
		inverted = false
		idleMode = IdleMode.kBrake
	}
	private val magazineEncoder = magazineMotor.encoder
	private val magazinePIDController = magazineMotor.pidController
	val magazineBallsPerSec get() = magazineEncoder.velocity / Constants.MAGAZINE_BALLS_PER_MOTOR_ROTATION

	private val conveyorMotor = HaCANSparkMax(RobotMap.Conveyor.CONVEYOR_MOTOR_ID).apply {
		inverted = false
		idleMode = IdleMode.kBrake
	}
	private val conveyorEncoder = conveyorMotor.encoder
	private val conveyorPIDController = conveyorMotor.pidController
	val conveyorBallsPerSec get() = conveyorEncoder.velocity / Constants.CONVEYOR_BALLS_PER_MOTOR_ROTATION

	init {
		SmartDashboard.putData(this)
	}

	fun setMagazine(percentOutput: Double) {
		magazineMotor.set(percentOutput)
	}

	fun setConveyor(percentOutput: Double) {
		conveyorMotor.set(percentOutput)
	}

	fun runMagazine(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.MAGAZINE_BALLS_PER_MOTOR_ROTATION / 60.0
		magazinePIDController.setReference(setpointRPM, ControlType.kVelocity)
	}

	fun runConveyor(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.CONVEYOR_BALLS_PER_MOTOR_ROTATION / 60.0
		conveyorPIDController.setReference(setpointRPM, ControlType.kVelocity)
	}

	fun stopMagazine() {
		magazineMotor.stopMotor()
	}

	fun stopConveyor() {
		conveyorMotor.stopMotor()
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("ConveyorSubsystem")
		builder.addDoubleProperty("Conveyor balls per sec", { conveyorBallsPerSec }, null)
		builder.addDoubleProperty("Loader balls per sec", { magazineBallsPerSec }, null)
	}
}