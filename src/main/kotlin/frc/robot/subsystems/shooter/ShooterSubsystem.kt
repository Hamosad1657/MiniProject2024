package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonSRX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem: SubsystemBase() {
	private val motor1 = HaTalonSRX(RobotMap.Shooter.HATALONSRX_ID1)
	private val motor2 = HaTalonSRX(RobotMap.Shooter.HATALONSRX_ID2)
	private val encoder1 = CANCoder(RobotMap.Shooter.CANCODER_ID1)
	private val encoder2 = CANCoder(RobotMap.Shooter.CANCODER2_ID)

	init {
		motor1.configRemoteFeedbackFilter(encoder1, 0, ShooterConstants.TIMEOUT_MS)
		motor1.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0)
		motor2.follow(motor1)
	}

	fun shootBallsCommand(angularVelocity: AngularVelocity): Command {
	return RunCommand({motor1.set(ControlMode.Velocity, angularVelocity.degPs)}, this).finallyDo { motor1.set(0.0) }
	}
}