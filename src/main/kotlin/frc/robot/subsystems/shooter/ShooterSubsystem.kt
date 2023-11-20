package frc.robot.subsystems.shooter

import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonSRX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.RobotMap

object ShooterSubsystem: Subsystem {
	private val motor1 = HaTalonSRX(RobotMap.Shooter.HATALONSRX_ID1)
	private val motor2 = HaTalonSRX(RobotMap.Shooter.HATALONSRX_ID2)
	private val encoder1 = CANCoder(RobotMap.Shooter.CANCODER_ID1)
	private val encoder2 = CANCoder(RobotMap.Shooter.CANCODER2_ID)

	fun shootBallsCommand(): Command {
	return InstantCommand({RunCommand({motor1.set(ShooterConstants.OUTPUT)}, this)})
	}
}