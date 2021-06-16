package org.usfirst.frc.team4944.robot.commands;

import javax.swing.JList.DropLocation;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team4944.robot.subsystems.DriveSystem;
import org.usfirst.team4944.robot.PID.BasicPID;
import org.usfirst.team4944.robot.PID.DrivePID;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;

public class TurnLeft extends Command {
	// SUBSYSTEMS
	DriveSystem driveSystem;
	
	// NAVX
	AHRS gyro = new AHRS(I2C.Port.kMXP);
	double kGyroRateCorrection;
  	double gyroRateCoeff = 0.0;
  	double robotPeriod = TimedRobot.kDefaultPeriod;
  	int rate = gyro.getActualUpdateRate();
	double gyroPeriod = 1.0 / rate;
	double targetAngle;
	double angle;

	public TurnLeft(Double targetAngle) {
		this.driveSystem = new DriveSystem();
		requires(driveSystem);
		//this.angle = gyro.getRoll();
    	// kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
		// this.angle += gyro.getRate() * kGyroRateCorrection;
		// this.angle = Math.IEEEremainder(angle, 360.0);
		this.targetAngle = targetAngle;
		init();
	}

	public void init() {
		System.out.println("Init");
		gyro.calibrate();
	}

	public void execute() {
		this.angle = this.gyro.getYaw();
    	// kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
		// this.angle += gyro.getRate() * kGyroRateCorrection;
		// this.angle = Math.IEEEremainder(angle, 360.0);
		driveSystem.setLeftPower(-0.25);
		driveSystem.setRightPower(-0.25);
		System.out.println(this.gyro.getYaw());
		// System.out.println(gyro.isConnected());
		// System.out.println(gyro.isCalibrating());
		SmartDashboard.putNumber("Gyro Yaw: ", this.angle);
	}

	public boolean isFinished() {
		if ((this.angle <= -90 /*Constant for now will eventually change it to a variable */)) {
			if(this.driveSystem.getDoneDriveing()){
				System.out.println("Exited");
				driveSystem.setLeftPower(0);
				driveSystem.setRightPower(0);
				return true;
			}else{
				return false;
			}
		}else{
			return false;	
		}

	}
}
