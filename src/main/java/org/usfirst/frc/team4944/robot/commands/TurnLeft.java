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

	//PID
	BasicPID anglePID;
	
	// NAVX
	//AHRS gyro = new AHRS(I2C.Port.kMXP);
	double kGyroRateCorrection;
  	double gyroRateCoeff = 0.0;
  	double robotPeriod = TimedRobot.kDefaultPeriod;
  	//int rate = gyro.getActualUpdateRate();
	//double gyroPeriod = 1.0 / rate;
	double targetAngle, angleInit;
	double angle;

	public TurnLeft(Double targetAngle, BasicPID anglePID) {
		this.driveSystem = new DriveSystem();
		requires(driveSystem);
		this.anglePID = anglePID;
		//this.angle = gyro.getRoll();
    	// kGyroRateCorrection = (robotPeriod / gyroPeriod) * gyroRateCoeff;
		// this.angle += gyro.getRate() * kGyroRateCorrection;
		// this.angle = Math.IEEEremainder(angle, 360.0);
		this.targetAngle = targetAngle;
		this.angleInit = driveSystem.getAngle();
		init();
	}

	public void init() {
		System.out.println("Init");
		//gyro.calibrate();
		anglePID.setSetPoint(this.angleInit + this.targetAngle);
	}

	public void execute() {
		double anglePower = anglePID.getPower(driveSystem.getAngle());
		SmartDashboard.putNumber("Gyro Yaw: ", this.driveSystem.getAngle());
		driveSystem.setPower(anglePower, -anglePower);
	}

	public boolean isFinished() {
		if (this.anglePID.getError() < 5) {
			if(this.driveSystem.getDoneDriveing()){
				System.out.println("Exited");
				driveSystem.setPower(0, 0);
				return true;
			}else{
				return false;
			}
		}else{
			return false;	
		}

	}
}
