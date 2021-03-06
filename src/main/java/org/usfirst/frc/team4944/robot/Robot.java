package org.usfirst.frc.team4944.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4944.robot.custom.XboxController;
import org.usfirst.frc.team4944.robot.subsystems.ArmSubsystem;
import org.usfirst.frc.team4944.robot.subsystems.DriveSystem;
import org.usfirst.frc.team4944.robot.subsystems.HoodSubsystem;
import org.usfirst.frc.team4944.robot.subsystems.HopperSubsystem;
import org.usfirst.frc.team4944.robot.subsystems.IntakeSubsystem;
import org.usfirst.frc.team4944.robot.subsystems.ShooterSubsystem;
import org.usfirst.frc.team4944.robot.subsystems.TurretSubsystem;

import com.kauailabs.navx.frc.AHRS;

import org.usfirst.frc.team4944.robot.commands.DriveCommandTesting;
import org.usfirst.frc.team4944.robot.commands.QueNShootTesting;
import org.usfirst.frc.team4944.robot.commands.ThreeBallAuto;
import org.usfirst.frc.team4944.robot.commands.VisionAlign;
import org.usfirst.frc.team4944.robot.custom.Limelight;

public class Robot extends TimedRobot {

	// CONTROLLERS

	XboxController driver;
	XboxController operator;

	// SUBSYSTEMS

	OI oi;
	DriveSystem driveSystem;
	TurretSubsystem turret;
	ShooterSubsystem shooter;
	IntakeSubsystem intake;
	HopperSubsystem hopper;
	HoodSubsystem hood;
	ArmSubsystem arms;
	Limelight lm;

	// Auto Commands
	Command autoCommand;
	AHRS gyro = new AHRS(I2C.Port.kMXP);
	double kGyroRateCorrection;
  	double gyroRateCoeff = 0.0;
  	double robotPeriod = TimedRobot.kDefaultPeriod;
  	int rate = gyro.getActualUpdateRate();
	double gyroPeriod = 1.0 / rate;
	double targetAngle;
	double angle;

	// SmartDashboard Values

	double turretEncoder;
	double shooterPower;
	double shooterCoeficient;

	// Comp Bot

	@Override
	public void robotInit() {
		// CONTROLLERS INIT
		this.driver = new XboxController(0);
		this.operator = new XboxController(1);

		// SUBSYSTEMS INIT
		this.arms = new ArmSubsystem();
		this.turret = new TurretSubsystem();
		this.shooter = new ShooterSubsystem();
		this.driveSystem = new DriveSystem();
		this.hopper = new HopperSubsystem();
		this.intake = new IntakeSubsystem();
		this.hood = new HoodSubsystem();
		this.oi = new OI();
		this.lm = new Limelight();
		// 0.0109621429
		this.shooterCoeficient = .0063;
		// SmartDashboard
		SmartDashboard.putNumber("Shooter Power", this.shooterPower);
		SmartDashboard.putNumber("Shooter Coefficient", this.shooterCoeficient);
		this.SmartDashboardDisplay();

		this.driveSystem.resetAngle();

		// Double
		// this.shooterPower = 0.5;
	}

	@Override
	public void disabledInit() {
		// this.arms.applyBreak();
		this.SmartDashboardDisplay();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run(); // KEEP HERE TO RUN COMMANDS
		this.SmartDashboardDisplay();
	}

	@Override
	public void autonomousInit() {
		Scheduler.getInstance().run();
		// this.autoCommand = new ThreeBallAuto();
		// this.autoCommand = new QueNShootTesting();
		this.autoCommand = new DriveCommandTesting();
		this.autoCommand.start();
		this.SmartDashboardDisplay();
	}

	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run(); // KEEP HERE TO RUN COMMANDS
	}

	@Override
	public void teleopInit() {
		Scheduler.getInstance().run();
		this.SmartDashboardDisplay();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run(); // KEEP HERE TO RUN COMMANDS
		System.out.println("Teleop");
		// Drive Code

		final double Y = -driver.getLeftStickY();
		final double X = driver.getRightStickX();
		this.driveSystem.setPower(X + Y, X - Y);

		// Update Values
		this.updateValues();
	}

	@Override
	public void testPeriodic() {
	}

	public void updateValues() {
		this.shooterCoeficient = SmartDashboard.getNumber("Shooter Coefficient", this.shooterCoeficient);


		// SETS SHOOTER TO DESIRED SPEED
		if (this.driver.getLeftTriggerDown()) {
			this.shooter.setManualShooterPower(this.shooterPower);
		} else {
			this.shooter.setManualShooterPower(0.15);
		}

		if (this.driver.getLeftBumper()) {
			this.hood.setHoodMotorPower(-0.1);
		}

		if (this.driver.getRightBumper()) {
			this.hood.setHoodMotorPower(0.1);
		}


		// Encoders
		// System.out.println(this.driveSystem.getLeftEncoder() + " Left");
		// System.out.println(this.driveSystem.getRightEncoder() + " Right");

		// Gyro
		// System.out.println(this.driveSystem.getAngle());

		// RIGHT MENU LOCK ON TURRET/HOOD

		if (this.driver.getRightTriggerDown()) {
			this.hood.updateValues();
			this.hood.setAngleByLM();
			this.hood.driveHoodPID();
			this.turret.followLimelightNoEncoder();
			this.shooterPower = (.5211761905 + (this.shooterCoeficient * this.lm.getDistInFeet()));
		} else {
			this.hood.setHoodMotorPower(0);
			this.turret.setTurretMotorPower(0);
		}
		//.00835
		// SMARTDASHBOARD

		// Displays all Smartdashboard Values
		this.angle = gyro.getYaw();

		this.SmartDashboardDisplay();

		// Shooter Power Equation

	}

	public void SmartDashboardDisplay() {

		// Turret

		// SmartDashboard.putNumber("Turret SetPoint", this.turret.getTurretSetPoint());
		// SmartDashboard.putNumber("Turret Encoder",
		// this.turret.getTurretEncoderValue());
		// SmartDashboard.putNumber("Turret Power", this.turret.getTurretPower());

		// Hood

		SmartDashboard.putNumber("Hood Encoder", this.hood.getHoodEncoderValue());
		SmartDashboard.putNumber("Hood SetPoint", this.hood.getHoodSetPoint());
		SmartDashboard.putNumber("Hood Power", this.hood.getHoodMotorPower());
		SmartDashboard.putNumber("Set Hood Angle", this.hood.getRequiredAngle());

		SmartDashboard.putNumber("Set Shooter Power", .5211761905 + (this.shooterCoeficient * this.lm.getDistInFeet()));
		// SmartDashboard.putNumber("Shooter Power v2", this.shooter.shooterMotor1());
		// Calculated Values

		// SmartDashboard.putNumber("Vx", this.hood.getVx());
		// SmartDashboard.putNumber("Vy", this.hood.getVy());

		// Limelight

		// SmartDashboard.putNumber("Limelight Y Offset", turret.lm.getYOffset());
		// SmartDashboard.putNumber("Limelight X Offset", turret.lm.getXOffset());
		SmartDashboard.putNumber("Distance From Target", turret.lm.getDistInFeet());
		SmartDashboard.putBoolean("Limelight Connection:", turret.lm.getLimeLightConnected());

		SmartDashboard.putNumber("Gyro Yaw: ", this.angle);
	}
	

}