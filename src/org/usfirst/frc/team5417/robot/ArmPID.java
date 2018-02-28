package org.usfirst.frc.team5417.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class ArmPID extends PIDSubsystem {
	Encoder encoder;
	VictorSP motor;
	double correction = 0;
	public ArmPID(Encoder userEncoder, PIDSourceType source, VictorSP userMotor, double p, double i, double d) {
		super(p, i, d);
		encoder = userEncoder;
		encoder.setMaxPeriod(.1);
		encoder.setDistancePerPulse(0.17578125);
		encoder.setPIDSourceType(source);
		motor = userMotor;
	}

	@Override
	protected double returnPIDInput() {
		return encoder.pidGet();
	}

	@Override
	protected void usePIDOutput(double output) {
		correction = output;
		motor.set(output * 1);
	}
	
	double getOutput() {
		return correction;
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}

	

	

}
