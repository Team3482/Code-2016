package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.Robot;
import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Intake extends Subsystem {

    private final CANTalon intake = (CANTalon) RobotMap.intake;
    private final SpeedController wheels = RobotMap.wheels;
    private final Relay feed = RobotMap.intakeFeed;
    
	StringBuilder sb = new StringBuilder();
	int loops = 0;
	final double startPosition; //.703
	final double lowerPosition; //-34.196
	final double restPosition; //-2.283
	final double portPosition; //-46.324
	/*final double lowerPosition = -33.679; //intake
	final double restPosition = .517;
	final double portPosition = -53.650;*/
	double targetPositionRotations;
    boolean isPID = true;
    
    //
	
    public Intake() {        
		int absolutePosition = intake.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
        intake.setEncPosition(absolutePosition);
        intake.reverseSensor(false);
        intake.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        //motor.configEncoderCodesPerRev(codesPerRev); //only for quad
        intake.configNominalOutputVoltage(+0f, -0f);
        intake.configPeakOutputVoltage(+12f, -12f);
        
        intake.setAllowableClosedLoopErr(0);
        intake.setProfile(0);
        intake.setF(0.0);
        intake.setP(0.03);
        intake.setI(0.0);
        intake.setD(0.0);
        
        //intake.changeControlMode(TalonControlMode.Position);
        
        
        startPosition = intake.getPosition();
        restPosition = startPosition-3.986;
        lowerPosition = startPosition-32.5;
		portPosition = startPosition-47.027;
        targetPositionRotations = restPosition;
    }
    
    public void maintainPosition() {
    	if (isPID) { 
	    	double motorOutput = intake.getOutputVoltage()/intake.getBusVoltage();
	    	sb.append("Intake:");
	    	sb.append("\tout:");
		  	sb.append(motorOutput);
		  	sb.append("\tpos:");
	        sb.append(intake.getPosition() );
	        intake.changeControlMode(TalonControlMode.Position);
	    	intake.set(targetPositionRotations);
	    	sb.append("\terrNative:");
	    	sb.append(intake.getClosedLoopError());
	    	sb.append("\ttrg:");
	    	sb.append(targetPositionRotations);
	    	if(++loops >= 10) {
	          	loops = 0;
	          	System.out.println(sb.toString());
	        }
	        sb.setLength(0);
    	}
    }
    
    public void setTargetPositionRotations (double target) {
    	targetPositionRotations = target;
    }
    
    public void setTargetLower() {
    	targetPositionRotations = lowerPosition;
    }
    
    public void setTargetPortcullis() {
    	targetPositionRotations = portPosition;
    }
    public void stopPID() {
    	isPID = false;
    }
    
    public void startPID() {
    	isPID = true;
    }
    
    public void setTargetRest() {
    	targetPositionRotations = restPosition;
    }
    
    public void runWheels() {
    	wheels.set(1.0);
    }
    
    public void runWheelsBackward() {
    	wheels.set(-1.0);
    }
    
    public void spinIntake() {
    	intake.set(1.0);
    }
    
    public boolean isPIDOn() {
    	return isPID;
    }
    
    public void spinIntake(double speed) {
    	intake.set(speed);
    }
    
    public void runWithXboxController(Joystick s) {
		double y = s.getAxis(AxisType.kY); 
		intake.changeControlMode(TalonControlMode.PercentVbus);
		intake.set(y);
	}
    
    public void stopWheels() {
    	wheels.set(0.0);
    }
    
    public void stopIntake() {
    	intake.set(0.0);
    }    
    
    public void startFeed() {
    	feed.set(Relay.Value.kForward);
    }
    
    public void reverseFeed() {
    	feed.set(Relay.Value.kReverse);
    }
    
    public void stopFeed() {
    	feed.set(Relay.Value.kOff);
    }

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}

}

