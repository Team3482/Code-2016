package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Arm extends Subsystem {

    private final CANTalon lowerJoint = RobotMap.armlowerJoint;
    private final CANTalon upperJoint = RobotMap.armupperJoint;

    StringBuilder sb = new StringBuilder();
	int loopsL = 0;
	int loopsU = 0;
	final double lowerStartPosition;
	final double upperStartPosition;
	/*final double lowerRestPosition = 0;
	final double upperRestPosition = 0;
	final double upperHomePosition = .938;
	final double sallyLowerPosition = 0.270;
	final double sallyUpperPosition = 0.526;
	final double drawReachLowerPosition = .382;
	final double drawReachUpperPosition = .214;
	final double drawPressLowerPosition = .463;
	final double drawPressUpperPosition = 1.274;*/
	final double lowerRestPosition;
	final double upperRestPosition;
	final double upperHomePosition;
	final double sallyLowerPosition;
	final double sallyUpperPosition;
	final double drawReachLowerPosition;
	final double drawReachUpperPosition;
	final double drawPressLowerPosition;
	final double drawPressUpperPosition;
	
	double targetLowerPositionRotations;
	double targetUpperPositionRotations;
    boolean isUpperPID = true;
    boolean isLowerPID = true;
	
    public Arm() {
    	//Lower Joint PID setup
		int lowerAbsolutePosition = lowerJoint.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
        lowerJoint.setEncPosition(lowerAbsolutePosition);
        lowerJoint.reverseSensor(true);
        lowerJoint.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        lowerJoint.configEncoderCodesPerRev(7 * 70); //only for quad
        lowerJoint.configNominalOutputVoltage(+0f, -0f);
        lowerJoint.configPeakOutputVoltage(+12f,  -12f);
        
        lowerJoint.setAllowableClosedLoopErr(0);
        lowerJoint.setProfile(0);
        lowerJoint.setF(0.0);
        lowerJoint.setP(2.5);
        lowerJoint.setI(0.0); 
        lowerJoint.setD(0.0);
        
        
        //Upper Joint PID setup
        int upperAbsolutePosition = upperJoint.getPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
        /* use the low level API to set the quad encoder signal */
        upperJoint.setEncPosition(upperAbsolutePosition);
        upperJoint.reverseSensor(true);
        upperJoint.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        upperJoint.configEncoderCodesPerRev(7 * 30); //only for quad
        upperJoint.configNominalOutputVoltage(+0f, -0f);
        upperJoint.configPeakOutputVoltage(+12f,  -12f);
        
        upperJoint.setAllowableClosedLoopErr(0);
        upperJoint.setProfile(0);
        upperJoint.setF(0.0);
        upperJoint.setP(2.5);
        upperJoint.setI(0.0); 
        upperJoint.setD(0.0);
		
        
		lowerStartPosition = lowerJoint.getPosition();
		upperStartPosition = upperJoint.getPosition();
		
		lowerRestPosition = 0;
		upperRestPosition = 0;
		upperHomePosition = .938;
		sallyLowerPosition = 0.314;
		sallyUpperPosition = 0.4;
		drawReachLowerPosition = .382;
		drawReachUpperPosition = .214;
		drawPressLowerPosition = .463;
		drawPressUpperPosition = 1.274;

		targetLowerPositionRotations = lowerRestPosition;
		targetUpperPositionRotations = upperRestPosition;
    }
    
    public void maintainLowerJointPosition() { 
    	double motorOutput = lowerJoint.getOutputVoltage()/lowerJoint.getBusVoltage();
    	sb.append("Lower:");
    	sb.append("\tout:");
	  	sb.append(motorOutput);
	  	sb.append("\tpos:");
        sb.append(lowerJoint.getPosition() );
        lowerJoint.changeControlMode(TalonControlMode.Position);
    	lowerJoint.set(targetLowerPositionRotations);
    	sb.append("\terrNative:");
    	sb.append(lowerJoint.getClosedLoopError());
    	sb.append("\ttrg:");
    	sb.append(targetLowerPositionRotations);
    	if(++loopsL >= 10) {
          	loopsL = 0;
          	System.out.println(sb.toString());
        }
        sb.setLength(0);
    }
    
    public void maintainUpperJointPosition() { 
    	double motorOutput = upperJoint.getOutputVoltage()/upperJoint.getBusVoltage();
    	sb.append("Upper:");
    	sb.append("\tout:");
	  	sb.append(motorOutput);
	  	sb.append("\tpos:");
        sb.append(upperJoint.getPosition() );
        upperJoint.changeControlMode(TalonControlMode.Position);
    	upperJoint.set(targetUpperPositionRotations);
    	sb.append("\terrNative:");
    	sb.append(upperJoint.getClosedLoopError());
    	sb.append("\ttrg:");
    	sb.append(targetUpperPositionRotations);
    	if(++loopsU >= 10) {
          	loopsU = 0;
          	System.out.println(sb.toString());
        }
        sb.setLength(0);
    }
    
    public void setLowerRest() {
    	targetLowerPositionRotations = lowerRestPosition;
    }
    public void setUpperRest() {
    	targetUpperPositionRotations = upperRestPosition;
    }
    public void setUpperHome() {
    	targetUpperPositionRotations = upperHomePosition;
    }
    
    public void setLowerSally() {
    	targetLowerPositionRotations = sallyLowerPosition;
    }
    public void setUpperSally() {
    	targetUpperPositionRotations = sallyUpperPosition;
    }
    
    public void setLowerDrawReach() {
    	targetLowerPositionRotations = drawReachLowerPosition;
    }
    public void setUpperDrawReach() {
    	targetUpperPositionRotations = drawReachUpperPosition;
    }
    public void setLowerDrawPress() {
    	targetLowerPositionRotations = drawPressLowerPosition;
    }
    public void setUpperDrawPress() {
    	targetUpperPositionRotations = drawPressUpperPosition;
    }

    public void stopLowerPID() {
    	isLowerPID = false;
    }
    public void startLowerPID() {
    	isLowerPID = true;
    }
    public void stopUpperPID() {
    	isUpperPID = false;
    }
    public void startUpperPID() {
    	isUpperPID = true;
    }
    
    public void runLowerJointWithXboxController(Joystick s) {
		double y = s.getAxis(AxisType.kY);
		lowerJoint.set(-y);
	}
    public void runUpperJointWithXboxController(Joystick s) {
		double y = s.getAxis(AxisType.kY);
		upperJoint.set(y);
	}
    
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}
    
}

