package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.Robot;
import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


/**
 *
 */
public class Shooter extends Subsystem {

    private final CANTalon shooter = RobotMap.shootershooter;
    
    double targetSpeed = 100.0;
    int loops = 0;
    StringBuilder sb = new StringBuilder();

    public Shooter() {
    	shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    	shooter.reverseSensor(false);
        //motor.configEncoderCodesPerRev(codesPerRev); //only for quad
        
    	shooter.configNominalOutputVoltage(+0.0f, -0.0f);
    	shooter.configPeakOutputVoltage(+12.0f, -0.0f);
        
    	shooter.setProfile(0);
    	shooter.setF(.0284);
    	shooter.setP(.113333);
        shooter.setI(0);
        shooter.setD(0);
        
        shooter.changeControlMode(TalonControlMode.Speed);
    }
    
    public void run() {
    	shooter.changeControlMode(TalonControlMode.PercentVbus);
    	shooter.set(1.0);
    	System.out.println("Speed: " + shooter.getSpeed());
    }
    public void setTargetSpeed(double speed) {
    	targetSpeed = speed;
    }
    
    public void spin() {
    	double motorOutput = shooter.getOutputVoltage()/shooter.getBusVoltage();
    	sb.append("\tout:");
	  	sb.append(motorOutput);
	  	sb.append("\tspd:");
	    sb.append(shooter.getSpeed());
    	shooter.set(targetSpeed);
    	sb.append("\terr:");
        sb.append(shooter.getClosedLoopError());
        sb.append("\ttrg:");
        sb.append(targetSpeed);
    	if(++loops >= 10) {
          	loops = 0;
          	System.out.println(sb.toString());
        }
          sb.setLength(0);
    }
    
    public void stopSpin() { 
    	shooter.set(0);
    }
    
    public void setControllerRumble() {
    	System.out.println(shooter.getSpeed());
    	if (hasAccelerated()) 
        	Robot.oi.getxboxController().setRumble(RumbleType.kLeftRumble, 1.0f);
        else
        	Robot.oi.getxboxController().setRumble(RumbleType.kLeftRumble, 0.0f);
    }
    
    public boolean hasAccelerated() {
    	int delta = 100;
    	return shooter.getSpeed() < (targetSpeed + delta) && shooter.getSpeed() > (targetSpeed - delta); 
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

}

