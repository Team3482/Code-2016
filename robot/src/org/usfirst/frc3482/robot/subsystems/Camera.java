package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Camera extends Subsystem {

	private CameraServer server;
	private double tilt = Math.toRadians(31);
	private double hT = 226.06;
	private double hc = 29.21;
	private double wc = 10;
	private double FOVx = 640;
	private double FOVy = 480;
	private double view = Math.toRadians(24.5);
	private double h = hT-hc;
	
    public Camera() {
    	server = CameraServer.getInstance();
        server.setQuality(100); //was 50
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
    }
    /*
    public double targetAngleOffset() {
    	int wp = (int) Math.round(FOVx/2-Robot.centerX);
    	double angle = Math.atan(2*wp*Math.tan(FOVx)/FOVx);
    	double cameraToShooterAngleOffset = Math.atan(wc/12)/targetDistance();
    	return Math.toDegrees(angle+cameraToShooterAngleOffset);
    }
    
    public double targetDistance() {
    	int hp = (int) Math.round(FOVy/2-Robot.centerY);
    	if(hp == 0) {
    		return h/Math.tan(tilt); 
    	}
    	double fov_adjustment = (FOVy*Math.cos(tilt)/2/hp/Math.tan(view))-Math.sin(tilt);
    	return h*fov_adjustment/(1+fov_adjustment*Math.tan(tilt));
    }
*/
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

