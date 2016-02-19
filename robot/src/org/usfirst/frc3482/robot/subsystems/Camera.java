package org.usfirst.frc3482.robot.subsystems;

import org.usfirst.frc3482.robot.RobotMap;
import org.usfirst.frc3482.robot.commands.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Camera extends Subsystem {

	CameraServer server;
	
    public Camera() {
    	server = CameraServer.getInstance();
        server.setQuality(100); //was 50
        //the camera name (ex "cam0") can be found through the roborio web interface
        server.startAutomaticCapture("cam0");
    }


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}

