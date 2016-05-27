package org.usfirst.frc3482.robot;

import org.usfirst.frc3482.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc3482.robot.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public JoystickButton armPositionBridgeButton;
    public JoystickButton lowerIntakeButton;
    public JoystickButton lowerIntakeWheelsButton;
    public JoystickButton invertDirectionButton;
    public JoystickButton runShooterButton;
    public JoystickButton rotateClimberButton;
    public JoystickButton reverseFeedButton;
    public JoystickButton topJointButton;
    public JoystickButton bridgeButton;
    public JoystickButton moveIntakeButton;
    public JoystickButton spinIntakeWheelsButton;
    public JoystickButton runWheelsOnGroundButton;
    public JoystickButton sallyPortButton;
    public JoystickButton portcullisButton;
    public JoystickButton armRestButton;
    public JoystickButton moveIntakeWheelsButton;
    public JoystickButton climberButton;
    public JoystickButton extendClimberButton;
    public JoystickButton rotateTo0Button;
    public JoystickButton rotateTo90Button;
    public JoystickButton rotateTo180Button;
    public JoystickButton rotateTo270Button;
    public JoystickButton rotateTo45Button;
    public JoystickButton rotateTo315Button;
    public JoystickButton rotate360Button;
    public JoystickButton holdSlopeButton;
    public JoystickButton autoShootButton;
    public JoystickButton shootButton;
    
    public Joystick xboxController;
    public Joystick joystick;
    public Joystick arcadeButtons;

    public OI() {
        xboxController = new Joystick(0);
        joystick = new Joystick(1);
        arcadeButtons = new Joystick(2); //2,3,4,5 and 10,11,12,13
        
        //runShooterButton = new JoystickButton(joystick, 2);
        //runShooterButton.whileHeld(new SpinShooter());
        shootButton = new JoystickButton(joystick, 1);
        //shootButton.whenPressed(new SpinAndShoot());
        /*rotateTo0Button = new JoystickButton(xboxController, 4);
        rotateTo0Button.whenPressed(new Rotate(0, false));
        rotateTo90Button = new JoystickButton(xboxController, 2);
        rotateTo90Button.whenPressed(new Rotate(90, false));
        rotateTo180Button = new JoystickButton(xboxController, 1);
        rotateTo180Button.whenPressed(new Rotate(180, false));
        rotateTo270Button = new JoystickButton(xboxController, 3);
        rotateTo270Button.whenPressed(new Rotate(-90, false));
        rotateTo45Button = new JoystickButton(xboxController, 6);
        rotateTo45Button.whenPressed(new Rotate(45, false));
        rotateTo315Button = new JoystickButton(xboxController, 5);
        rotateTo315Button.whenPressed(new Rotate(-45, false));*/
        invertDirectionButton = new JoystickButton(xboxController, 8);
        invertDirectionButton.whenPressed(new InvertDirection());
        holdSlopeButton = new JoystickButton(xboxController, 7);
        //holdSlopeButton.whileHeld(new HoldSlope());
        
        
        autoShootButton = new JoystickButton(arcadeButtons, 2);
        //autoShootButton.whenPressed(new AutoShoot());
        lowerIntakeWheelsButton = new JoystickButton(arcadeButtons, 3);
        //lowerIntakeWheelsButton.whileHeld(new LowerIntake(true));
        portcullisButton = new JoystickButton(arcadeButtons, 4);
        //portcullisButton.whileHeld(new PortcullisIntake());
        lowerIntakeButton = new JoystickButton(arcadeButtons, 5);
        //lowerIntakeButton.whileHeld(new LowerIntake(false));
        reverseFeedButton = new JoystickButton(arcadeButtons, 10);
        //reverseFeedButton.whileHeld(new FeedShooter(false));
        moveIntakeButton = new JoystickButton(arcadeButtons, 11);
        //moveIntakeButton.whileHeld(new MoveIntake());
        moveIntakeWheelsButton = new JoystickButton(arcadeButtons, 12);
        //moveIntakeWheelsButton.whileHeld(new MoveIntake(false));
                
  
       	spinIntakeWheelsButton = new JoystickButton(joystick, 4); 
        //spinIntakeWheelsButton.whileHeld(new SpinIntakeWheels());
//        runWheelsOnGroundButton = new JoystickButton(joystick, 7);
//        runWheelsOnGroundButton.whileHeld(new RunWheelsOnGround());
        //portcullisButton = new JoystickButton(joystick, 12);
        //portcullisButto n.whileHeld(new ArmPositionPortCullis());
        
        climberButton = new JoystickButton(joystick, 7);
        climberButton.whileHeld(new ClimbUp());
        extendClimberButton = new JoystickButton(joystick, 9);
        extendClimberButton.whileHeld(new ExtendClimber());
        rotateClimberButton = new JoystickButton(joystick, 8);
        rotateClimberButton.whileHeld(new RotateClimber());
               
        // SmartDashboard Buttons
        //SmartDashboard.putData("ArmPositionPort", new AutoSallyPort());
        //SmartDashboard.putData("ArmPositionBridge", new ArmPositionBridge());
        SmartDashboard.putData("InvertDirection", new InvertDirection());
        //SmartDashboard.putData("LowerIntake", new LowerIntake(true));
        //SmartDashboard.putData("SpinAndShoot", new SpinAndShoot());
        //SmartDashboard.putData("Shoot", new Shoot());
        SmartDashboard.putData("SpinShooter", new SpinShooter());
        //SmartDashboard.putBoolean("Low Bar", false);
        //SmartDashboard.putBoolean("Port Cullis", false);
        //SmartDashboard.putBoolean("Sally Port", false);
        //SmartDashboard.putBoolean("Cheval de Fris", false);
        //SmartDashboard.putBoolean("Waiting", false);
        //SmartDashboard.putBoolean("GUN IT", false);
        //SmartDashboard.putNumber("Position", 1);
        SmartDashboard.putNumber("Maximum Turn Output", 0.8);
        SmartDashboard.putNumber("Gyro P", 0.01);
        SmartDashboard.putNumber("Gyro I", 0.001);
        SmartDashboard.putNumber("Gyro D", 0.0);
        //SmartDashboard.putData("Move", new Move());
        //SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Drive", new Drive());
    }

    public Joystick getxboxController() {
        return xboxController;
    }
    public Joystick getJoystick() {
    	return joystick;
    }
    public Joystick getArcadeButtons() {
    	return arcadeButtons;
    }
}

