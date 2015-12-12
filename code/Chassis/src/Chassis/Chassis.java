package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

/**
 * <p>Entry point for chassis lab.</p>
 **/
public class Chassis {

  /**
   * <p>The robot.</p>
   **/
  public static OdometryRobot robot = new OdometryRobot();

  /**
   * <p>Entry point for the chassis lab.</p>
   *
   * @param args command line arguments
   **/
  public static void main(String [] args) {

    //////////////////// create velocity controller //////////////////////    

    RobotVelocityController robotVelocityController =
      new RobotVelocityControllerBalanced();

    robot.setRobotVelocityController(robotVelocityController);


    //////////////////// create position controller /////////////////////    

    RobotPositionController robotPositionController =
      new RobotPositionController(robot);

    robot.setRobotPositionController(robotPositionController);


    //////////////////// config controllers /////////////////////////////    

    //if your velocity and/or position controllers need to be configured
    //(i.e. gains set, etc), then do it here

    //this block to configures *our* solution to lab 2 (yours may or
    //may not be configured the same way)
    final double VELOCITY_BALANCE_GAIN = 0.4; 
    final double VELOCITY_WHEEL_GAIN = 30.0;
    robotVelocityController.setGain(VELOCITY_BALANCE_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.LEFT). 
      setGain(VELOCITY_WHEEL_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.RIGHT).
      setGain(VELOCITY_WHEEL_GAIN);



    //////////////////// display estop button //////////////////////////    

    EstopButton estop = new EstopButton(Thread.currentThread());


    //////////////////// command motion ////////////////////////////////    

    // percent_angle is an empirically determined value to prevent
    // overshooting because of the control loop time
    double percent_angle = 0.95;
    double angle = 3* Math.PI /2;
    //double angle = Math.PI/2;
    
    robot.enableMotors(true);

    robotPositionController.rotate(0.5, angle * percent_angle);
    
    /*
    for (int i = 0; i < 4; i++){
    	// move forward in a straight line for 1 meter at 2 rad/s
    	// wheel revolution speed
        robotPositionController.translate(2.0, 1.0);
        
        // turn 90 degrees at 1 rad/s wheel revolution speed
        robotPositionController.rotate(1.0, angle * percent_angle);
    }*/
    

    //////////////////// shutdown //////////////////////////////////////    

    //robot should already be stopped, but just in case
    robot.estop();
    System.exit(0);
  }

  /**
   * <p>The estop button.</p>
   **/
  protected static class EstopButton extends JDialog {
    
    EstopButton(final Thread mainThread) {

      JButton eb = new JButton("ESTOP");
      eb.setBackground(Color.RED);
      eb.setPreferredSize(new Dimension(200, 200));
      eb.addActionListener(new ActionListener() {
          public void actionPerformed(ActionEvent e) {
            mainThread.interrupt();
            robot.estop();
            System.exit(-1);
          }
        });

      setContentPane(eb);
      setTitle("Emergency Stop");
      setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
      pack();
      setVisible(true);
    }
  }
} 
