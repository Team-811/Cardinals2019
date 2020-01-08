/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.lib.vision.VisionList;
import frc.robot.lib.vision.VisionTarget;
import frc.robot.Constants;
import frc.robot.RobotState;

import java.net.DatagramSocket;
import java.util.ArrayList;
import java.util.List;
import java.net.DatagramPacket;

/**
 * This is a subsystem class.  A subsystem interacts with the hardware components on the robot.  The vision subsystem takes in data from the rasp pi which does the vision 
 * processing and adds targets to the VisionList which keeps a list of vision targets and information about the targets.
 */
public class Vision extends Subsystem implements ISubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Vision instance = new Vision();

  public static Vision getInstance() {
    return instance;
  }


  // Port used for receiving variables
  private final int PORT = 5800;

  // Network temporary variables
  private DatagramSocket socket;
  private byte[] buf = new byte[28];
  private String[] data = new String[4];

  private VisionList targets = new VisionList();
  private RobotState robot_state_ = RobotState.getInstance();

  private NetworkTable table;
  private NetworkTableEntry xEntry;
  private NetworkTableEntry distEntry;
  private NetworkTableEntry angleEntry;




  public Vision() 
  {
    networkTableInit();
  }

  public void onLoop() 
  {
    networkTableUpdate();
  }

  private void udpUpdate()
  {
    try {
      // port 5800-5810
      socket = new DatagramSocket(PORT);
    } catch (Exception e) {
      System.out.println("Can't initialize vision socket");
    }
    // receive information
    DatagramPacket packet = new DatagramPacket(buf, buf.length);
    try {
      socket.receive(packet);
    } catch (Exception e) {
      //Ignore
    }
    // put information into String[]
    data = new String(packet.getData(), 0, packet.getLength()).split(",");

    targets.clearList(); //Clear previous targets before adding more
    
    // parse all data from String[] into variables
    for(int i = 0; i < data.length; i+= 2)
    {
      double xOffset = Double.parseDouble(data[i]);
      double distance = Double.parseDouble(data[i + 1]);
      //double angle = Double.parseDouble(data[i + 2]);

      targets.addTarget(new VisionTarget(xOffset, distance));
    }

    targets.setTimestamp(Timer.getFPGATimestamp() - Constants.cameraLatency); //Set timestamp of targets

    robot_state_.addVisionUpdate(targets.getTimestamp(), targets); //Adds vision targets to robot state class to be used for targeting
  }

  private void networkTableUpdate()
  {
      double[] xOffset;
      double[] distance;
      double[] angle;

      xOffset = xEntry.getDoubleArray(new double[0]);
      distance = distEntry.getDoubleArray(new double[0]);
      angle = angleEntry.getDoubleArray(new double[0]);

      targets.clearList(); //Clear previous targets before adding more

      for(int i = 0; i < xOffset.length; i++)
      {
        targets.addTarget(new VisionTarget(xOffset[i], distance[i]));
      }

      targets.setTimestamp(Timer.getFPGATimestamp() - Constants.cameraLatency); //Set timestamp of targets

      robot_state_.addVisionUpdate(targets.getTimestamp(), targets); //Adds vision targets to robot state class to be used for targeting

  }

  private void networkTableInit()
  {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      table = inst.getTable("VisionTarget");
      xEntry = table.getEntry("xOffset");
      distEntry = table.getEntry("distance");
      angleEntry = table.getEntry("angle");
  }
  

  @Override
  public void outputSmartdashboard() {
    
  }

  @Override
  public void zeroSensors() {
    
  }

  @Override
  public void resetSubsystem() {
    
  }
  
  public int[][][] RyanVision()
  {
    int rows=640;
    int col= 480;
    int pixel= 3;
    try {
      // port 5800-5810
      socket = new DatagramSocket(PORT);
    } catch (Exception e) {
      System.out.println("Can't initialize vision socket");
    }
    // receive information
    DatagramPacket packet = new DatagramPacket(buf, buf.length);
    try {
      socket.receive(packet);
    } catch (Exception e) {
      //Ignore
    }
    // put information into String[]
    data = new String(packet.getData(), 0, packet.getLength()).split(",");
    int [][][] picture = new int [rows][col][pixel];
    // parse all data from String[] into variables
    for(int i = 0; i < data.length; i+= 2)
    {
      for (int row=0; row<rows; row++)
        for (int cols=0; cols<col; cols++)
          for(int pix=0; pix<pixel; pix++)
            picture[row][cols][pix]= Integer.parseInt(data[((row)*cols+cols)*pix+pix]);
      double xOffset = Double.parseDouble(data[i]);
      double distance = Double.parseDouble(data[i + 1]);
      //double angle = Double.parseDouble(data[i + 2]);
    }
    return picture;
  }



  @Override
  public void testSubsystem() {
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

  }
}
