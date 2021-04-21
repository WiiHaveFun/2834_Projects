// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Socket extends SubsystemBase {
  DatagramSocket ds;
  byte[] receive;
  List<List<Double>> mostRecentData;

  boolean threadRunning;

  SocketThread thread;

  public Socket(int port, int buffer) {
    // Recommended port: 5801. Recommended buffer: 4096
    try {
      ds = new DatagramSocket(port);
    } catch (SocketException e) {
      e.printStackTrace();
    }

    receive = new byte[buffer];

    mostRecentData = null;

    threadRunning = false;

    thread = new SocketThread();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // receive = new byte[4096];
    // dpReceive = new DatagramPacket(receive, receive.length);
  }

  public static StringBuilder data(byte[] a) { 
      if (a == null) 
          return null; 
      StringBuilder ret = new StringBuilder(); 
      int i = 0; 
      while (a[i] != 0) 
      { 
          ret.append((char) a[i]); 
          i++; 
      } 
      return ret; 
  }

  public List<List<Double>> getObjects() {
    String str = data(receive).toString();
    String objArray[] = str.split("/");
    
    List<List<String>> objList = new ArrayList<List<String>>();
    
    for(String obj : objArray) {
      String strArray[] = obj.split(",");
      
      List<String> strList = new ArrayList<String>();
      strList = Arrays.asList(strArray);
      
      objList.add(strList);
    }

    List<List<Double>> objListDouble = new ArrayList<List<Double>>();

    for(List<String> obj : objList) {
      List<Double> doubleList = new ArrayList<Double>();
      int count = 0;
      for(String coord : obj) {
        try {
          if(count == 2 || count == 0) {
            doubleList.add(-Double.parseDouble(coord));
          } else {
            doubleList.add(Double.parseDouble(coord));
          }
        } catch (Exception e) {
          doubleList.add(0.0);
        }
        count++;
      }
      objListDouble.add(doubleList);
    }

    objListDouble.sort((l1, l2) -> l1.get(2).compareTo(l2.get(2)));

    if(mostRecentData == null) {
      mostRecentData = objListDouble;
    } else if(objList.get(0).size() != 1) {
      mostRecentData = objListDouble;
      receive = new byte[4096];
    }

    return mostRecentData;
  }

  public int getObjectCount() {
    int count;
    if(mostRecentData != null) {
      if(mostRecentData.get(0).size() == 4) {
        count = 0;
      } else {
        count = mostRecentData.size() - 1;
      }
    } else {
      count = 0;
    }

    return count;
  }

  class SocketThread implements Runnable {
    private Thread t;

    SocketThread() {
      t = new Thread(this, "UDP_Thread");
      threadRunning = true;
      t.start();
    }

    @Override
    public void run() {
      while(threadRunning) {
        // receive = new byte[4096];
        DatagramPacket dpReceive = new DatagramPacket(receive, receive.length);

        try {
          ds.receive(dpReceive);
          Thread.sleep(25);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }
  }
}
