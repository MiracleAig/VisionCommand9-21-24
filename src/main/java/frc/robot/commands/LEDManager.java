// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LEDManager extends SubsystemBase {

  
  AddressableLED addLED;
  static DigitalInput intakeInput;
  static DigitalInput transportInput;
  static AddressableLEDBuffer ledBuffer;

  public LEDManager(){

    //Set Inputs & Buffer
    addLED = new AddressableLED(Constants.ledConstants.addressableLEDPort);
    intakeInput = new DigitalInput(Constants.ledConstants.intakeInputChannel);
    transportInput = new DigitalInput(Constants.ledConstants.transportInputChannel);
    ledBuffer = new AddressableLEDBuffer(Constants.ledConstants.ledBufferLength);

    //Configure LEDS
    addLED.setLength(ledBuffer.getLength());
    addLED.setData(ledBuffer);
    addLED.start();
  }

  public static void setLEDS(){

    //Get current game state (is yaw value ideal? is distance for shooting ideal? etc;)
    boolean isIdealYaw = Robot.ats.isIdealYaw();
    boolean isIdealDistance = Robot.ats.isIdealDistance();

    //If april tag is present - change the 0 to the april tag under speaker
    if(Robot.ats.hasTarget() && Robot.ats.getID() == 7){
      // if yaw is in range and distance(z) is in range
      if(isIdealDistance && isIdealYaw){
        for(int i = 0; i < ledBuffer.getLength(); i++){
          // set green
          ledBuffer.setRGB(i, 0, 255, 0);
        }
      }
      else if(isIdealDistance){
        for(int i = 0; i < ledBuffer.getLength(); i++){
          // set orange
          ledBuffer.setRGB(i, 255, 165, 0);
        }
      }
      else if(isIdealYaw){
        for(int i = 0; i < ledBuffer.getLength(); i++){
          // set yellow
          ledBuffer.setRGB(i, 255, 255, 0);
        }
      }
      else{
        for(int i = 0; i < ledBuffer.getLength(); i++){
          // set red
          ledBuffer.setRGB(i, 255, 0, 0);
        }
      }
    }
  }
}
