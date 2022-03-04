// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.lang.Math;
import frc.robot.Constants;
public class VisionSubsystem extends SubsystemBase {
    private NetworkTable nNetworkTable;
    public VisionSubsystem(NetworkTable nNetworkTable) { //No idea why this is here
        this.nNetworkTable = nNetworkTable;
    }
    private double targetHeightInches; 
    private double llMountHeightInches;
    private double llMountAngle;
    private double AngletoGoalRadians;
    private double AngletoGoalDegree;
    private double ty, tx;
    private double targetOffsetAngle_Vertical;

public void llvision() {
    nNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    targetHeightInches = 104;
    llMountHeightInches = 21;
    llMountAngle = 134.0;
    AngletoGoalDegree = llMountAngle + targetOffsetAngle_Vertical;
    AngletoGoalRadians = AngletoGoalDegree * (3.14159 / 180.0); //convert degrees to radians
    
   // double targetOffsetAngle_Vertical = ty.getDouble(0.0);

}



@Override
public void periodic(){
    ty = nNetworkTable.getEntry("<ty>").getDouble(0);
    tx = nNetworkTable.getEntry("<tx>").getDouble(0);
    
}

public double targetdistance(){
    double distancefromlltotarget = (targetHeightInches - llMountHeightInches)/Math.tan(AngletoGoalRadians);
    return(distancefromlltotarget);
}


}