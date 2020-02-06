// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu
// public class VisionSubsystem extends SubsystemBase
// {
//     NetworkTable table;
//     DriveSubsystem drive = new DriveSubsystem();

//     NetworkTableEntry targetX;
//     NetworkTableEntry targetY;
//     SmartDashboard tx;

//     //Error values for the control loop  
//     double rotationError;
//     double distanceError;

//     //Control Loop constants
//     double KpRot = -0.1;
//     double KpDist = -0.1;

//     double angleTolerance = 1; //Testing
//     double distanceTolerance = 5;

//     double constantForce =0.05; 

//     double rotationAjust;
//     double distanceAjust;
// }