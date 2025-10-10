// package frc.robot.subsystems;

// import org.opencv.core.Mat;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class camera extends SubsystemBase {
    
    
//     public camera() {
//         // Start the camera server
//         UsbCamera camera = CameraServer.startAutomaticCapture("IntakeCamera",0);
//         UsbCamera camera2 = CameraServer.startAutomaticCapture("ClimbCamera",1);
//         camera.setResolution(640, 480);
//         camera2.setResolution(640,480);
//         camera.setFPS(30);
//         camera2.setFPS(30);
//     }

//     public void periodic() {}


// }