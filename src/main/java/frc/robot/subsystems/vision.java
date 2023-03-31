package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class vision {
    // Limelight
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public static NetworkTableEntry tx = table.getEntry("tx");
  public static NetworkTableEntry ty = table.getEntry("ty");
  public static NetworkTableEntry ta = table.getEntry("ta");
    public static double xspeed;
    public static double yspeed;
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

    
}
