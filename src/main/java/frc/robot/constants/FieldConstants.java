package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldRectangle2d;
import java.util.List;

public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.22);
  public static double fieldWidth = Units.inchesToMeters(317.69);

  public static double centerLineX = fieldLength / 2;
  public static double centerLineY = fieldWidth / 2;
  public static double blueLineX = Units.inchesToMeters(182.11);
  public static double redLineX = fieldLength - Units.inchesToMeters(182.11);

  // All left/right designations are relative to blue alliance station
  public static class Blue {
    public static Translation2d hubTranslation = new Translation2d(blueLineX, centerLineY);
    public static FieldRectangle2d allianceZone =
        new FieldRectangle2d(new Translation2d(blueLineX, fieldWidth), new Translation2d(0, 0));
    public static FieldRectangle2d rightAllianceZone =
        new FieldRectangle2d(new Translation2d(0, 0), new Translation2d(blueLineX, centerLineY));
    public static FieldRectangle2d leftAllianceZone =
        new FieldRectangle2d(
            new Translation2d(0, centerLineY), new Translation2d(blueLineX, fieldWidth));
    public static FieldRectangle2d trenchLeft =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(blueLineX + Units.inchesToMeters(22.20), fieldWidth));
    public static FieldRectangle2d trenchRight =
        new FieldRectangle2d(
            new Translation2d(blueLineX - Units.inchesToMeters(22.20), 0),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20), Units.inchesToMeters(50.59)));

    public static FieldRectangle2d bumpRight =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20), Units.inchesToMeters(50.59 + 12.00)),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(50.59 + 12.00 + 73.00)));

    public static FieldRectangle2d bumpLeft =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00 + 73.00))),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00))));

    public static FieldRectangle2d stopShootLeft =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(blueLineX + Units.inchesToMeters(22.20 + 6), fieldWidth));
    public static FieldRectangle2d stopShootRight =
        new FieldRectangle2d(
            new Translation2d(blueLineX - Units.inchesToMeters(22.20 + 6), 0),
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20 + 6), Units.inchesToMeters(50.59)));

    public static FieldRectangle2d frontOfHub =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20 + 40),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static class Red {
    public static Translation2d hubTranslation = new Translation2d(redLineX, centerLineY);
    public static FieldRectangle2d allianceZone =
        new FieldRectangle2d(
            new Translation2d(fieldLength, fieldWidth), new Translation2d(redLineX, 0));

    public static FieldRectangle2d rightAllianceZone =
        new FieldRectangle2d(
            new Translation2d(redLineX, 0), new Translation2d(fieldLength, centerLineY));
    public static FieldRectangle2d leftAllianceZone =
        new FieldRectangle2d(
            new Translation2d(redLineX, centerLineY), new Translation2d(fieldLength, fieldWidth));

    public static FieldRectangle2d trenchLeft =
        new FieldRectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(redLineX + Units.inchesToMeters(22.20), fieldWidth));
    public static FieldRectangle2d trenchRight =
        new FieldRectangle2d(
            new Translation2d(redLineX - Units.inchesToMeters(22.20), 0),
            new Translation2d(redLineX + Units.inchesToMeters(22.20), Units.inchesToMeters(50.59)));
    public static FieldRectangle2d bumpRight =
        new FieldRectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20), Units.inchesToMeters(50.59 + 12.00)),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(50.59 + 12.00 + 73.00)));

    public static FieldRectangle2d bumpLeft =
        new FieldRectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00 + 73.00))),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                Units.inchesToMeters(fieldWidth - (50.35 + 12.00))));

    public static FieldRectangle2d stopShootLeft =
        new FieldRectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - (24.97 + 12.00))),
            new Translation2d(redLineX + Units.inchesToMeters(22.20 + 6), fieldWidth));
    public static FieldRectangle2d stopShootRight =
        new FieldRectangle2d(
            new Translation2d(redLineX - Units.inchesToMeters(22.20 + 6), 0),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20 + 6), Units.inchesToMeters(50.59)));
    public static FieldRectangle2d frontOfHub =
        new FieldRectangle2d(
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20 + 40),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static class Neutral {
    public static FieldRectangle2d rightNeutral =
        new FieldRectangle2d(
            new Translation2d(blueLineX, 0), new Translation2d(redLineX, centerLineY));
    public static FieldRectangle2d leftNeutral =
        new FieldRectangle2d(
            new Translation2d(blueLineX, centerLineY), new Translation2d(redLineX, fieldWidth));
    public static FieldRectangle2d backBlueHub =
        new FieldRectangle2d(
            new Translation2d(
                blueLineX + Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                blueLineX + Units.inchesToMeters(80.00),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
    public static FieldRectangle2d backRedHub =
        new FieldRectangle2d(
            new Translation2d(
                redLineX - Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                redLineX - Units.inchesToMeters(80.00),
                centerLineY + Units.inchesToMeters(58.41 / 2)));
  }

  public static void plotZones() {
    RobotContainer.getField()
        .getObject("Blue.allianceZone")
        .setPoses(Blue.allianceZone.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.rightAllianceZone")
        .setPoses(Blue.rightAllianceZone.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.leftAllianceZone")
        .setPoses(Blue.leftAllianceZone.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.trenchLeft")
        .setPoses(Blue.trenchLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.trenchRight")
        .setPoses(Blue.trenchRight.getCornerPoses());
    RobotContainer.getField().getObject("Blue.bumpRight").setPoses(Blue.bumpRight.getCornerPoses());
    RobotContainer.getField().getObject("Blue.bumpLeft").setPoses(Blue.bumpLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.stopShootLeft")
        .setPoses(Blue.stopShootLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.stopShootRight")
        .setPoses(Blue.stopShootRight.getCornerPoses());
    RobotContainer.getField()
        .getObject("Blue.frontOfHub")
        .setPoses(Blue.frontOfHub.getCornerPoses());

    RobotContainer.getField()
        .getObject("Red.allianceZone")
        .setPoses(Red.allianceZone.getCornerPoses());
    RobotContainer.getField()
        .getObject("Red.rightAllianceZone")
        .setPoses(Red.rightAllianceZone.getCornerPoses());
    RobotContainer.getField()
        .getObject("Red.leftAllianceZone")
        .setPoses(Red.leftAllianceZone.getCornerPoses());
    RobotContainer.getField().getObject("Red.trenchLeft").setPoses(Red.trenchLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Red.trenchRight")
        .setPoses(Red.trenchRight.getCornerPoses());
    RobotContainer.getField().getObject("Red.bumpRight").setPoses(Red.bumpRight.getCornerPoses());
    RobotContainer.getField().getObject("Red.bumpLeft").setPoses(Red.bumpLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Red.stopShootLeft")
        .setPoses(Red.stopShootLeft.getCornerPoses());
    RobotContainer.getField()
        .getObject("Red.stopShootRight")
        .setPoses(Red.stopShootRight.getCornerPoses());
    RobotContainer.getField().getObject("Red.frontOfHub").setPoses(Red.frontOfHub.getCornerPoses());

    RobotContainer.getField()
        .getObject("Red.hubTranslation")
        .setPose(new Pose2d(Red.hubTranslation, new Rotation2d()));
    RobotContainer.getField()
        .getObject("Blue.hubTranslation")
        .setPose(new Pose2d(Blue.hubTranslation, new Rotation2d()));

    RobotContainer.getField()
        .getObject("Neutral.rightNeutral")
        .setPoses(Neutral.rightNeutral.getCornerPoses());
    RobotContainer.getField()
        .getObject("Neutral.leftNeutral")
        .setPoses(Neutral.leftNeutral.getCornerPoses());
  }
}