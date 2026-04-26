package org.firstinspires.ftc.teamcode.subsystems;
import com.pedropathing.geometry.Pose;
public class PoseStorage {
    public enum AutoRoute {
        CLOSE_BLUE, FAR_BLUE, CLOSE_RED, FAR_RED, BLUE_RESET, RED_RESET, NONE
    }

    // Storage
    public static Pose currentPose = null; // null helps us detect "No Auto Run"
    public static AutoRoute lastRoute = AutoRoute.NONE;

    // Default Starting Positions (Failsafes)
    public static Pose getFailsafePose(AutoRoute route) {
        switch (route) {
            case CLOSE_BLUE: return new Pose(49.748, 115.673,Math.toRadians(90));
            case FAR_BLUE:   return new Pose(7.7, 124.5, Math.toRadians(0));
            case CLOSE_RED:  return new Pose(136.3, 8.1, Math.toRadians(180));
            case FAR_RED:    return new Pose(136.3, 124.5, Math.toRadians(180));
            case BLUE_RESET:  return new Pose(136.3, 8.1, Math.toRadians(0));
            case RED_RESET:    return new Pose(7.7, 8.1, Math.toRadians(180));
            default:         return new Pose(0, 0, 0); // Absolute fallback
        }
    }
}