package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;

public class SetPoint {
    private double xSetpoint;
    private double ySetpoint;
    private double tSetpoint;
    private boolean reset;
    private Pose2d pose2d;
    public SetPoint(double xSetpoint, double ySetpoint, double tSetpoint, boolean reset, Pose2d resetPose){
        this.tSetpoint = tSetpoint;
        this.xSetpoint = xSetpoint;
        this.ySetpoint = ySetpoint;
        this.reset = reset;
        this.pose2d = resetPose;
    }

    public double getxSetpoint(){
        return xSetpoint;
    }

    public double getySetpoint(){
        return ySetpoint;
    }

    public double gettSetpoint(){
        return tSetpoint;
    }
    public boolean getReset(){
        return reset;
    }

    public Pose2d getPose2d() {
        return pose2d;
    }
}