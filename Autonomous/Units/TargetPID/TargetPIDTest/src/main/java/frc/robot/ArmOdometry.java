package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class ArmOdometry {

    private Pose3d armPosition;

    public static ArmOdometry armOdometry = null;

    public static final ArmOdometry getInstance(){
        if(armOdometry == null){
            armOdometry = new ArmOdometry();
        }
        return armOdometry;
    }

    public ArmOdometry(){
        armPosition = new Pose3d();
    }

    public void update(double turretAngle, double shoulderAngle, double armLength){
        double armX = (Math.cos(turretAngle) * (Math.cos(shoulderAngle) * armLength));
        double armY = (Math.sin(turretAngle) * (Math.cos(shoulderAngle) * armLength));
        double armZ = (1.2 + (Math.sin(shoulderAngle) * (armLength + 1.2)));
        armPosition = new Pose3d(armX, armY, armZ, new Rotation3d());
    }

    public Pose3d getEstimatedPosition(){
        return armPosition;
    }

    public void reset(){
        armPosition = new Pose3d();
    }
    
}
