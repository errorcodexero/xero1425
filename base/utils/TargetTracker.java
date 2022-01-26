package org.xero1425.base.utils;

import org.xero1425.misc.XeroMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TargetTracker {
    
    Translation2d target_ = new Translation2d() ;
    // TODO: support turret offset in further math calculations.
    Translation2d turret_offset_ = new Translation2d() ;

    // \param target the position of the goal/target on the field
    public TargetTracker(Translation2d target) throws Exception {
        target_ = target ;
        turret_offset_ = new Translation2d(0, 0) ;
    }

    // \param target  the position of the goal/arget on the field
    // \param turret_offset the offset of the turret from the center of the robot (if back/right of robot is aligned to the field's (0, 0) corner)
    public TargetTracker(Translation2d target, Translation2d turret_offset) throws Exception {
        target_ = target ;
        turret_offset_ = turret_offset ;
    }

    // \returns double of relative angle difference; unit = degrees
    public double getRelativeTargetAngle(Pose2d robot) {
        
        //following math [in some way/shape/form]
        //  Gtheta = arctan((Gy - Ty)/(Gx - Tx))
        //  Ttheta = Gtheta - Rtheta

        Translation2d rel_target_ = target_.minus(robot.getTranslation()) ;
        Rotation2d rel_target_angle_ = new Rotation2d(rel_target_.getX(), rel_target_.getY()) ;

        double rel_target_angle_degrees_ = rel_target_angle_.getDegrees() - robot.getRotation().getDegrees() ;
            
        return XeroMath.normalizeAngleDegrees(rel_target_angle_degrees_) ;
    }

    // \returns double distance
    public double getRelativeTargetDistance(Pose2d robot) {

        return robot.getTranslation().getDistance(target_);
    }

}
