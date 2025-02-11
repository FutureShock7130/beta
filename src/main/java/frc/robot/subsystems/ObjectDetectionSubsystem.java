package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;

public class ObjectDetectionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    public ObjectDetectionSubsystem() {
        camera = new PhotonCamera("WEB_CAM");
    }

    @Override
    public void periodic() {
        // 每個週期更新相機結果
        latestResult = camera.getLatestResult();
    }

    /**
     * 獲取最大目標
     * @return 如果找到目標則返回 PhotonTrackedTarget，否則返回 null
     */
    public PhotonTrackedTarget getBestTarget() {
        if (latestResult.hasTargets()) {
            return latestResult.getBestTarget();
        }
        return null;
    }

    /**
     * 檢查是否有找到任何目標
     * @return 是否有目標
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }


    /**
     * 獲取目標的水平角度
     * @return 如果有目標則返回偏航角，否則返回 0
     */
    public double getTargetYaw() {
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            return target.getYaw();
        }
        return 0.0;
    }

    /**
     * 獲取目標的垂直角度
     * @return 如果有目標則返回俯仰角，否則返回 0
     */
    public double getTargetPitch() {
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            return target.getPitch();
        }
        return 0.0;
    }

    /**
     * 獲取目標的面積（佔畫面比例）
     * @return 如果有目標則返回面積百分比，否則返回 0
     */
    public double getTargetArea() {
        PhotonTrackedTarget target = getBestTarget();
        if (target != null) {
            return target.getArea();
        }
        return 0.0;
    }
}