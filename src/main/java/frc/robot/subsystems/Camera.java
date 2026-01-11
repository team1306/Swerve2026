

package frc.robot.subsystems;




import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import badgerlog.BadgerLog;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class Camera extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
 
    public final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(1, 1, 1);
    public final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);
    private Matrix<N3, N1> curStdDevs;

    public final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final EstimateConsumer estConsumer;

    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private CommandSwerveDrivetrain drivetrain;

    private Pose2d estPose2d = Pose2d.kZero;

    /**
     * @param drivetrain Drivetrain instance, used for updating camera simulation
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     * @param cameraName Name of the PhotonVision camera
     * @param cameraPosition The position (In meters) of the camera from the origin of the robot
     */
    public Camera(CommandSwerveDrivetrain drivetrain, EstimateConsumer estConsumer, String cameraName, Transform3d cameraPosition) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(cameraName);
        this.drivetrain = drivetrain;
        photonEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        if (Robot.isSimulation()) {
            
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(60);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
           
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            
            visionSim.addCamera(cameraSim, cameraPosition);


            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);


            cameraSim.enableDrawWireframe(true);
        }
    }


    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());


            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est -> getSimDebugField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d()), () -> {
                                    getSimDebugField().getObject("VisionEstimation").setPoses();
                                });
            }


            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();
    
                        // Dog log because badgerlog is changing my values
                        
                        DogLog.log("Cameras/" + camera.getName(),  est.estimatedPose);
                       
                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        
                        
                    });
        }
    }


    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
                                         Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = singleTagStdDevs;


        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = singleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;


            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }


            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = singleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = multiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }


    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }



    @Override
    public void simulationPeriodic() {
            visionSim.update(drivetrain.getState().Pose);
    }


    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }


    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }


    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
