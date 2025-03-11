#pragma once
#include "IRSegmentation.h"
#include "readerwriterqueue.h"
#include <future>
#include <condition_variable>

//*************Pose Tracking Constants************
const float MROI_SCALE_X = 1.0f; // / 1.25f;
const float MROI_SCALE_Y = 1.0f; // / 1.25f;

//Outlier Removal (near/far clip)
const float OUTLIER_NEAR_CLIP = NEAR_CLIP;
const float OUTLIER_FAR_CLIP = FAR_CLIP;

const float MARKER_DIAMETER_ERROR_THRESHOLD = 0.01f; //Allowable difference between actual marker diam and measured

class PoseTracker
{
public:
	struct IRPose {
		IRPose() {
			pose = Eigen::Isometry3d::Identity();
		}

		Eigen::Isometry3d pose;
		std::vector<Eigen::Vector3d> markerPositions;
		std::vector<Eigen::Vector2i> imageCoords;
	};

	explicit PoseTracker(std::shared_ptr<IRSegmentation> irTracker, std::vector<Eigen::Vector3d> geometry, float markerDiameter);
	~PoseTracker();

	/// Update the tracking with a new set of images and the device pose
	/// @param world_T_cam Pose of the camera in the world when the images were captured</param>
	/// @param irIm Infrared reflectivity image for segmenting the markers. Ownership of the pointer passes to this function</param>
	/// @param depthMap Depth map for obtaining the depth value at the segmented locations. Ownership of the pointer passes to this function</param>
	void update(std::unique_ptr<std::vector<uint8_t>> irImLeft, std::unique_ptr<std::vector<uint8_t>> irImRight);

	/// Return the most recent computed pose of the tracked object
	Eigen::Matrix4d getPose();

	/// Return not only the marker pose, but also the positions of the individual spheres in world and image coordinates
	IRPose getLastMeasurement();

	/// Return true if it measured a pose within the last n milliseconds
	bool hasRecentPose(uint64_t milliseconds);

	/// Return true if it has measured a pose that hasn't been fetched yet through getPose or getLastMeasurement
	bool hasNewPose();

	/// Set the smooothing (1 means it never updates and 0 means no smoothing)
	void setSmoothing(const float smoothing);

	/// Set jump detection settings
	/// @param filterJumps turn filtering of jumps on/off
	/// @param jumpThresholdMetres if a point's position changes more than this amount (in metres) between frames, it is considered to have jumped and is ignored
	/// @param numFramesUntilSet if a point jumped but is still there after this many frames, the new position is probably correct so we accept it
	void setJumpSettings(bool filterJumps, const float jumpThresholdMetres, const int numFramesUntilSet);



	//***********Threading Vars**********:
	std::mutex m_poseMutex;

	/// Thread the continuously looks for new detections in the images and then handles the further processing
	std::shared_ptr<std::thread> m_detectionThread;

	/// Atomic flag to control the detection thread
	std::atomic<bool> m_runDetectionThread = true;
	bool m_hasNewPose = false;

	// Condition Variable to notify main when new pose is available
	std::condition_variable m_pose_here_CV;

	/// The current computed object pose
	//Isometry3d m_objectPose = Isometry3d::Identity();
	IRPose m_objectPose;

	//Variables for updating the image
	std::mutex m_imageMutex;
	std::condition_variable m_image_CV;

private:
	struct SensorPacket {
		SensorPacket(std::unique_ptr<std::vector<uint8_t>> ir_left, std::unique_ptr<std::vector<uint8_t>> ir_right )
			: irImLeft(std::move(ir_left))
			, irImRight(std::move(ir_right))
		{}
		std::unique_ptr<std::vector<uint8_t>> irImLeft;
		std::unique_ptr<std::vector<uint8_t>> irImRight;
	};

	/// The IR tracker which actually interacts with the images and finds the keypoints
	std::shared_ptr<IRSegmentation> m_irTracker;

	/// The marker points in local coordinates, in units of metres. Should be arranged so the centroid of the points is the origin
	std::vector<Eigen::Vector3d> m_markerGeom;
	Eigen::MatrixXd m_markerDiffs;

	/// Lockless, concurrent queue to pass images to IR detection thread
	moodycamel::ReaderWriterQueue<SensorPacket> m_detectionQ;		

	/// Some things for the specific tracking implementation
	double m_matchThreshold = 0.008;
	float m_maxPointDistance = 0;
	float m_markerDiameter = 0.011f; // metres
	int m_voteThreshold = 1;
	
	uint64_t m_lastPoseTime = 0;
	bool m_extrapolate = false;
	int m_nPoints;
	int m_roiBuffer = 100;
	int m_width;
	int m_height;

	// Corrections for specific devices used with Unity (left-handed coordinates)
	//bool m_unity;
	IRSegmentation::LogLevel m_logLevel;

	// Search only a region of interest
	Eigen::Vector4i m_roi;
	float m_roiSmoothing = 0.4f;
	float m_roiDeceleration = 0.9f;
	Eigen::Vector2d m_roiVel = { 0,0 };
	uint64_t m_lastRoiTime = 0;

	// Avoid big, sudden jumps
	std::vector<Eigen::Vector3d> m_lastPoints;
	int m_nBadJumps = -1;
	bool m_filterJumps = false;
	float m_jumpThreshold = 0.1f; // A point moving more than this is considered to have jumped
	int m_numJumpFrames = 3; // If the jump remains in place for more than this number of frames, we take it to be the new position

	// Smoothing
	long long m_lastMeasTime = 0;
	double m_smoothing = 0.9;
	Eigen::Quaterniond m_lastRot = Eigen::Quaterniond::Identity();
	Eigen::Vector3d m_lastPos = Eigen::Vector3d(0, 0, 0);

	// Permutations for brute force
	std::vector<std::vector<int>> m_perm3;
	std::vector<std::vector<int>> m_perm4;
	std::vector<std::vector<int>> m_perm5;

	/// The loop function that runs in the detection thread
	void detectionThreadFunction();

	/// Perform some preprocessing, discarding obvious outliers, etc.
	bool preprocessMarkerDetection(std::shared_ptr<IRSegmentation::IrDetection> detection);

	/// Perform processing on the detected keypoints to obtain the object pose
	void processMarkerDetection(std::shared_ptr<IRSegmentation::IrDetection> detection);

	// Functions for the pose computation
	double fillInMissing(std::vector<Eigen::Vector3d>& mes, std::vector<int>& idxs);
	bool tryEuclideanMatching(const std::vector<Eigen::Vector3d>& mesPts, std::vector<int>& idxs);
	std::vector<int> euclideanVotingOutliers(const std::vector<Eigen::Vector3d>& mesPts);
	double bruteForceMatch(const std::vector<Eigen::Vector3d>& mesPts, std::vector<int>& idxsWithOutliers, std::vector<int>& idxs, bool useFixedIdxs);
	void poseCalcFailed();
	Eigen::Matrix3d kabsch(const std::vector<Eigen::Vector3d>& mes, const std::vector<Eigen::Vector3d>& exp);
	bool svd(const Eigen::Matrix3d& mat, Eigen::Matrix3d& U, Eigen::Matrix3d& V);
	bool noBadJumps(const std::vector<Eigen::Vector3d>& mes, std::vector<int> idxs);
	void setPoseFromResult(const std::shared_ptr<IRSegmentation::IrDetection> mes, const std::vector<int> idxs);
	Eigen::Isometry3d transformFromIdxs(const std::vector<Eigen::Vector3d>& mes, const std::vector<int>& idxs);
	Eigen::Isometry3d findTransformFromPoints(const std::vector<Eigen::Vector3d>& mes, const std::vector<Eigen::Vector3d>& exp);
	double meanMatchError(const std::vector<Eigen::Vector3d>& mesPts, const std::vector<int>& idxs);
	bool contains(const std::vector<int>& vec, int val);
	bool badMatch(const std::vector<int>& lst);
	Eigen::Vector3d multPoint3x4(const Eigen::Matrix4d& M, const Eigen::Vector3d& v);
	Eigen::Vector3d multPoint3x4(const Eigen::Isometry3d& M, const Eigen::Vector3d& v);
	Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const float& t);
	std::vector<std::vector<int>> recursiveCreateCombo(const std::vector<int>& ns);
	void setRoi(int minX, int maxX, int minY, int maxY, int buffer);
	void updateRoi(int minX, int maxX, int minY, int maxY);

	/// Current timestamp in milliseconds
	uint64_t time();

};


