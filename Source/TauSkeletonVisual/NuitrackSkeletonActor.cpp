// Fill out your copyright notice in the Description page of Project Settings.


#include "NuitrackSkeletonActor.h"

using tdv::nuitrack::Nuitrack;
using tdv::nuitrack::JointType;

// Sets default values
ANuitrackSkeletonActor::ANuitrackSkeletonActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	DidInitNuitrack = false;
}

// Sets default values
ANuitrackSkeletonActor::~ANuitrackSkeletonActor()
{
	Nuitrack::release();
}


// Called when the game starts or when spawned
void ANuitrackSkeletonActor::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Warning, TEXT("BeginPlay"));

	this->JointBuffer = NewObject<UNuitrackSkeletonJointBuffer>(this);
	JointBuffer->RegisterComponent();

	if (!DidInitNuitrack) {
		Nuitrack::release();
		UE_LOG(LogTemp, Warning, TEXT("Nuitrack::init() CALLING..."));
		Nuitrack::init();

		UE_LOG(LogTemp, Warning, TEXT("SkeletonTracker::create() CALLING..."));
		skeletonTracker = SkeletonTracker::create();

		UE_LOG(LogTemp, Warning, TEXT(
			"skeletonTracker->connectOnUpdate() CALLING..."));
		skeletonTracker->connectOnUpdate(
			std::bind(&ANuitrackSkeletonActor::OnSkeletonUpdate,
				this, std::placeholders::_1));

		UE_LOG(LogTemp, Warning, TEXT("Nuitrack::run() CALLING..."));
		Nuitrack::run();
		DidInitNuitrack = true;
	}
	AssignedId = -1;
	ReadyForUpdate = false;
}

// Called every frame
void ANuitrackSkeletonActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	LastDeltaTime = DeltaTime;		
	Nuitrack::update();

	if (ReadyForUpdate && JointBuffer != nullptr) {

		//JointBuffer->UpdateTrackingRenderTargets();
		//UE_LOG(LogTemp, Warning, TEXT("Buffer did update"));
		SkeletonJointBufferDidUpdate();
		ReadyForUpdate = false;
	}
}


void ANuitrackSkeletonActor::OnSkeletonUpdate(SkeletonData::Ptr userSkeletons)
{
	auto skeletons = userSkeletons->getSkeletons();

	if (!skeletons.empty())
	{
		//UE_LOG(LogTemp, Warning, TEXT("Nuitrack::OnSkeletonUpdate()..."));
		for (auto skeleton : skeletons)
		{
			if (AssignedId == -1) {
				AssignedId = skeleton.id; // TODO: Detect Unusued IDs;
			}

			//DrawSkeleton(skeleton.id, skeleton.joints);
			UpdateJointBuffer(skeleton.joints);
			//UE_LOG(LogTemp, Warning, TEXT("Processing socket raw data for time: %f"), LastDeltaTime);
			
			JointBuffer->ProcessSocketRawData(LastDeltaTime);
		}
		if (JointBuffer->SocketBoneNames.Num() == JointBuffer->SocketLocations.Num()) {
			ReadyForUpdate = true;
		}
	}
	else {
		ReadyForUpdate = true;
	}
}


/**
 * @ingroup SkeletonTracker_group
 * @brief Joint index meaning (please note that <i>JOINT_LEFT_FINGERTIP, JOINT_RIGHT_FINGERTIP, JOINT_LEFT_FOOT, JOINT_RIGHT_FOOT</i> are not used in the current version).

enum NT_JOINT
{
	JOINT_NONE = 0, ///< Reserved joint (unused).

	JOINT_HEAD = 1, ///< Head
	JOINT_NECK = 2, ///< Neck
	JOINT_TORSO = 3, ///< Torso
	JOINT_WAIST = 4, ///< Waist

	JOINT_LEFT_COLLAR = 5, ///< Left collar
	JOINT_LEFT_SHOULDER = 6, ///< Left shoulder
	JOINT_LEFT_ELBOW = 7, ///< Left elbow
	JOINT_LEFT_WRIST = 8, ///< Left wrist
	JOINT_LEFT_HAND = 9, ///< Left hand
	JOINT_LEFT_FINGERTIP = 10, ///< Left fingertip (<b>not used in the current version</b>).

	JOINT_RIGHT_COLLAR = 11, ///< Right collar
	JOINT_RIGHT_SHOULDER = 12, ///< Right shoulder
	JOINT_RIGHT_ELBOW = 13, ///< Right elbow
	JOINT_RIGHT_WRIST = 14, ///< Right wrist
	JOINT_RIGHT_HAND = 15, ///< Right hand
	JOINT_RIGHT_FINGERTIP = 16, ///< Right fingertip (<b>not used in the current version</b>).

	JOINT_LEFT_HIP = 17, ///< Left hip
	JOINT_LEFT_KNEE = 18, ///< Left knee
	JOINT_LEFT_ANKLE = 19, ///< Left ankle
	JOINT_LEFT_FOOT = 20, ///< Left foot (<b>not used in the current version</b>).

	JOINT_RIGHT_HIP = 21, ///< Right hip
	JOINT_RIGHT_KNEE = 22, ///< Right knee
	JOINT_RIGHT_ANKLE = 23, ///< Right ankle
	JOINT_RIGHT_FOOT = 24 ///< Right foot (<b>not used in the current version</b>).
};
 */

void ANuitrackSkeletonActor::UpdateJointBuffer(std::vector<Joint> joints)
{
	if (joints.empty())
		return;

	FName headName = FName(TEXT("Head"));
	FName neckName = FName(TEXT("Neck"));
	FName torsoName = FName(TEXT("Torso"));
	FName waistName = FName(TEXT("Waist"));
	//FName leftCollarName = FName(TEXT("LeftCollar"));
	FName leftShoulderName = FName(TEXT("LeftShoulder"));
	FName leftElbowName = FName(TEXT("LeftElbow"));
	FName leftWristName = FName(TEXT("LeftWrist"));
	FName leftHandName = FName(TEXT("LeftHand"));
	//FName rightCollarName = FName(TEXT("RightCollar"));
	FName rightShoulderName = FName(TEXT("RightShoulder"));
	FName rightElbowName = FName(TEXT("RightElbow"));
	FName rightWristName = FName(TEXT("RightWrist"));
	FName rightHandName = FName(TEXT("RightHand"));
	FName leftHipName = FName(TEXT("LeftHip"));
	FName leftKneeName = FName(TEXT("LeftKnee"));
	FName leftAnkleName = FName(TEXT("LeftAnkle"));
	FName rightHipName = FName(TEXT("RightHip"));
	FName rightKneeName = FName(TEXT("RightKnee"));
	FName rightAnkleName = FName(TEXT("RightAnkle"));

	FVector headProjectedPosition = FVector(joints[JointType::JOINT_HEAD].proj.x, joints[JointType::JOINT_HEAD].proj.y, joints[JointType::JOINT_HEAD].proj.z);
	FVector neckProjectedPosition = FVector(joints[JointType::JOINT_NECK].proj.x, joints[JointType::JOINT_NECK].proj.y, joints[JointType::JOINT_NECK].proj.z);
	FVector torsoProjectedPosition = FVector(joints[JointType::JOINT_TORSO].proj.x, joints[JointType::JOINT_TORSO].proj.y, joints[JointType::JOINT_TORSO].proj.z);
	FVector waistProjectedPosition = FVector(joints[JointType::JOINT_WAIST].proj.x, joints[JointType::JOINT_WAIST].proj.y, joints[JointType::JOINT_WAIST].proj.z);
	FVector leftShoulderProjectedPosition = FVector(joints[JointType::JOINT_LEFT_SHOULDER].proj.x, joints[JointType::JOINT_LEFT_SHOULDER].proj.y, joints[JointType::JOINT_LEFT_SHOULDER].proj.z);
	FVector leftElbowProjectedPosition = FVector(joints[JointType::JOINT_LEFT_ELBOW].proj.x, joints[JointType::JOINT_LEFT_ELBOW].proj.y, joints[JointType::JOINT_LEFT_ELBOW].proj.z);
	FVector leftWristProjectedPosition = FVector(joints[JointType::JOINT_LEFT_WRIST].proj.x, joints[JointType::JOINT_LEFT_WRIST].proj.y, joints[JointType::JOINT_LEFT_WRIST].proj.z);
	FVector leftHandProjectedPosition = FVector(joints[JointType::JOINT_LEFT_HAND].proj.x, joints[JointType::JOINT_LEFT_HAND].proj.y, joints[JointType::JOINT_LEFT_HAND].proj.z);
	FVector rightShoulderProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_SHOULDER].proj.x, joints[JointType::JOINT_RIGHT_SHOULDER].proj.y, joints[JointType::JOINT_RIGHT_SHOULDER].proj.z);
	FVector rightElbowProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_ELBOW].proj.x, joints[JointType::JOINT_RIGHT_ELBOW].proj.y, joints[JointType::JOINT_RIGHT_ELBOW].proj.z);
	FVector rightWristProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_WRIST].proj.x, joints[JointType::JOINT_RIGHT_WRIST].proj.y, joints[JointType::JOINT_RIGHT_WRIST].proj.z);
	FVector rightHandProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_HAND].proj.x, joints[JointType::JOINT_RIGHT_HAND].proj.y, joints[JointType::JOINT_RIGHT_HAND].proj.z);
	FVector leftHipProjectedPosition = FVector(joints[JointType::JOINT_LEFT_HIP].proj.x, joints[JointType::JOINT_LEFT_HIP].proj.y, joints[JointType::JOINT_LEFT_HIP].proj.z);
	FVector leftKneeProjectedPosition = FVector(joints[JointType::JOINT_LEFT_KNEE].proj.x, joints[JointType::JOINT_LEFT_KNEE].proj.y, joints[JointType::JOINT_LEFT_KNEE].proj.z);
	FVector leftAnkleProjectedPosition = FVector(joints[JointType::JOINT_LEFT_ANKLE].proj.x, joints[JointType::JOINT_LEFT_ANKLE].proj.y, joints[JointType::JOINT_LEFT_ANKLE].proj.z);
	FVector rightHipProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_HIP].proj.x, joints[JointType::JOINT_RIGHT_HIP].proj.y, joints[JointType::JOINT_RIGHT_HIP].proj.z);
	FVector rightKneeProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_KNEE].proj.x, joints[JointType::JOINT_RIGHT_KNEE].proj.y, joints[JointType::JOINT_RIGHT_KNEE].proj.z);
	FVector rightAnkleProjectedPosition = FVector(joints[JointType::JOINT_RIGHT_ANKLE].proj.x, joints[JointType::JOINT_RIGHT_ANKLE].proj.y, joints[JointType::JOINT_RIGHT_ANKLE].proj.z);

	FVector headRealPosition = FVector(joints[JointType::JOINT_HEAD].real.x, joints[JointType::JOINT_HEAD].real.y, joints[JointType::JOINT_HEAD].real.z);
	FVector neckRealPosition = FVector(joints[JointType::JOINT_NECK].real.x, joints[JointType::JOINT_NECK].real.y, joints[JointType::JOINT_NECK].real.z);
	FVector torsoRealPosition = FVector(joints[JointType::JOINT_TORSO].real.x, joints[JointType::JOINT_TORSO].real.y, joints[JointType::JOINT_TORSO].real.z);
	FVector waistRealPosition = FVector(joints[JointType::JOINT_WAIST].real.x, joints[JointType::JOINT_WAIST].real.y, joints[JointType::JOINT_WAIST].real.z);
	FVector leftShoulderRealPosition = FVector(joints[JointType::JOINT_LEFT_SHOULDER].real.x, joints[JointType::JOINT_LEFT_SHOULDER].real.y, joints[JointType::JOINT_LEFT_SHOULDER].real.z);
	FVector leftElbowRealPosition = FVector(joints[JointType::JOINT_LEFT_ELBOW].real.x, joints[JointType::JOINT_LEFT_ELBOW].real.y, joints[JointType::JOINT_LEFT_ELBOW].real.z);
	FVector leftWristRealPosition = FVector(joints[JointType::JOINT_LEFT_WRIST].real.x, joints[JointType::JOINT_LEFT_WRIST].real.y, joints[JointType::JOINT_LEFT_WRIST].real.z);
	FVector leftHandRealPosition = FVector(joints[JointType::JOINT_LEFT_HAND].real.x, joints[JointType::JOINT_LEFT_HAND].real.y, joints[JointType::JOINT_LEFT_HAND].real.z);
	FVector rightShoulderRealPosition = FVector(joints[JointType::JOINT_RIGHT_SHOULDER].real.x, joints[JointType::JOINT_RIGHT_SHOULDER].real.y, joints[JointType::JOINT_RIGHT_SHOULDER].real.z);
	FVector rightElbowRealPosition = FVector(joints[JointType::JOINT_RIGHT_ELBOW].real.x, joints[JointType::JOINT_RIGHT_ELBOW].real.y, joints[JointType::JOINT_RIGHT_ELBOW].real.z);
	FVector rightWristRealPosition = FVector(joints[JointType::JOINT_RIGHT_WRIST].real.x, joints[JointType::JOINT_RIGHT_WRIST].real.y, joints[JointType::JOINT_RIGHT_WRIST].real.z);
	FVector rightHandRealPosition = FVector(joints[JointType::JOINT_RIGHT_HAND].real.x, joints[JointType::JOINT_RIGHT_HAND].real.y, joints[JointType::JOINT_RIGHT_HAND].real.z);
	FVector leftHipRealPosition = FVector(joints[JointType::JOINT_LEFT_HIP].real.x, joints[JointType::JOINT_LEFT_HIP].real.y, joints[JointType::JOINT_LEFT_HIP].real.z);
	FVector leftKneeRealPosition = FVector(joints[JointType::JOINT_LEFT_KNEE].real.x, joints[JointType::JOINT_LEFT_KNEE].real.y, joints[JointType::JOINT_LEFT_KNEE].real.z);
	FVector leftAnkleRealPosition = FVector(joints[JointType::JOINT_LEFT_ANKLE].real.x, joints[JointType::JOINT_LEFT_ANKLE].real.y, joints[JointType::JOINT_LEFT_ANKLE].real.z);
	FVector rightHipRealPosition = FVector(joints[JointType::JOINT_RIGHT_HIP].real.x, joints[JointType::JOINT_RIGHT_HIP].real.y, joints[JointType::JOINT_RIGHT_HIP].real.z);
	FVector rightKneeRealPosition = FVector(joints[JointType::JOINT_RIGHT_KNEE].real.x, joints[JointType::JOINT_RIGHT_KNEE].real.y, joints[JointType::JOINT_RIGHT_KNEE].real.z);
	FVector rightAnkleRealPosition = FVector(joints[JointType::JOINT_RIGHT_ANKLE].real.x, joints[JointType::JOINT_RIGHT_ANKLE].real.y, joints[JointType::JOINT_RIGHT_ANKLE].real.z);

	FTransform headOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_HEAD].orient));
	FTransform neckOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_NECK].orient));
	FTransform torsoOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_TORSO].orient));
	FTransform waistOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_WAIST].orient));
	FTransform leftShoulderOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_SHOULDER].orient));
	FTransform leftElbowOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_ELBOW].orient));
	FTransform leftWristOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_WRIST].orient));
	FTransform leftHandOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_HAND].orient));
	FTransform rightShoulderOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_SHOULDER].orient));
	FTransform rightElbowOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_ELBOW].orient));
	FTransform rightWristOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_WRIST].orient));
	FTransform rightHandOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_HAND].orient));
	FTransform leftHipOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_HIP].orient));
	FTransform leftKneeOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_KNEE].orient));
	FTransform leftAnkleOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_LEFT_ANKLE].orient));
	FTransform rightHipOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_HIP].orient));
	FTransform rightKneeOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_KNEE].orient));
	FTransform rightAnkleOrientationTransform = FTransform(OrientationMatrixToQuaternion(joints[JointType::JOINT_RIGHT_ANKLE].orient));


	TArray<FName> SocketNames;
	SocketNames.Init(headName, 1);
	SocketNames.Emplace(neckName);
	SocketNames.Emplace(torsoName);
	SocketNames.Emplace(waistName);
	//BoneNames.Emplace(leftCollarName);
	SocketNames.Emplace(leftShoulderName);
	SocketNames.Emplace(leftElbowName);
	SocketNames.Emplace(leftWristName);
	SocketNames.Emplace(leftHandName);
	//BoneNames.Emplace(rightCollarName);
	SocketNames.Emplace(rightShoulderName);
	SocketNames.Emplace(rightElbowName);
	SocketNames.Emplace(rightWristName);
	SocketNames.Emplace(rightHandName);
	SocketNames.Emplace(leftHipName);
	SocketNames.Emplace(leftKneeName);
	SocketNames.Emplace(leftAnkleName);
	SocketNames.Emplace(rightHipName);
	SocketNames.Emplace(rightKneeName);
	SocketNames.Emplace(rightAnkleName);

	TArray<FVector> SocketLocations;
	SocketLocations.Init(RealToPosition(headRealPosition), 1);
	SocketLocations.Emplace(RealToPosition(neckRealPosition));
	SocketLocations.Emplace(RealToPosition(torsoRealPosition));
	SocketLocations.Emplace(RealToPosition(waistRealPosition));
	SocketLocations.Emplace(RealToPosition(leftShoulderRealPosition));
	SocketLocations.Emplace(RealToPosition(leftElbowRealPosition));
	SocketLocations.Emplace(RealToPosition(leftWristRealPosition));
	SocketLocations.Emplace(RealToPosition(leftHandRealPosition));
	SocketLocations.Emplace(RealToPosition(rightShoulderRealPosition));
	SocketLocations.Emplace(RealToPosition(rightElbowRealPosition));
	SocketLocations.Emplace(RealToPosition(rightWristRealPosition));
	SocketLocations.Emplace(RealToPosition(rightHandRealPosition));
	SocketLocations.Emplace(RealToPosition(leftHipRealPosition));
	SocketLocations.Emplace(RealToPosition(leftKneeRealPosition));
	SocketLocations.Emplace(RealToPosition(leftAnkleRealPosition));
	SocketLocations.Emplace(RealToPosition(rightHipRealPosition));
	SocketLocations.Emplace(RealToPosition(rightKneeRealPosition));
	SocketLocations.Emplace(RealToPosition(rightAnkleRealPosition));

	TArray<FRotator> SocketRotations;
	SocketRotations.Init(headOrientationTransform.Rotator(), 1);
	SocketRotations.Emplace(neckOrientationTransform.Rotator());
	SocketRotations.Emplace(torsoOrientationTransform.Rotator());
	SocketRotations.Emplace(waistOrientationTransform.Rotator());
	SocketRotations.Emplace(leftShoulderOrientationTransform.Rotator());
	SocketRotations.Emplace(leftElbowOrientationTransform.Rotator());
	SocketRotations.Emplace(leftWristOrientationTransform.Rotator());
	SocketRotations.Emplace(leftHandOrientationTransform.Rotator());
	SocketRotations.Emplace(rightShoulderOrientationTransform.Rotator());
	SocketRotations.Emplace(rightElbowOrientationTransform.Rotator());
	SocketRotations.Emplace(rightWristOrientationTransform.Rotator());
	SocketRotations.Emplace(rightHandOrientationTransform.Rotator());
	SocketRotations.Emplace(leftHipOrientationTransform.Rotator());
	SocketRotations.Emplace(leftKneeOrientationTransform.Rotator());
	SocketRotations.Emplace(leftAnkleOrientationTransform.Rotator());
	SocketRotations.Emplace(rightHipOrientationTransform.Rotator());
	SocketRotations.Emplace(rightKneeOrientationTransform.Rotator());
	SocketRotations.Emplace(rightAnkleOrientationTransform.Rotator());

	TArray<float> SocketConfidences;
	SocketConfidences.Init(joints[JointType::JOINT_HEAD].confidence, 1);
	SocketConfidences.Emplace(joints[JointType::JOINT_NECK].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_TORSO].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_WAIST].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_SHOULDER].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_ELBOW].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_WRIST].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_HAND].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_SHOULDER].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_ELBOW].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_WRIST].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_HAND].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_HIP].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_KNEE].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_LEFT_ANKLE].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_HIP].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_KNEE].confidence);
	SocketConfidences.Emplace(joints[JointType::JOINT_RIGHT_ANKLE].confidence);

	if (SocketNames.Num() != SocketLocations.Num()) {
		UE_LOG(LogTemp, Warning, TEXT("Warning: Check socket locations size"));
	}
	else if (SocketNames.Num() != SocketRotations.Num()) {
		UE_LOG(LogTemp, Warning, TEXT("Warning: Check socket rotations size"));
	}
	else if (SocketNames.Num() != SocketConfidences.Num()) {
		UE_LOG(LogTemp, Warning, TEXT("Warning: Check socket confidences size"));
	}

	JointBuffer->UpdateSocketRawData(SocketNames, SocketLocations, SocketRotations, SocketConfidences);
	JointBuffer->InitCalculations(SocketNames, SocketLocations, SocketRotations, SocketConfidences, JointBuffer->TriangleTauBuffers);
}

void ANuitrackSkeletonActor::DrawSkeleton(int skeleton_index, std::vector<Joint> joints)
{
	if (joints.empty())
		return;

	//DrawBone(joints[JointType::JOINT_HEAD], joints[JointType::JOINT_NECK]);
	//DrawBone(joints[JointType::JOINT_NECK], joints[JointType::JOINT_TORSO]);
	//DrawBone(joints[JointType::JOINT_RIGHT_SHOULDER], joints[JointType::JOINT_LEFT_SHOULDER]);
	//DrawBone(joints[JointType::JOINT_WAIST], joints[JointType::JOINT_LEFT_HIP]);
	//DrawBone(joints[JointType::JOINT_WAIST], joints[JointType::JOINT_RIGHT_HIP]);
	//DrawBone(joints[JointType::JOINT_TORSO], joints[JointType::JOINT_WAIST]);
	//DrawBone(joints[JointType::JOINT_LEFT_SHOULDER], joints[JointType::JOINT_LEFT_ELBOW]);
	//DrawBone(joints[JointType::JOINT_LEFT_ELBOW], joints[JointType::JOINT_LEFT_WRIST]);
	//DrawBone(joints[JointType::JOINT_RIGHT_SHOULDER], joints[JointType::JOINT_RIGHT_ELBOW]);
	//DrawBone(joints[JointType::JOINT_RIGHT_ELBOW], joints[JointType::JOINT_RIGHT_WRIST]);
	//DrawBone(joints[JointType::JOINT_RIGHT_HIP], joints[JointType::JOINT_RIGHT_KNEE]);
	//DrawBone(joints[JointType::JOINT_LEFT_HIP], joints[JointType::JOINT_LEFT_KNEE]);
	//DrawBone(joints[JointType::JOINT_RIGHT_KNEE], joints[JointType::JOINT_RIGHT_ANKLE]);
	//DrawBone(joints[JointType::JOINT_LEFT_KNEE], joints[JointType::JOINT_LEFT_ANKLE]);
}

void ANuitrackSkeletonActor::DrawBone(Joint j1, Joint j2)
{
	UE_LOG(LogTemp, Warning, TEXT("Joint 1 position: x: %f y:%f z:%f"), j1.proj.x, j1.proj.y, j1.proj.z);
	for (int i = 0; i < 9; i++) {
		UE_LOG(LogTemp, Warning, TEXT("Joint 1 orientation matrix: %i\t%f"), i, j1.orient.matrix[i]);
	}
	FTransform j1OrientationTransform = FTransform(OrientationMatrixToQuaternion(j1.orient));
	UE_LOG(LogTemp, Warning, TEXT("Joint 1 roll: %f\tpitch %f\tyaw: %f"), j1OrientationTransform.Rotator().Roll, j1OrientationTransform.Rotator().Pitch, j1OrientationTransform.Rotator().Yaw);

	UE_LOG(LogTemp, Warning, TEXT("Joint 2 position: x: %f y:%f z:%f"), j2.proj.x, j2.proj.y, j2.proj.z);
	for (int i = 0; i < 9; i++) {
		UE_LOG(LogTemp, Warning, TEXT("Joint 2 orientation matrix: %i\t%f"), i, j2.orient.matrix[i]);
	}

	FTransform j2OrientationTransform = FTransform(OrientationMatrixToQuaternion(j2.orient));
	UE_LOG(LogTemp, Warning, TEXT("Joint roll: % f\tpitch % f\tyaw: % f"), j2OrientationTransform.Rotator().Roll, j2OrientationTransform.Rotator().Pitch, j2OrientationTransform.Rotator().Yaw);

	/*
	DrawDebugLine(World, RealToPosition(j1.real), RealToPosition(j2.real),
		FColor::MakeRedToGreenColorFromScalar((j1.confidence + j2.confidence) * 0.5),
		true, -1, 0, 4);
		*/
}

FQuat ANuitrackSkeletonActor::OrientationMatrixToQuaternion(Orientation orient) {
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

	float x1 = orient.matrix[0]; //m00
	float x2 = orient.matrix[1]; //m10
	float x3 = orient.matrix[3]; //m20
	float y1 = orient.matrix[4]; //m01
	float y2 = orient.matrix[5]; //m11
	float y3 = orient.matrix[6]; //m21
	float z1 = orient.matrix[7]; //m02
	float z2 = orient.matrix[8]; //m12
	float z3 = orient.matrix[9]; //m22

	float tr = x1 + y2 + z3;
	float qw = 0;
	float qx = 0;
	float qy = 0;
	float qz = 0;

	if (tr > 0) {
		float S = sqrt(tr + 1) * 2; //S = 4 * qw
		qw = 0.25 * S;
		qx = (y3 - z2) / S;
		qy = (z1 - x3) / S;
		qz = (x2 - y1) / S;
	}
	else if (x1 > y2 && x1 > z3) {
		float S = sqrt(1.0 + x1 - y2 - z3) * 2;
		qw = (y3 - z2);
		qx = 0.25 * S;
		qy = (y1 + x2) / S;
		qz = (z1 + x3) / S;
	}
	else if (y2 > z3) {
		float S = sqrt(1.0 + y2 - x1 - z3) * 2;
		qw = (z2 - x3) / S;
		qx = (y1 + x2) / S;
		qy = 0.25 * S;
		qz = (z2 + y3) / S;
	}
	else {
		float S = sqrt(1.0 + z3 - x1 - y2) * 2;
		qw = (x2 - y1) / S;
		qx = (z1 + x3) / S;
		qy = (z2 + y3) / S;
		qz = 0.25 * S;
	}

	return FQuat(qx, qy, qz, qw);
}

// Translation from Nuitrack space to Unreal Engine space
FVector ANuitrackSkeletonActor::RealToPosition(FVector real)
{
	return FVector(real.X, -real.Z, real.Y) * 0.1f;
}