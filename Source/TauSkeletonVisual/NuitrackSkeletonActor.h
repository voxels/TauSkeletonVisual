// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NuitrackSkeletonJointBuffer.h"

#include <iostream>
#include <vector>

#include "nuitrack/Nuitrack.h"
#include "nuitrack/modules/SkeletonTracker.h"
#include "nuitrack/types/Skeleton.h"
#include "nuitrack/types/SkeletonData.h"

#include "NuitrackSkeletonActor.generated.h"

using tdv::nuitrack::Skeleton;
using tdv::nuitrack::SkeletonTracker;
using tdv::nuitrack::SkeletonData;
using tdv::nuitrack::Joint;
using tdv::nuitrack::Orientation;
using tdv::nuitrack::Vector3;


UCLASS()
class TAUSKELETONVISUAL_API ANuitrackSkeletonActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ANuitrackSkeletonActor();
	~ANuitrackSkeletonActor();

	SkeletonTracker::Ptr skeletonTracker;

	int AssignedId;
	float LastDeltaTime;
	bool ReadyForUpdate;
	bool DidInitNuitrack;


	void OnSkeletonUpdate(SkeletonData::Ptr userSkeletons);
	void DrawSkeleton(int skeleton_index, std::vector<Joint> joints);
	void DrawBone(Joint j1, Joint j2);
	void UpdateJointBuffer(std::vector<Joint> joints);

	static FQuat OrientationMatrixToQuaternion(Orientation orient);
	static FVector RealToPosition(FVector real);

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	UNuitrackSkeletonJointBuffer* JointBuffer;

	UFUNCTION(BlueprintImplementableEvent, Category = "NuitrackSkeletonJointBuffer")
		void SkeletonJointBufferDidUpdate();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;	

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
