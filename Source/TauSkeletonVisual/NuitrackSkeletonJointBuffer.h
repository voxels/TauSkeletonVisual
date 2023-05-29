// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <vector>
#include "Math/Vector.h"
#include "Math/Rotator.h"
#include "GameFramework/Character.h"
#include "TauBuffer.h"
#include "Containers/CircularBuffer.h"
#include "Components/ActorComponent.h"
#include "Engine/Texture2DArray.h"
#include "Math/Color.h"
#include "Engine/Texture2D.h"
#include "JointBufferThread.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "NuitrackSkeletonJointBuffer.generated.h"



UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class TAUSKELETONVISUAL_API UNuitrackSkeletonJointBuffer : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UNuitrackSkeletonJointBuffer();
	~UNuitrackSkeletonJointBuffer();

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FName> SocketNames;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FName> SocketBoneNames;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FVector> SocketLocations;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FRotator> SocketRotations;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<float> SocketConfidences;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<int> TriangleIndexes;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FVector> TrianglePositions;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FName> TriangleIndexBoneNames;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FRotator> TriangleRotations;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FVector> TriangleCentroids;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FVector> TriangleCircumcenters;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<FVector> EulerLines;

	std::vector<UTauBuffer*> TriangleTauBuffers;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<float> IncrementalAngleTauSamples;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<float> IncrementalPositionTauSamples;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<float> IncrementalAngleTauDotSamples;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
	TArray<float> IncrementalPositionTauDotSamples;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		int MinDebugTriangleIndex;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		int MaxDebugTriangleIndex;


	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateSocketRawData(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations, TArray<float>Confidences);

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void ProcessSocketRawData(float DeltaTime);
	
	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void CreateTextureSliceWithColors(UDynamicTexture* BufferTexture, int32 ALPHA_MAP_WIDTH, int32 ALPHA_MAP_HEIGHT, int32 DEPTH_INDEX, TArray<FColor> FillColors);

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer0Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer2Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer11Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer12Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer13Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer14Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer15Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauLayer16Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer0Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer2Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer11Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer12Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer13Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer14Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer15Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* AngleTauDotLayer16Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer0Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer2Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer11Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer12Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer13Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer14Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer15Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauLayer16Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer0Texture;

	UPROPERTY(BlueprintReadWrite, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer2Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer11Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer12Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer13Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer14Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer15Texture;

	UPROPERTY(BlueprintReadOnly, Category = "NuitrackSkeletonJointBuffer")
		UDynamicTexture* PositionTauDotLayer16Texture;

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		TArray<FColor> CreateFillColors(TArray<int> JointIndexes, TArray<float>FrameSamples, float ClampMin, float ClampMax);

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateTrackingRenderTargets();


	class FJointBufferThread* CalcThread = nullptr;
	FRunnableThread* CurrentRunningThread = nullptr;

	TQueue<TArray<int>> TriangleIndexesQueue;
	TQueue<TArray<FVector>> TrianglePositionsQueue;
	TQueue<TArray<FName>> TriangleIndexBoneNamesQueue;
	TQueue<TArray<FRotator>> TriangleRotationsQueue;
	TQueue<TArray<FVector>> TriangleCentroidsQueue;
	TQueue<TArray<FVector>> TriangleCircumcentersQueue;
	TQueue<TArray<FVector>> EulerLinesQueue;
	TQueue<std::vector<UTauBuffer*>> TriangleTauBuffersQueue;

		void InitCalculations(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations, TArray<float>Confidences, std::vector<UTauBuffer*>PreviousTriangleTauBuffers);

protected:
	// Called when the game starts
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	/*
	TArray<FVector> AB;
	TArray<FVector> ABmid;
	TArray<FVector> BC;
	TArray<FVector> BCmid;
	TArray<FVector> CA;
	TArray<FVector> CAmid;
	TArray<FVector> V;
	TArray<FVector> D1;
	TArray<FVector> D2;
	TArray<FVector> D3;
	TArray<FVector> ABBC;
	
	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateTriangles(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations);

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateDebugLines();

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateEulerLines();

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
		void UpdateTracking();

		*/

	UFUNCTION(BlueprintCallable, Category = "NuitrackSkeletonJointBuffer")
	float Map(float value,
		float istart,
		float istop,
		float ostart,
		float ostop);
public:	
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;


};
