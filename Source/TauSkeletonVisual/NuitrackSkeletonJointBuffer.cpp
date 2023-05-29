// Fill out your copyright notice in the Description page of Project Settings.


#include "NuitrackSkeletonJointBuffer.h"

#include "Camera/CameraComponent.h"
#include "Engine/SceneCapture2D.h"
#include "Components/SceneCaptureComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "UObject/ScriptMacros.h"
#include "Engine/Texture2D.h"
#include "AssetRegistry/AssetRegistryModule.h"


#include "GameFramework/Character.h"
#include "Logging/LogMacros.h"
#include "UObject/Object.h"
#include "TauBuffer.h"
#include <Runtime/RenderCore/Public/RenderGraphBuilder.h>
#include "UObject/UObjectGlobals.h"
#include "Math/Vector.h"
#include "DynamicTexture.h"
#include "Kismet/KismetMathLibrary.h"

// Classes below from circumcenter.cpp in MeshKit   https://bitbucket.org/fathomteam/meshkit.git
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <chrono>
#include <sstream>
#include <iostream>

using namespace std;
using namespace std::chrono;

typedef struct RgbColor
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
} RgbColor;

typedef struct HsvColor
{
	unsigned char h;
	unsigned char s;
	unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv)
{
	RgbColor rgb;
	unsigned char region, remainder, p, q, t;

	if (hsv.s == 0)
	{
		rgb.r = hsv.v;
		rgb.g = hsv.v;
		rgb.b = hsv.v;
		return rgb;
	}

	region = hsv.h / 43;
	remainder = (hsv.h - (region * 43)) * 6;

	p = (hsv.v * (255 - hsv.s)) >> 8;
	q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
	t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
	case 0:
		rgb.r = hsv.v; rgb.g = t; rgb.b = p;
		break;
	case 1:
		rgb.r = q; rgb.g = hsv.v; rgb.b = p;
		break;
	case 2:
		rgb.r = p; rgb.g = hsv.v; rgb.b = t;
		break;
	case 3:
		rgb.r = p; rgb.g = q; rgb.b = hsv.v;
		break;
	case 4:
		rgb.r = t; rgb.g = p; rgb.b = hsv.v;
		break;
	default:
		rgb.r = hsv.v; rgb.g = p; rgb.b = q;
		break;
	}

	return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
	HsvColor hsv;
	unsigned char rgbMin, rgbMax;

	rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
	rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

	hsv.v = rgbMax;
	if (hsv.v == 0)
	{
		hsv.h = 0;
		hsv.s = 0;
		return hsv;
	}

	hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
	if (hsv.s == 0)
	{
		hsv.h = 0;
		return hsv;
	}

	if (rgbMax == rgb.r)
		hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
	else if (rgbMax == rgb.g)
		hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
	else
		hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

	return hsv;
}

// Sets default values
UNuitrackSkeletonJointBuffer::UNuitrackSkeletonJointBuffer()
{
	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryComponentTick.bCanEverTick = true;
}

UNuitrackSkeletonJointBuffer::~UNuitrackSkeletonJointBuffer()
{

}

// Called when the game starts or when spawned
void UNuitrackSkeletonJointBuffer::BeginPlay()
{
	Super::BeginPlay();

	MinDebugTriangleIndex = 0;
	MaxDebugTriangleIndex = 67 * 3;

	AngleTauLayer0Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer0Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer2Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer2Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer11Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer11Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer12Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer12Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer13Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer13Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer14Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer14Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer15Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer15Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauLayer16Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauLayer16Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer0Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer0Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer2Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer2Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer11Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer11Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer12Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer12Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer13Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer13Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer14Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer14Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer15Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer15Texture->Initialize(18, 18, FLinearColor::Black);

	AngleTauDotLayer16Texture = NewObject<UDynamicTexture>(GetOuter());
	AngleTauDotLayer16Texture->Initialize(18, 18, FLinearColor::Black);


	PositionTauLayer0Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer0Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer2Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer2Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer11Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer11Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer12Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer12Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer13Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer13Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer14Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer14Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer15Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer15Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauLayer16Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauLayer16Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer0Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer0Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer2Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer2Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer11Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer11Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer12Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer12Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer13Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer13Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer14Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer14Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer15Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer15Texture->Initialize(18, 18, FLinearColor::Black);

	PositionTauDotLayer16Texture = NewObject<UDynamicTexture>(GetOuter());
	PositionTauDotLayer16Texture->Initialize(18, 18, FLinearColor::Black);
}

void UNuitrackSkeletonJointBuffer::EndPlay(const EEndPlayReason::Type EndPlayReason) {
	Super::EndPlay(EndPlayReason);
	if (CurrentRunningThread && CalcThread) {
		CurrentRunningThread->Suspend(true);
		CalcThread->bStopThread = true;
		CurrentRunningThread->Suspend(false);
		CurrentRunningThread->Kill(false);
		CurrentRunningThread->WaitForCompletion();
		delete CalcThread;
	}
}

// Called every frame
void UNuitrackSkeletonJointBuffer::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}


void UNuitrackSkeletonJointBuffer::ProcessSocketRawData(float DeltaTime)
{
	if (!TriangleIndexesQueue.IsEmpty()) {
		TriangleIndexesQueue.Dequeue(TriangleIndexes);
	}
	if (!TrianglePositionsQueue.IsEmpty()) {
		TrianglePositionsQueue.Dequeue(TrianglePositions);
	}
	if (!TriangleIndexBoneNamesQueue.IsEmpty()) {
		TriangleIndexBoneNamesQueue.Dequeue(TriangleIndexBoneNames);
	}
	if (!TriangleRotationsQueue.IsEmpty()) {
		TriangleRotationsQueue.Dequeue(TriangleRotations);
	}
	if (!TriangleCentroidsQueue.IsEmpty()) {
		TriangleCentroidsQueue.Dequeue(TriangleCentroids);
	}
	if (!TriangleCircumcentersQueue.IsEmpty()) {
		TriangleCircumcentersQueue.Dequeue(TriangleCircumcenters);
	}
	if (!EulerLinesQueue.IsEmpty()) {
		EulerLinesQueue.Dequeue(EulerLines);
	}
	if (!TriangleTauBuffersQueue.IsEmpty()) {
		TriangleTauBuffersQueue.Dequeue(TriangleTauBuffers);
		
		IncrementalAngleTauSamples.Empty();
		IncrementalPositionTauSamples.Empty();
		IncrementalAngleTauDotSamples.Empty();
		IncrementalPositionTauDotSamples.Empty();

		std::vector<UTauBuffer*>::iterator it;
		for (it = TriangleTauBuffers.begin(); it != TriangleTauBuffers.end(); it++) {
			UTauBuffer* Buffer = *it;
			if (Buffer->IncrementalAngleTauSamples.Num() > 0) {
				IncrementalAngleTauSamples.Emplace(Buffer->IncrementalAngleTauSamples.Last());
			}
			if (Buffer->IncrementalAngleTauDotSamples.Num() > 0) {
				IncrementalAngleTauDotSamples.Emplace(Buffer->IncrementalAngleTauDotSamples.Last());
			}

			if (Buffer->IncrementalPositionTauSamples.Num() > 0) {
				IncrementalPositionTauSamples.Emplace(Buffer->IncrementalPositionTauSamples.Last());
			}

			if (Buffer->IncrementalPositionTauDotSamples.Num() > 0) {
				IncrementalPositionTauDotSamples.Emplace(Buffer->IncrementalPositionTauDotSamples.Last());
			}
		}

		//UE_LOG(LogTemp, Display, TEXT("Found IncrementalPositionTauDotSamples size %i"), IncrementalPositionTauDotSamples.Num());

	}
	else {
		UE_LOG(LogTemp, Display, TEXT("TriangleTauBufferQueue is empty"));
	}
}

void UNuitrackSkeletonJointBuffer::UpdateSocketRawData(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations, TArray<float>Confidences)
{
	if (BoneNames.Num() != Locations.Num() || BoneNames.Num() != Rotations.Num()) {
		UE_LOG(LogTemp, Warning, TEXT("Not Updating socket data because data is not aligned."));
		return;
	}

	SocketBoneNames.Empty();
	SocketLocations.Empty();
	SocketRotations.Empty();
	SocketConfidences.Empty();

	for (int ii = 0; ii < BoneNames.Num(); ii++) {
		FName BoneName = BoneNames[ii];
		FRotator Rotation = Rotations[ii] /* MeshComponent->GetSocketRotation(Name)*/;
		FVector Location = Locations[ii] /* MeshComponent->GetSocketLocation(Name)*/;
		float Confidence = Confidences[ii];

		SocketNames.Emplace(BoneName);
		SocketBoneNames.Emplace(BoneName);
		SocketRotations.Emplace(Rotation);
		SocketLocations.Emplace(Location);
		SocketConfidences.Emplace(Confidence);
	}
}


void UNuitrackSkeletonJointBuffer::UpdateTrackingRenderTargets() {
	if (TriangleTauBuffers.size() == 0) {
		return;
	}

	TArray<float> LastAngleTauSamples;
	TArray<float> LastAngleTauDotSamples;
	TArray<float> LastPositionTauSamples;
	TArray<float> LastPositionTauDotSamples;
	std::vector<UTauBuffer*>::iterator it;
	for (it = TriangleTauBuffers.begin(); it != TriangleTauBuffers.end(); it++) {
		UTauBuffer* TauBuffer = *it;
		//UE_LOG(LogTemp, Display, TEXT("\n%s"), *TauBuffer->GetFName().ToString());
		if (TauBuffer->IncrementalAngleTauSamples.Num() > 0) {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Angle Tau Sample:\t%f"), TauBuffer->IncrementalAngleTauSamples.Last());
			LastAngleTauSamples.Add(TauBuffer->IncrementalAngleTauSamples.Last());
		}
		if (TauBuffer->IncrementalAngleTauDotSamples.Num() > 0) {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Angle Tau Dot Sample:\t%f"), TauBuffer->IncrementalAngleTauDotSamples.Last());
			LastAngleTauDotSamples.Add(TauBuffer->IncrementalAngleTauDotSamples.Last());
		}

		if (TauBuffer->IncrementalPositionTauSamples.Num() > 0) {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau Sample:\t%f"), TauBuffer->IncrementalPositionTauSamples.Last());
			LastPositionTauSamples.Add(TauBuffer->IncrementalPositionTauSamples.Last());
		}
		if (TauBuffer->IncrementalPositionTauDotSamples.Num() > 0) {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau Dot Sample:\t%f\n"), TauBuffer->IncrementalPositionTauDotSamples.Last());
			LastPositionTauDotSamples.Add(TauBuffer->IncrementalPositionTauDotSamples.Last());
		}
	}

	TArray<float> SortedAngleTauSamples = TArray<float>(LastAngleTauSamples);
	TArray<float> SortedAngleTauDotSamples = TArray<float>(LastAngleTauDotSamples);
	TArray<float> SortedPositionTauSamples = TArray<float>(LastPositionTauSamples);
	TArray<float> SortedPositionTauDotSamples = TArray<float>(LastPositionTauDotSamples);

	SortedAngleTauSamples.Sort();
	SortedAngleTauDotSamples.Sort();
	SortedPositionTauSamples.Sort();
	SortedPositionTauDotSamples.Sort();

	int CompleteSamples = TriangleIndexes.Num() / 3;

	if (SortedAngleTauSamples.Num() == CompleteSamples && SortedAngleTauDotSamples.Num() == CompleteSamples && SortedPositionTauSamples.Num() == CompleteSamples && SortedPositionTauDotSamples.Num() == CompleteSamples) {

		TArray<FColor> AngleTauFillColors = CreateFillColors(TriangleIndexes, LastAngleTauSamples, -1, 1);
		TArray<FColor> AngleTauDotFillColors = CreateFillColors(TriangleIndexes, LastAngleTauDotSamples, -1 ,1);
		TArray<FColor> PositionTauFillColors = CreateFillColors(TriangleIndexes, LastPositionTauSamples, -1 ,1);
		TArray<FColor> PositionTauDotFillColors = CreateFillColors(TriangleIndexes, LastPositionTauDotSamples, -1 ,1);

		int CompleteArray = 18 * 18 * 18;

		if (AngleTauFillColors.Num() == CompleteArray && AngleTauDotFillColors.Num() == CompleteArray && PositionTauFillColors.Num() == CompleteArray && PositionTauDotFillColors.Num() == CompleteArray) {
			for (int32 Depth = 0; Depth <= 17; Depth++) {
				if (Depth == 1) {
					CreateTextureSliceWithColors(AngleTauLayer0Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer0Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer0Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer0Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 3) {
					CreateTextureSliceWithColors(AngleTauLayer2Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer2Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer2Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer2Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 12) {
					CreateTextureSliceWithColors(AngleTauLayer11Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer11Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer11Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer11Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 13) {
					CreateTextureSliceWithColors(AngleTauLayer12Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer12Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer12Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer12Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 14) {
					CreateTextureSliceWithColors(AngleTauLayer13Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer13Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer13Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer13Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 15) {
					CreateTextureSliceWithColors(AngleTauLayer14Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer14Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer14Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer14Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 16) {
					CreateTextureSliceWithColors(AngleTauLayer15Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer15Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer15Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer15Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else if (Depth == 17) {
					CreateTextureSliceWithColors(AngleTauLayer16Texture, 18, 18, Depth, AngleTauFillColors);
					CreateTextureSliceWithColors(AngleTauDotLayer16Texture, 18, 18, Depth, AngleTauDotFillColors);
					CreateTextureSliceWithColors(PositionTauLayer16Texture, 18, 18, Depth, PositionTauFillColors);
					CreateTextureSliceWithColors(PositionTauDotLayer16Texture, 18, 18, Depth, PositionTauDotFillColors);
				}
				else {
					continue;
				}
			}
			//UE_LOG(LogTemp, Display, TEXT("Added %i textures to array"), AngleTauTexture2DArray->SourceTextures.Num());
		}
	}
	else {
		//UE_LOG(LogTemp, Display, TEXT("Incremental Angle Tau Sample:\t%f"), TauBuffer->IncrementalAngleTauSamples.Last());
	     UE_LOG(LogTemp, Display, TEXT("Found unequal samples (%i) and triangles (%i)"), LastAngleTauSamples.Num(), TriangleIndexes.Num() / 3);
	}
}

TArray<FColor> UNuitrackSkeletonJointBuffer::CreateFillColors(TArray<int> JointIndexes, TArray<float>FrameSamples, float ClampMin, float ClampMax )
{
	TArray<FColor> RetVal;

	if (JointIndexes.Num() / 3 != FrameSamples.Num()) {
		UE_LOG(LogTemp, Display, TEXT("Found unequal samples (%i) and triangles (%i)"), JointIndexes.Num(), FrameSamples.Num() / 3);
		return RetVal;
	}

	RetVal.SetNum(18 * 18 * 18);
	for (int ii = 0; ii < 18 * 18 * 18; ii++) {
		RetVal[ii] = FColor::Transparent;
	}

	for (int ii = 0; ii < FrameSamples.Num(); ii++) {
		int x = JointIndexes[ii * 3 + 1];		// Triangle p1 as X
		int y = JointIndexes[ii * 3 + 2];		// Triangle p2 as Y
		int z = JointIndexes[ii * 3 + 0];		// Using Z Value as triangle joint 0
		int index = z + y * 18 + x * 18 * 18;

		if( index < RetVal.Num() ){
			//UE_LOG(LogTemp, Display, TEXT("Converting x:%i y:%i z:%i to index: %i"),x, y, z, index);
			
			if( FrameSamples[ii] < 0 ){
				//UE_LOG(LogTemp, Display, TEXT("Setting Negative Value Color: %f"), Value);
				float Value = Map(FrameSamples[ii], ClampMin, 0, 0, 255);
				if( Value > 255 ){
					Value = 255;
				}
				else if (Value < 0) {
					Value = 0;
				}
				RetVal[index] = FColor(0, uint8(Value), 255 - uint8(Value), 255);
			}
			else {
				//UE_LOG(LogTemp, Display, TEXT("Setting Positive Value Color: %f"), Value);
				float Value = Map(FrameSamples[ii], 0, ClampMax, 0, 255);
				if (Value > 255) {
					Value = 255;
				}
				else if (Value < 0) {
					Value = 0;
				}
				RetVal[index] = FColor(uint8(Value), 0, 255 - uint8(Value), 255);
			}
		}
		else {
			UE_LOG(LogTemp, Display, TEXT("Found index outsize of range x:%i y:%i z:%i to index: %i"), x, y, z, index);
		}
	}
	
	return RetVal;
}

float UNuitrackSkeletonJointBuffer::Map(float value,
	float istart,
	float istop,
	float ostart,
	float ostop) {
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}


void UNuitrackSkeletonJointBuffer::CreateTextureSliceWithColors(UDynamicTexture* BufferTexture, int32 ALPHA_MAP_WIDTH, int32 ALPHA_MAP_HEIGHT, int32 DEPTH_INDEX, TArray<FColor> FillColors)
{
	BufferTexture->Clear();

	if (!BufferTexture->bDidInitialize || BufferTexture->GetWidth() == 0) {
		UE_LOG(LogTemp, Warning, TEXT("Buffer Texture did not initialize: %i"), BufferTexture->bDidInitialize);
		return;
	}

	for (int32 Y = 0; Y < ALPHA_MAP_HEIGHT; Y++)
	{
		for (int32 X = 0; X < ALPHA_MAP_WIDTH; X++)
		{
			int index = DEPTH_INDEX + ALPHA_MAP_HEIGHT * Y + ALPHA_MAP_HEIGHT * ALPHA_MAP_WIDTH * X;
			FColor Color = FillColors[index];
			BufferTexture->SetPixel(X, Y, Color.ReinterpretAsLinear());

			//if (Y == 0 && (X == 0 || X == 1) && false) {
//				if (Color.R > 0 || Color.B > 0 || Color.G > 0) {
					/*
					UE_LOG(LogTemp, Warning, TEXT("Buffer Texture Index: %i, X: %i, Y:%i"), index, X, Y);
					UE_LOG(LogTemp, Warning, TEXT("Red: %i"), static_cast<int>(Color.ReinterpretAsLinear().R * 255));
					UE_LOG(LogTemp, Warning, TEXT("Green, %i"), static_cast<int>(Color.ReinterpretAsLinear().G * 255));
					UE_LOG(LogTemp, Warning, TEXT("Blue: %i"), static_cast<int>(Color.ReinterpretAsLinear().B * 255));
					UE_LOG(LogTemp, Warning, TEXT("Alpha: %i"), static_cast<int>(Color.ReinterpretAsLinear().A * 255));
				*/
//				}
			//}
		}
	}
	BufferTexture->UpdateTexture();
}

void UNuitrackSkeletonJointBuffer::InitCalculations(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations, TArray<float>Confidences, std::vector<UTauBuffer*>PreviousTriangleTauBuffers) {
	CalcThread = new FJointBufferThread(BoneNames, Locations, Rotations, Confidences, PreviousTriangleTauBuffers, this);
	CurrentRunningThread = FRunnableThread::Create(CalcThread, TEXT("CalculationThread"));
}