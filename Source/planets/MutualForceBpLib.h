// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MutualForceBpLib.generated.h"

/**
 * 
 * 
 */
UCLASS()
class PLANETS_API UMutualForceBpLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
        UFUNCTION(BlueprintCallable, Category = "AaMy Nodes")
        static void AddForceFunction(UStaticMeshComponent* obj, FVector vect);

        static void MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2);
};

/*
UCLASS()
class EDIFY_API UMyNodes : public UObject
{

    GENERATED_UCLASS_BODY()

};*/

// #include "Quadtree.h"
// //#include "../../../Box2D/Box2D/Common/b2BlockAllocator.h"
// 
// class b2MutualForceController /*: public b2ControllerDef*/
// {
// public:
// 	void AddGroup(b2ParticleSystem* particleSystem);
// 	void Step(/*const b2TimeStep& step*/);
// 	b2MutualForceController();
// 	~b2MutualForceController();
// 
// private:
// 	b2ParticleGroup* m_group;
// 	int m_stepCount;
// 	Tree* m_tree;
// 	b2ParticleSystem* m_particleSystem;
// 	float m_radius;
// };