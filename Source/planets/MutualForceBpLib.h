// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Quadtree.h"
#include "MutualForceBpLib.generated.h"


/**
 * 
 * 
 */
UCLASS()
class PLANETS_API UMutualForceBpLib : public UBlueprintFunctionLibrary
{
    // public:

    float m_G = 1000;
    /*void setBigG(float fG) {
        this->G = fG;
    }*/
	GENERATED_BODY()
        UFUNCTION(BlueprintCallable, Category = "AaMy Nodes")
        void AddForceFunction(UStaticMeshComponent* obj, FVector vect);
        
    // GENERATED_BODY()
        UFUNCTION(BlueprintCallable, Category = "AaMy Nodes")
 	    void AddGroup(UStaticMeshComponent* particleSystem, int iParticleCount);
    // GENERATED_BODY()
 	    void Step(/*const b2TimeStep& step*/);
 	UMutualForceBpLib();
 	~UMutualForceBpLib();


        UFUNCTION(BlueprintCallable, Category = "AaMy Nodes")
        static float MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2, float G);




 //private:
    int m_particleCount;
 	int m_stepCount;
 	Tree* m_tree;
 	UStaticMeshComponent* m_particleSystem;
 	float m_radius;

};

