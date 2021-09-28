// Fill out your copyright notice in the Description page of Project Settings.

#include "MutualForceBpLib.h"


UMutualForceBpLib::~UMutualForceBpLib()
{
	//Remove attached bodies
    printf("in cpp this UMutualForceBpLib destructor \n");
	//Clear();
    //printf("start in cpp this UMutualForceBpLib constructor \n");
}

UMutualForceBpLib::UMutualForceBpLib()
{
	// constructor
	m_stepCount = 0;
	m_tree = new Tree;
	m_particleSystem = NULL;
	m_radius = 0.025f;  // needs to be taken from the objects
}

void UMutualForceBpLib::AddForceFunction(UStaticMeshComponent* obj, FVector Force) {

	// add force to body
	obj->AddForce(Force, "None");
}


float UMutualForceBpLib::MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2) {

	// calc force 
	float G = 10000;


	float r = 700;

    float mass1 = obj1->CalculateMass();
    float mass2 = obj2->CalculateMass();

    FVector pos1 = obj1->GetCenterOfMass();
    FVector pos2 = obj2->GetCenterOfMass();

    FVector direction = pos2 - pos1;

    float fForce = G * mass1 * mass2 / (r * r);

	FVector Force = direction * fForce;

	obj1->AddForce(Force, "None");
	obj2->AddForce(-Force, "None");

    return fForce;

}


void UMutualForceBpLib::Step(/*const b2TimeStep& step*/)
{
	//gravitational constant bigG
	// (TODO:) this should be accassible in from UE 
	float fltOurG = 0.00002f;

    // int32 iParticleCount =  m_particleSystem->GetParticleCount();	
	FVector* paHeadPos = new FVector(0); // = m_particleSystem->GetPositionBuffer(); TODO:
	// b2ParticleColor* paColor = m_particleSystem->GetColorBuffer();

    // UPrimitiveComponent::GetMass();
    //#include "Components/PrimitiveComponent.h"

    FVector vPos;
    FVector vDistance;
    FVector vForce;
    float fltForce;
    float fltDistance;

    FVector CoM;
    float mass;

    // once every steps
    if (this->m_stepCount % 1 == 0)
    {	
        m_tree->cleanup(); 
        m_tree->setup(paHeadPos, m_particleCount);
        //printf(" ################## add to tree ");
        for (int32 idx = 0; idx < m_particleCount; idx++)
        {
            //printf("%d ",idx);
            
            // printf("now particle add %"PRIx64"\n", (void*)&paHeadPos[idx]);
            //m_tree->add(&paHeadPos[idx], m_particleSystem->GetParticleHandleFromIndex(idx));
            m_tree->add(&paHeadPos[idx], &m_particleSystem[idx] );
        }
        //printf("\n\n\n\n");
        // m_tree->draw(m_world);
    }

    for (int32 idx = 0; idx < m_particleCount; idx++)
    {
        // particle of interest
        vPos = paHeadPos[idx];
        //paColor[idx].Set(255,255,255,128);
        vForce = FVector(0.0f, 0.0f, 0.0f);
        float k = 0.5f;

        //printf("\n %d with \n",idx);

        int iterations = 0;
        // float seenmass = 0;  // debug help
        for (Tree* it = m_tree->iterator()->Next(k,&vPos);
            it != NULL;
            it = it->Next(k,&vPos))
        {
            //printf("    d:%d X:%f Y:%f K:%f \n",it->GetDepth() ,it->midX, it->midY, it->GetK(&vPos));
            

            // highlight all nodes of first particle
            /*if (idx == 0)
            {
                char szAncestors[200] = {0};
                it->drawonlythis(m_world);

                printf("idx %d iteration %d treeid %d %s\n",idx,iterations,  (int)it , it->getAncestors(szAncestors));
            }*/

            iterations++;

            mass = it->Mass;
            //seenmass +=mass; // debug help
            CoM = it->getCenterOfMass();

            vDistance = CoM - vPos;

            fltDistance = vDistance.Dist(CoM,vPos);//  ;vDistance.Dist();
            
            if ( fltDistance < m_radius/2)
            {

                // this is too cloose to exist must be touching forces or self
                //printf(" ### skipping distance %f idx %d iteration %d treeid %d\n",fltDistance,idx,iterations,  (int)it );
                continue;
            }

            vDistance.Normalize();

            fltForce = fltOurG * mass / (fltDistance * fltDistance) ; //b2Dot(vDistance,vDistance)
            vForce += vDistance * fltForce;	
        }

        //printf("idx %d seenmass %f\n",idx, seenmass);

        /*printf("  %f Force \n",vForce.Length());*/

        /*float force = vForce.Length();
        if (force > 0.1)
        {
              // color high force
              paColor[idx].Set(200,10,10,250);
        }
        else 
        {
            paColor[idx].Set( (uint8)(255*(1-force)), (uint8)(255*(1-force)), (uint8)(255*(1-force)),(uint8)(255*(1-force)));
        }*/

        // aply sum of all forces to particle
        // 
        // 
        // TODO !!!  m_particleSystem->ParticleApplyForce( idx, vForce);	
        // 
        // 
        //printf("iterations %d\n\n",iterations);
    }
}

 /* particleSystem is a StaticMesh Array */
void UMutualForceBpLib::AddGroup(UStaticMeshComponent* particleSystem, int iParticleCount)
{
    this->m_particleSystem = particleSystem;
    this->m_particleCount = iParticleCount;
}
