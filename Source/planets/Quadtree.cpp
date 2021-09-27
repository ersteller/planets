
#include "Quadtree.h"

//#define max(x,y) x>=y?x:y

/* this should be usable like:
adding particles 
use iterator(curpos, qoutient) of tree and let tree decide which node you get

*/

/* 
stolen from https://code.google.com/p/kyle/
	addons and demos for openframework
 maxParticles describes how many particles are allowed
 in each leaf. a normal quadtree only has 1 particle per
 leaf. this can cause issues if two particles are very close
 together, as it causes a large number of subdivisions.
 in an evenly distributed system, if maxParticles has a high
 value, the tree will approximate a binning system.
*/


/*
an illustration of the north-east-top naming system 
this is not visible from outside
    ┌──────────────────────────► X
   /│                             
  / │           ┌───────┬───────┐
 /  │          /│       │       │
Z   │         / │   nwb │   neb │
    │        /  │       │       │
    │       /   ├───────┼───────┤
    │      /    │       │       │
    │     /     │   swb │   seb │
    │    /      │       │       │
  Y ▼   /       └───────┴───────┘
       /                       /
      ┌───────┬───────┐       /
      │       │       │      /
      │   nwt │   net │     /
      │       │       │    /
      ├───────┼───────┤   /
      │       │       │  /
      │   swt │   set │ /
      │       │       │/
      └───────┴───────┘

*/




/* TODO: define qoutient s/d (spread / distance) to conditionally use above*/

/* TODO: binning: find large bodies of connectect particles */
/* 1. make round static bodys to a bin */
/* 2. tidal force or impact should break bins (testcase: check roche border) d= 2.423 * R * (rohM/rohm)^(1/3)*/

	char* Tree::getAncestors(char* buff)
	{
		Tree* w = this;
		int d = GetDepth() * 4;
		memcpy(&buff[0], " roo", 4);
		while (w->parent != NULL)
		{
			if(w == w->parent->nwt)
				memcpy(&buff[d], "_nwt", 4);
			else if(w == w->parent->net)
				memcpy(&buff[d], "_net", 4);		
			else if(w == w->parent->swt)
				memcpy(&buff[d], "_swt", 4);
			else if(w == w->parent->set)
				memcpy(&buff[d], "_set", 4);
			else if(w == w->parent->nwb)
				memcpy(&buff[d], "_nwb", 4);
			else if(w == w->parent->neb)
				memcpy(&buff[d], "_neb", 4);
			else if(w == w->parent->swb)
				memcpy(&buff[d], "_swb", 4);
			else if(w == w->parent->seb)
				memcpy(&buff[d], "_seb", 4);
			//else 
				//assert(0);
			d-=4;
			//assert(d>=0);
			w = w->parent;
		}	 
		return buff;
	}

	// k error between 0 and 1 
	// if lager than 1 we are only evaluating root      (100% error)
	// if k == 0 we use direct method and evaluate O^n  (0% error)
	float Tree::GetK(FVector* cur)
	{
		// spread of box
		FVector direction;
		float radius = ((midY - minY) * 2);  // this is an aproximation for the width of the element
	    direction.X = midX - cur->X;
		direction.Y = midY - cur->Y;
		direction.Z = midZ - cur->Z;
		// this is an approximation for the distance between us and the diameter of the
		float length = abs(direction.X) + abs(direction.Y) + abs(direction.Z); // direction.Length(); // this takes 10%of all time direction.Length(); 

		// k is an indicator for expectet error impact 
		// sensable between 0 and 1 if lager than 1 we are very close to or inside this node
		float k = 0;
		if (length != 0)
			k = radius / length;
		else 
			k = std::numeric_limits<float>::max();		
		return k;
	}


	/// find next node after this one (sequence is nwt --> net --> swt --> set --> nwb --> neb --> swb --> seb -->  parent-parent)
	Tree* Tree::Next(float kerror, FVector* cur)
	{	
		Tree* ret = NULL;
		Tree* worker = this;

		/* use short circuit before calling GetK */
		if (worker->hasChildren && worker->GetK(cur) > kerror )
		{
			while (worker->hasChildren && worker->GetK(cur) > kerror)
			{
				// we should go deeper
				worker = worker->nwt;
			}
			// we are now at the correct level
			// skip border nodes
			if (worker->nSubParticles != 0 || worker->nParticles != 0)
				ret = worker;
			else 
				ret = worker->Next(kerror,cur);
			return ret;
		}


		while (worker->parent != NULL && worker == worker->parent->seb )
		{
			// at the end of children go up
			worker = worker->parent; 
			// and one more next to the side 
		}
				
		// sequence is nwt-->net-->swt-->set-->nwb-->neb-->swb-->seb-->parent-parent
		// get next of same layer 
		if (worker->parent != NULL)
		{
			if(worker == worker->parent->nwt)
				worker = worker->parent->net;
			else if(worker == worker->parent->net)
				worker = worker->parent->swt;			
			else if(worker == worker->parent->swt)
				worker = worker->parent->set;
			else if (worker == worker->parent->set)
				worker = worker->parent->neb;
			else if (worker == worker->parent->neb)
				worker = worker->parent->swb;
			else if (worker == worker->parent->swb)
				worker = worker->parent->seb;
			//else 
				//assert(0); // did not find itself in the children of parent
			

			while (worker->hasChildren && worker->GetK(cur) > kerror)
			{
				// we should go deeper
				worker = worker->nwt;
			}
			// we are now at the correct level
			// skip border nodes
			if (worker->nSubParticles != 0 || worker->nParticles != 0)
				ret = worker;
			else 
				ret = worker->Next(kerror,cur);
			return ret;


		}
		// we arrived in root node 
		else 
			return NULL;	
	}

	void Tree::UpdateCenterOfMass(FVector* cur, float CurMass)
	{
		float NewMass = Mass + CurMass;

		/* arithmetic middle incrementally  */
		CenterOfMass.X = CenterOfMass.X * Mass / NewMass + cur->X * CurMass / NewMass;
		CenterOfMass.Y = CenterOfMass.Y * Mass / NewMass + cur->Y * CurMass / NewMass;
		CenterOfMass.Z = CenterOfMass.Z * Mass / NewMass + cur->Z * CurMass / NewMass;
		Mass = NewMass;
	}

	Tree::HandleAndPos* Tree::add(FVector* cur, void* phP, Tree::HandleAndPos* ptHPin) {
		// TODO: maybe optimizable
		HandleAndPos* pRes = NULL;
		
		if(hasChildren) 
		{
			if(cur->X < midX) 
			{
				if(cur->Y < midY) 
				{
					if (cur->Z < midZ)
					{
						pRes = nwb->add(cur, phP, ptHPin);
					}
					else 
					{
						pRes = nwt->add(cur, phP, ptHPin);
					}
				} 
				else 
				{
					if (cur->Z < midZ)
					{
						pRes = swb->add(cur, phP, ptHPin);
					} else 
					{
						pRes = swt->add(cur, phP, ptHPin);
					}
				}
			} 
			else 
			{
				if(cur->Y < midY) 
				{
					if (cur->Z < midZ){
						pRes = neb->add(cur, phP, ptHPin);
					} 
					else 
					{
						pRes = net->add(cur, phP, ptHPin);
					}
				} 
				else 
				{
					if (cur->Z < midZ)
					{
						pRes = seb->add(cur, phP, ptHPin);
					} 
					else 
					{
						pRes = set->add(cur, phP, ptHPin);
					}
				}
			}
			// add the current particle to the subparticle list because we have added it to the child
			nSubParticles++;
			subparticles.push_back(pRes);
			
			/* update center of mass */
			if(ptHPin == NULL)
			{  // so this is a new/unknown particle (not moved to children from parent)
				float CurMass = 1; //TODO: mass of this particle may be individuel
				UpdateCenterOfMass(cur, CurMass);
			}
		} 
		else 
		{
			if(nParticles < maxParticles) {
				if (ptHPin == NULL)
				{
					ptHPin = new HandleAndPos;
					ptHPin->pos = cur;
					ptHPin->handle = phP;
				}
				pRes = ptHPin;
				particles.push_back(ptHPin); 
				nParticles++;

				float CurMass = 1; //TODO: mass of this particle may be individuel
				UpdateCenterOfMass(cur, CurMass);

			} else {

				if(nwt == NULL)
				{
					// float _minX, float _minY, float _minZ, float _maxX, float _maxY, float _maxZ, Tree* pparent
					nwt = new Tree(minX, midX, minY, midY, midZ, maxZ, this);
					net = new Tree(midX, maxX, minY, midY, midZ, maxZ, this);
					swt = new Tree(minX, midX, midY, maxY, midZ, maxZ, this);
					set = new Tree(midX, maxX, midY, maxY, midZ, maxZ, this);

					nwb = new Tree(minX, midX, minY, midY, minZ, midZ, this);
					neb = new Tree(midX, maxX, minY, midY, minZ, midZ, this);
					swb = new Tree(minX, midX, midY, maxY, minZ, midZ, this);
					seb = new Tree(midX, maxX, midY, maxY, minZ, midZ, this);
				}
				else
				{ 
					/* only update positions*/
					// top
					nwt->minX = minX;
					nwt->minY = minY;
					nwt->maxX = midX;
					nwt->maxY = midY;
					nwt->minZ = midZ;
					nwt->maxZ = maxZ;
					nwt->setMid();

					net->minX = midX;
					net->minY = minY;
					net->maxX = maxX;
					net->maxY = midY;
					net->minZ = midZ;
					net->maxZ = maxZ;
					net->setMid();

					swt->minX = minX;
					swt->minY = midY;
					swt->maxX = midX;
					swt->maxY = maxY;
					swt->minZ = midZ;
					swt->maxZ = maxZ;
					swt->setMid();

					set->minX = midX;
					set->minY = midY;
					set->maxX = maxX;
					set->maxY = maxY;
					set->minZ = midZ;
					set->maxZ = maxZ;
					set->setMid();

					// bottom
					nwb->minX = minX;
					nwb->minY = minY;
					nwb->maxX = midX;
					nwb->maxY = midY;
					nwb->minZ = minZ;
					nwb->maxZ = midZ;
					nwb->setMid();

					neb->minX = midX;
					neb->minY = minY;
					neb->maxX = maxX;
					neb->maxY = midY;
					neb->minZ = minZ;
					neb->maxZ = midZ;
					neb->setMid();

					swb->minX = minX;
					swb->minY = midY;
					swb->maxX = midX;
					swb->maxY = maxY;
					swb->minZ = minZ;
					swb->maxZ = midZ;
					swb->setMid();

					seb->minX = midX;
					seb->minY = midY;
					seb->maxX = maxX;
					seb->maxY = maxY;
					seb->minZ = minZ;
					seb->maxZ = midZ;
					seb->setMid();
				}
				hasChildren = true;

				/* TODO: Binning here somewhere */

				/* move particles to children*/
				HandleAndPos* ptHP = NULL;
				std::deque<HandleAndPos*>::iterator it;
				while(nParticles > 0)
				{
					it = particles.begin();
					ptHP = it[0];
					// calling add again but now we have children and pointer
					add(ptHP->pos, ptHP->handle, ptHP);

					// We keep track of all subparticles when we have children 
					particles.pop_front();
					nParticles--;
				}
				//assert(nParticles == 0);
				// finally adding the current particle now reentering to add to correct child 
				pRes = add(cur, phP);
			}
		}
		return pRes; // return pointer of partricle for subparticle list of parents
	}

	void Tree::setup(FVector* all, int32 iParticleCount) 
	{
		int n = iParticleCount;
		if(n > 0) 
		{
			// find boundaries 
			// we just init with the first 
			minX = all[0].X;
			minY = all[0].Y;
			minZ = all[0].Z;
			maxX = minX;
			maxY = minY;
			maxZ = minZ;
			

			for(int i = 0; i < n; i++) 
			{
				if(all[i].X < minX)
					minX = all[i].X;
				if(all[i].Y < minY)
					minY = all[i].Y;
				if(all[i].Z < minZ)
					minZ = all[i].Z;
				if(all[i].X > maxX)
					maxX = all[i].X;
				if(all[i].Y > maxY)
					maxY = all[i].Y;
				if(all[i].Z > maxZ)
					maxZ = all[i].Z;
			}

			// center and square boundaries
			setMid();
			float diffX = maxX - minX;
			float diffY = maxY - minY;
			float diffZ = maxZ - minZ;

			// find largest dimension
			float halfSide = std::max(diffX, std::max(diffY, diffZ)) / 2;
			minX = midX - halfSide;
			maxX = midX + halfSide;
			minY = midY - halfSide;
			maxY = midY + halfSide;
			minZ = midZ - halfSide;
			maxZ = midZ + halfSide;
			//draw(); //debugdraw
		}
	}

	/*cleanup tree 
	particlereferences are removed from tree
	nodes without particles and children (border cleanup)
	delets particle list 
	keeps children for later use if they just had particles or subparticles */
	void Tree::cleanup()
	{
		if (hasChildren)
		{
			nwt->cleanup();
			net->cleanup();
			swt->cleanup();
			set->cleanup();
			nwb->cleanup();
			neb->cleanup();
			swb->cleanup();
			seb->cleanup();
		}
		else if(nwt != NULL)
		{ // in this node no children were required 
			delete nwt;
			delete net;
			delete swt;
			delete set;
			delete nwb;
			delete neb;
			delete swb;
			delete seb;
			nwt = NULL;
			net = NULL;
			swt = NULL;
			set = NULL;
			nwb = NULL;
			neb = NULL;
			swb = NULL;
			seb = NULL;
		}

		delParticles();
		hasChildren = false;

		subparticles.clear();
		nSubParticles = 0;

		CenterOfMass.X = 0.f;
		CenterOfMass.Y = 0.f;
		Mass = 0; 

		
	}

    /* this is a debug function; 
	it creates the b2bodys around edges of this leef of the 
	tree called from drawonlythis */
	/*void Tree::makeBox()
	{
		b2BodyDef bd;
		m_debugBody = m_world->CreateBody(&bd);
		m_debugBody->SetActive(false);
		b2ChainShape shape;
		const FVector vertices[4] = {
			FVector(minX, minY, maxZ),
			FVector(maxX, minY, maxZ),
			FVector(maxX, maxY, maxZ),
			FVector(minX, maxY, maxZ)
			FVector(minX, minY, minZ),
			FVector(maxX, minY, minZ),
			FVector(maxX, maxY, minZ),
			FVector(minX, maxY, minZ)};
		
		shape.CreateLoop(vertices, 4);  
		m_debugBody->CreateFixture(&shape, 0.0f); 
	}*/ 
	
    /* this is a debug function; 
	it draws a debugbodys around the edges of the tree recursifly */
	/*void Tree::draw(b2World* m_world) {
		this->m_world = m_world;
		destroyBox();
		if(nParticles)
			makeBox();
		
		//ofRect(minX, minY, maxX - minX, maxY - minY);
		if(hasChildren) {
			nw->draw( m_world);
			ne->draw( m_world);
			sw->draw( m_world);
			se->draw( m_world);
		}
	}*/
