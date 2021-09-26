
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




/* TODO: define qoutient s/d (spread / distance) to conditionally use above*/

/* TODO: binning: find large bodies of connectect particles */
/* 1. make round static bodys to a bin */
/* 2. tidal force or impact should break bins (testcase: check roche border) d= 2.423 * R * (rohM/rohm)^(1/3)*/

	char* Tree::getAncestors(char* buff)
	{
		Tree* w = this;
		int d = GetDepth() * 3;
		memcpy(&buff[0], " ro", 3);
		while (w->parent != NULL)
		{
			if(w == w->parent->nw)
				memcpy(&buff[d], "_nw", 3);
			else if(w == w->parent->ne)
				memcpy(&buff[d], "_ne", 3);		
			else if(w == w->parent->sw)
				memcpy(&buff[d], "_sw", 3);
			else if(w == w->parent->se)
				memcpy(&buff[d], "_se", 3);
			//else 
				//assert(0);
			d-=3;
			//assert(d>=0);
			w = w->parent;
		}	 
		return buff;
	}

	// k error between 0 and 1 
	// if lager than 1 we are only evaluating root 
	// if k == 0 we use direct method
	float Tree::GetK(FVector* cur)
	{
		// spread of box
		FVector direction;
		float radius = ((midY - minY) * 2);
	    direction.X = midX - cur->X;
		direction.Y = midY - cur->Y;
		float length = abs(direction.X) + abs(direction.Y); // direction.Length(); // this takes 10%of all time direction.Length(); 


		// k is an indicator for expectet error impact 
		// sensable between 0 and 1 if lager than 1 we are very close to or inside this node
		float k = 0;
		if (length != 0)
			k = radius / length;
		else 
			k = std::numeric_limits<float>::max();		
		return k;
	}


	/*Tree* oldNext(float kerror, FVector* cur)
	{
		Tree* ret = NULL;
		Tree* worker = this;
		return NULL;
	}*/


	/// find next node after this one (sequence is nw --> ne --> sw --> se --> parent-parent)
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
				worker = worker->nw;
			}
			// we are now at the correct level
			// skip border nodes
			if (worker->nSubParticles != 0 || worker->nParticles != 0)
				ret = worker;
			else 
				ret = worker->Next(kerror,cur);
			return ret;
		}

		while (worker->parent != NULL && worker == worker->parent->se )
		{
			// at the end of children go up
			worker = worker->parent; 
			// and one more next to the side 
		}
				
		// get next of same layer 
		if (worker->parent != NULL)
		{
			if(worker == worker->parent->nw)
				worker = worker->parent->ne;
			else if(worker == worker->parent->ne)
				worker = worker->parent->sw;			
			else if(worker == worker->parent->sw)
				worker = worker->parent->se;
			//else 
				//assert(0); // did not find itself in the children of parent
			

			while (worker->hasChildren && worker->GetK(cur) > kerror)
			{
				// we should go deeper
				worker = worker->nw;
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
		//TODO: CenterOfMass.Z = CenterOfMass.Z * Mass / NewMass + cur->Z * CurMass / NewMass;
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
					pRes = nw->add(cur, phP, ptHPin);
				} 
				else 
				{
					pRes = sw->add(cur, phP, ptHPin);
				}
			} 
			else 
			{
				if(cur->Y < midY) 
				{
					pRes = ne->add(cur, phP, ptHPin);
				} 
				else 
				{
					pRes = se->add(cur, phP, ptHPin);
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
				
				{ 
				    float CurMass = 1; //TODO: mass of this particle may be individuel
				    UpdateCenterOfMass(cur, CurMass);
			    }

				
			} else {

				if(nw == NULL)
				{
					nw = new Tree(minX, minY, midX, midY, this);
					ne = new Tree(midX, minY, maxX, midY, this);
					sw = new Tree(minX, midY, midX, maxY, this);
					se = new Tree(midX, midY, maxX, maxY, this);
				}
				else
				{ 
					/* only update positions*/
					nw->minX = minX;
					nw->minY = minY;
					nw->maxX = midX;
					nw->maxY = midY;
					nw->setMid();

					ne->minX = midX;
					ne->minY = minY;
					ne->maxX = maxX;
					ne->maxY = midY;
					ne->setMid();

					sw->minX = minX;
					sw->minY = midY;
					sw->maxX = midX;
					sw->maxY = maxY;
					sw->setMid();

					se->minX = midX;
					se->minY = midY;
					se->maxX = maxX;
					se->maxY = maxY;
					se->setMid();
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
			minX = all[0].X;
			minY = all[0].Y;
			maxX = minX;
			maxY = minY;

			for(int i = 0; i < n; i++) 
			{
				if(all[i].X < minX)
					minX = all[i].X;
				if(all[i].Y < minY)
					minY = all[i].Y;
				if(all[i].X > maxX)
					maxX = all[i].X;
				if(all[i].Y > maxY)
					maxY = all[i].Y;
			}

			// center and square boundaries
			setMid();
			float diffX = maxX - minX;
			float diffY = maxY - minY;
			float halfSide = std::max(diffX, diffY) / 2;
			minX = midX - halfSide;
			maxX = midX + halfSide;
			minY = midY - halfSide;
			maxY = midY + halfSide;

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
			nw->cleanup();
			ne->cleanup();
			sw->cleanup();
			se->cleanup();
		}
		else if(nw != NULL)
		{ // in this node no children were required 
			delete nw;
			delete ne;
			delete sw;
			delete se;
			nw = NULL;
			ne = NULL;
			sw = NULL;
			se = NULL;
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
