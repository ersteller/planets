
# TODOs:
- [x] replace b2.. (Box2D) types from source with UE types
   - [x] b2Vec --> FVector
   - [x] b2ParticleHandle --> void*
- [x] find data or functions of engine for position mass and which handle
- [ ] define interface for bodyhandles array and position array and mass array and how to store it efficiently in the setup of tree
- [ ] check if call in blueprint works with setup and step loop 

- [x] add third dimension 
   - [x] extend tree with upper and lower quads
   - [x] check distance calkulation
   - [x] check center off mass calculation
- [ ] check for dynamic radius
- [ ] check for dynamic mass

### UE git setup from: 
https://github.com/MOZGIII/ue4-gitignore/

### quadtree concept implementation: 
stolen from https://code.google.com/p/kyle/
