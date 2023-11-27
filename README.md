# Body-Brain Quality-Diversity in Robot Swarms

This project is part of an ongoing thesis that investigates novel artificial life methodologies for demonstrating various swarm robotic systems as autonomous and adaptive collective behaviour systems. The research goal is to produce in situ robot factories that automate the automated design and production of self-replicating robot populations capable of exhibiting a broad range of collective behaviors. Potential collective construction manifestations include construction of novel, customized and dynamic functional structures (equipment) that contributes to task discovery, fitness function shaping and thus overall mission accomplishment -- for example, automated decentralized collective construction by cooperating robots, built on-the-fly given site-specific environmental conditions and constraints.

This project makes use of and extends the open source [Robogen](https://github.com/lis-epfl/robogen) platform for the simulated co-evolution of robot bodies and brains.  
The folder 'robogen' contains all the code used to run the experiments for this project and should be run on Linux.  
The folder 'robogen-windows' is included as the precompiled executable robogen-file-viewer (for viewing evolved robots in their simulation environments) was not available for linux.  

## Installation and Compilation

Robogen has numerous dependencies and tools that need to be installed. Instructions for doing so can be found at http://robogen.org/docs/robogen-with-source/.  

To compile on linux (using CMake):  
```
cd to the robogen/build directory
cmake -DCMAKE_BUILD_TYPE=Release -G"Unix Makefiles" ../src/
make -j3
```
