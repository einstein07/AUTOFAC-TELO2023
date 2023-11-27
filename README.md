# Body and Brain Quality-Diversity in Robot Swarms

In biological societies, complex interactions between the behavior and morphology of evolving organisms and their environment have given rise to a wide range of complex and diverse social structures. Similarly, in artificial counterparts such as swarm-robotics systems, collective behaviors emerge via the interconnected dynamics of robot morphology (sensory-motor configuration), behavior (controller), and environment (task). Various studies have demonstrated morphological and behavioral diversity enables biological groups to exhibit adaptive, robust, and resilient collective behavior across changing environments. However, in artificial (swarm robotic) systems, there is little research on the impact of changing environments on morphological and behavioral (body-brain) diversity in emergent collective behavior, and the benefits of such diversity. 

This study uses evolutionary collective robotics as an experimental platform to investigate the impact of increasing task environment complexity (collective behavior task difficulty) on the evolution and benefits of morphological and behavioral diversity in robotic swarms. Results indicate that body-brain evolution using coupled behavior and morphology diversity maintenance yields higher behavioral and morphological diversity, which is beneficial for collective behavior task performance across task environments. Results also indicate that such behavioral and morphological diversity maintenance, coupled with body-brain evolution produces neuro-morpho complexity that does not increase concomitantly with task complexity

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
