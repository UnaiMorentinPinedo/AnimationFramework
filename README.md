# AnimationFramework

Animation framework implemented in C++ from scratch. It uses GLTF to load skinned models and animations. Capable of creating 1D and 2D blend spaces on the editor. Implementation of Inverse Kinematics moving an end-effector in the editor. Also, implemented curve motion of animated objects.


## Blending

1D and 2D Animation blending. Implemented a solid API for handling 1D and 2D blending. 1D blending uses blend Lerp  and 2D uses barycentric coordinates.

## IK

Inverse Kinematics using several algorithms: 2D Analytical 2-bone IK solver, Cyclic Coordinated Descent IK Solver (CCD) and FABRIK.

## Path Following

Cuve motion of animated object with root motion based on distance, speed and orientation control and basic animation syncing.

## Skeletal Animation

Implementation of skeletal animation for skinned models with complex skeleton such as articulated humna-like (biped) models containing at least 16 links.
