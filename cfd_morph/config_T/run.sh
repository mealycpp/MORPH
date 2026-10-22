#!/bin/bash
set -e
echo "Running T-config CFD with drone geometry..."

# Base mesh
blockMesh

# Extract surface features for snapping
surfaceFeatureExtract

# Refine mesh around drone body
snappyHexMesh -overwrite

# Create actuator disk zones
topoSet

# Solve
simpleFoam | tee log.simpleFoam

echo "Done. Open in ParaView: paraview T.foam"
touch T.foam
