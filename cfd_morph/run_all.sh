#!/bin/bash
# Run all morphing drone CFD configurations
# Usage: ./run_all.sh

set -e


echo ""
echo "============================================================"
echo "  Running X-config"
echo "============================================================"
cd config_X

# Generate base mesh
blockMesh

# Create cell zones for actuator disks
topoSet

# Solve
simpleFoam | tee log.simpleFoam

# Post-process: sample velocity along lines
postProcess -func "sampleDict" -latestTime || true

cd ..

echo ""
echo "============================================================"
echo "  Running O-config"
echo "============================================================"
cd config_O

# Generate base mesh
blockMesh

# Create cell zones for actuator disks
topoSet

# Solve
simpleFoam | tee log.simpleFoam

# Post-process: sample velocity along lines
postProcess -func "sampleDict" -latestTime || true

cd ..

echo ""
echo "============================================================"
echo "  Running T-config"
echo "============================================================"
cd config_T

# Generate base mesh
blockMesh

# Create cell zones for actuator disks
topoSet

# Solve
simpleFoam | tee log.simpleFoam

# Post-process: sample velocity along lines
postProcess -func "sampleDict" -latestTime || true

cd ..

echo ""
echo "============================================================"
echo "  Running H-config"
echo "============================================================"
cd config_H

# Generate base mesh
blockMesh

# Create cell zones for actuator disks
topoSet

# Solve
simpleFoam | tee log.simpleFoam

# Post-process: sample velocity along lines
postProcess -func "sampleDict" -latestTime || true

cd ..
