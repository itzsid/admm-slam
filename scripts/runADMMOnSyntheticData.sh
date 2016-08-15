#!/bin/bash

# Params
isDebug=1;
solver=0; #GN
numSubgraphs=(16 25 49 81 144); # Num of subgraphs
datasets=('width49' 'width151' 'width211' 'width271' 'width361') # Datasets on which we'll experiment
datasetDir='../data/blocks_world/'

# make result directory
mkdir results_blocks_world

count=0;
# Run experiments on datasets
for dataset in ${datasets[@]}; 
do	
	#Run code
        echo "---------------------"
	resultDir='results_blocks_world/'$dataset
	admm_arguments='-i '$datasetDir$dataset/' -n '${numSubgraphs[$count]}' -r '$resultDir
	../cpp/build/RunADMM $admm_arguments 
	((count++))
        echo "---------------------"
done






