#!/bin/bash

# Params
isDebug=1;
numSubgraphs=10; # Num of subgraphs
datasets=('m3500_g2o.g2o' 'CSAIL_I_eq.g2o' 'FR079_I_eq.g2o'  'ais2klinik.g2o'  'intel.g2o') # Datasets on which we'll experiment
datasetDir='../data/benchmark-data/'

mkdir results
# Run experiments on datasets
for dataset in ${datasets[@]}; 
do
	#Run code	
	echo "---------------------"			
	resultDir='results/dataset_'$dataset'_n_'$numSubgraphs
	admm_arguments='-i '$datasetDir$dataset' -n '$numSubgraphs'  -r '$resultDir' --verbosity '$isDebug 
	../cpp/build/PartitionAndRunADMM $admm_arguments 
	echo "---------------------"			
done







