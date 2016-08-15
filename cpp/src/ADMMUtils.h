// ADMM
#include <ADMM.h>
#include <gtsam/3rdparty/metis/metis.h>
#include <gtsam/inference/MetisIndex.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


/********************************************************************************************/
/* Metis partitioning */
std::map<int, int> metis(gtsam::NonlinearFactorGraph graph, int num_subgraphs){

  // Instantiate structure for metis
  gtsam::MetisIndex met(graph);
  std::vector<idx_t> xadj = met.xadj();
  std::vector<idx_t> adj = met.adj();
  std::vector<idx_t> objval, part;
  std::map<int, int> partitioning; // Partitioning output
  idx_t nrNodes = met.nValues(); // number of vertices
  idx_t num_balancing_constraints = 1;
  for (idx_t i = 0; i < nrNodes; i++){
    objval.push_back(0);
    part.push_back(0);
  }

  std::cout << "Partitioning..." << std::endl;
  // Call partitioning

  idx_t options[METIS_NOPTIONS];
  METIS_SetDefaultOptions(options);
  options[METIS_OPTION_CONTIG] = 1; // Ensures that partitions are contiguous
  options[METIS_OPTION_MINCONN] = 1;
  options[METIS_OPTION_NCUTS] = 10;


  int outputError = METIS_PartGraphKway(&nrNodes, &num_balancing_constraints, &xadj[0], &adj[0],
      NULL, NULL, NULL, &num_subgraphs, NULL, NULL, &options[0], &objval[0], &part[0]);

  //  outputError = METIS_NodeND(&size, &xadj[0], &adj[0], NULL, NULL, &perm[0], &iperm[0]); // gtsam::Ordering result;

  if (outputError != METIS_OK){
    std::cout << "METIS failed during Partitioning!\n";
    return partitioning; // result;
  }
  std::cout << "Partitioning complete!" << std::endl;

  // Fill in our structure describing the partition: a map from key to partition id
  for (size_t j = 0; j < (size_t)nrNodes; ++j){
    //if(isDebug) std::cout << "key, partition: " << met.intToKey(j) << " " <<  part[j] << std::endl;
    partitioning[met.intToKey(j)] = part[j];
  }

  return partitioning; // result[j] = met.intToKey(j);
}

/********************************************************************************************/
boost::tuple<std::vector<gtsam::NonlinearFactorGraph>, std::vector<gtsam::Values> >
partitionGraph(gtsam::NonlinearFactorGraph graph,
               gtsam::Values initial,
               gtsam::PriorFactor<gtsam::Pose2> posePrior,
               size_t num_subgraphs, bool useMetisPartitioning, bool useLineGraph, bool orderSubgraphs){

  // Subgraph and Subinitials
  std::vector<gtsam::NonlinearFactorGraph> sub_graphs(num_subgraphs, gtsam::NonlinearFactorGraph());
  std::vector<gtsam::Values> sub_initials(num_subgraphs, gtsam::Values());

  size_t subgraph_withPrior_id = 0;
  bool includedPrior = false;

  std::map<int, int> partitioning; // edges -> subgraph

  // EDGE PARTITIONING USING METIS
  if(useMetisPartitioning){

    if(useLineGraph){
      // Create line graph: this allows to use metis for edge partitioning
      gtsam::SharedNoiseModel model = gtsam::noiseModel::Isotropic::Sigma(1, 1.0); // irrelevant
      gtsam::NonlinearFactorGraph lineGraph;
      gtsam::Values lineInitial;
      for (size_t i = 0; i < graph.size(); i++) { // for each factor in the original graph
        for (size_t j = i+1; j < graph.size(); j++) { // for all the other factors
          if(graph[i]->keys().size() != 2){
            std::cout << "Creation of the line graph currently only supports factor graph with between factors" << std::endl;
            return boost::make_tuple(sub_graphs, sub_initials);
          }
          if(graph[i]->keys()[0] == graph[j]->keys()[0] || graph[i]->keys()[1] == graph[j]->keys()[0] ||
             graph[i]->keys()[0] == graph[j]->keys()[1] || graph[i]->keys()[1] == graph[j]->keys()[1])
            lineGraph.add(gtsam::BetweenFactor<double>(i,j,0.0,model));
        }
      }

      // Do edge partitioning using Metis
      partitioning = metis(lineGraph, num_subgraphs);

      // Go through the factors and push each one to the corresponding subgraph
      std::map<int, int>::iterator it;
      for (it =partitioning.begin(); it != partitioning.end(); it++){
        int factor_id = it->first;
        int subgraph_id = it->second;
        sub_graphs[subgraph_id].add(graph[factor_id]);

        gtsam::Key key0 =  graph[factor_id]->keys()[0];
        gtsam::Key key1 =  graph[factor_id]->keys()[1];

        if(!includedPrior && key0 == 0){
          sub_graphs[subgraph_id].add(posePrior);
          subgraph_withPrior_id = subgraph_id;
          includedPrior = true;
        }

        if(!sub_initials[subgraph_id].exists(key0))
          sub_initials[subgraph_id].insert(key0, initial.at(key0));

        if(!sub_initials[subgraph_id].exists(key1))
          sub_initials[subgraph_id].insert(key1, initial.at(key1));
      }
    }
    else{
      partitioning = metis(graph, num_subgraphs);

      // Go through the factors and push each one to the corresponding subgraph
      std::map<int, int>::iterator it;
      for (it =partitioning.begin(); it != partitioning.end(); it++){
        int vertex_id = it->first;
        int subgraph_id = it->second;
        sub_initials[subgraph_id].insert(vertex_id, initial.at(vertex_id));
      }


      std::vector<int> separatorEdges;
      for(size_t factor_id = 0; factor_id < graph.size(); factor_id++)
      {
        gtsam::Key key0 =  graph[factor_id]->keys()[0];
        gtsam::Key key1 =  graph[factor_id]->keys()[1];

        int foundInSubgraphs = 0;
        int subgraphOne = -1;
        int subgraphTwo = -1;
        for(size_t sub_id=0; sub_id < num_subgraphs; sub_id++){

          if(!includedPrior && sub_initials[sub_id].exists(key0) && key0 == 0){
            sub_graphs[sub_id].add(posePrior);
            subgraph_withPrior_id = sub_id;
            includedPrior = true;
          }

          if(sub_initials[sub_id].exists(key0) && sub_initials[sub_id].exists(key1)){ // both the vertices belong to this subgraph
            sub_graphs[sub_id].add(graph[factor_id]);
            break;
          }
          else if(sub_initials[sub_id].exists(key0) || sub_initials[sub_id].exists(key1)){
            foundInSubgraphs++;
            if(subgraphOne == -1)
              subgraphOne = sub_id;
            else
              subgraphTwo = sub_id;

            if(foundInSubgraphs == 2){ // found in 2 subgraphs
              separatorEdges.push_back(factor_id);
              // found between subgraphOne and subgraphTwo
              //TODO: Randomly push this factor to one of the subgraphs
              int chosenSubgraph = subgraphTwo;
              sub_graphs[chosenSubgraph].add(graph[factor_id]);

              if(!sub_initials[chosenSubgraph].exists(key0))
                sub_initials[chosenSubgraph].insert(key0, initial.at(key0));
              if(!sub_initials[chosenSubgraph].exists(key1))
                sub_initials[chosenSubgraph].insert(key1, initial.at(key1));
            }
          }
        }
      }
      std::cout << "Done looping" << std::endl;

    }

  }else{
    // graph.print("GRAPH");
    // EDGE PARTITIONING USING ODOMETRY (Creates chain decomposition)
    std::vector<bool> assignedEdges(graph.size(), false);
    int numNodes = initial.size();
    int numOdometricEdges = numNodes-1;
    int partition_size = floor(numOdometricEdges/num_subgraphs);

    //std::cout << "Num of nodes: " << numNodes << std::endl <<  "Num of odometric edges: " << numOdometricEdges << std::endl << "Partition size: " << partition_size << std::endl << "Size of graph: " << graph.size() << std::endl;
    size_t factor_id = 0;
    for(size_t sub_id =0; sub_id < num_subgraphs; sub_id++){ // for each subgraph
      int init_pose = factor_id;
      int final_pose = init_pose + partition_size;
      if(sub_id == num_subgraphs - 1)
        final_pose = graph.size();
      factor_id = init_pose;
      for (int i = init_pose ; i < final_pose; ) { // for each factor in the original graph
        sub_graphs[sub_id].add(graph[factor_id]);

        gtsam::Key key0 =  graph[factor_id]->keys()[0];
        gtsam::Key key1 =  graph[factor_id]->keys()[1];

        size_t key0Index = gtsam::symbolIndex(key0);
        size_t key1Index = gtsam::symbolIndex(key1);

        if(abs(key0Index - key1Index)==1){  // Increase i only when its an odometry constraint
          i++;
        }

        if(!includedPrior && key0 == 0){
          sub_graphs[sub_id].add(posePrior);
          subgraph_withPrior_id = sub_id;
          includedPrior = true;
        }

        if(!sub_initials[sub_id].exists(key0))
          sub_initials[sub_id].insert(key0, initial.at(key0));

        if(!sub_initials[sub_id].exists(key1))
          sub_initials[sub_id].insert(key1, initial.at(key1));

        factor_id++;
        if (factor_id == graph.size())
          break;

      }
    }

  }  

  // REORGANIZE THE GRAPHS: put graph with prior first, then graphs connected with first, and so on
  if(orderSubgraphs){
    std::vector<gtsam::NonlinearFactorGraph> ordered_sub_graphs(num_subgraphs, gtsam::NonlinearFactorGraph());
    std::vector<gtsam::Values> ordered_sub_initials(num_subgraphs, gtsam::Values());
    ordered_sub_graphs[0] = sub_graphs[subgraph_withPrior_id];
    ordered_sub_initials[0] = sub_initials[subgraph_withPrior_id];
    int incrementalId = 1;
    for(size_t i = 0; i < num_subgraphs; i++){
      if(i != subgraph_withPrior_id){
        ordered_sub_graphs[incrementalId] = sub_graphs[i];
        ordered_sub_initials[incrementalId] = sub_initials[i];
        incrementalId += 1;
      }
    }
    for(size_t i = 0; i < num_subgraphs; i++){
      sub_graphs[i] = ordered_sub_graphs[i];
      sub_initials[i] = ordered_sub_initials[i];
    }
  }

  return boost::make_tuple(sub_graphs, sub_initials);
}

/********************************************************************************************/
std::vector<int>
createSeparators(std::vector<gtsam::Values> sub_initials, gtsam::Values initial){
  size_t num_subgraphs = sub_initials.size();
  std::vector<int> separators;
  BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair &keyValue, initial ){
    gtsam::Key key = keyValue.key;
    size_t keyCounter = 0; // counts how many pairs of subgraphs the node separates
    for(size_t i = 0; i < num_subgraphs; i++){
      bool isInSubGraph_i = false;
      if(sub_initials[i].exists(key))
        isInSubGraph_i = true;

      for(size_t j = i+1; j < num_subgraphs; j++){
        bool isInSubGraph_j = false;
        if(sub_initials[j].exists(key))
          isInSubGraph_j = true;

        if(isInSubGraph_i && isInSubGraph_j){
          keyCounter += 1;
          separators.push_back(key);
          separators.push_back(i);
          separators.push_back(j);
        }
      }
    }
  }
  return separators;
}

/********************************************************************************************/
void logResults(std::vector<gtsam::NonlinearFactorGraph> sub_graphs,
                std::vector<gtsam::Values> sub_results,
                gtsam::NonlinearFactorGraph graph,
                gtsam::Values initial,
                gtsam::PriorFactor<gtsam::Pose2> posePrior,
                std::string output_dir){

  std::string filepath; // g2o file
  // Pack ADMM results into a single Value structure, to check centralized error
  gtsam::Values ADMMresult;
  for(size_t i=0; i<sub_results.size(); i++){
    BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair &keyValue, sub_results[i]){
      if(!ADMMresult.exists(keyValue.key))
        ADMMresult.insert(keyValue.key, keyValue.value);
    }
  }
  std::cout << "Centralized error using ADMM results " << graph.error(ADMMresult) << std::endl;


  // CENTRALIZED OPTIMIZATION, for benchmark
  // Add prior to centralized graph
  gtsam::NonlinearFactorGraph graphWithPrior = graph;
  graphWithPrior.add(posePrior);

  // Optimize centralized graph
  ADMM::Time GN_start = boost::posix_time::microsec_clock::local_time ();    // GN start timer
  gtsam::GaussNewtonParams params;
  gtsam::GaussNewtonOptimizer GNoptimizer(graphWithPrior, initial, params);

  gtsam::Values GNresult = GNoptimizer.optimize();
  ADMM::Time GN_end = boost::posix_time::microsec_clock::local_time (); // GN end timer
  boost::posix_time::time_duration GN_time = GN_end - GN_start;
  double GN_duration_microsec = GN_time.total_microseconds();
  std::cout << "Centralized error GN " << graph.error(GNresult) << std::endl;

  filepath = output_dir + "/gtsamTime.txt";
  std::fstream gtsamTimeOutput(filepath.c_str(), std::fstream::out);
  gtsamTimeOutput << GN_duration_microsec << std::endl;
  gtsamTimeOutput.close();

  gtsam::LevenbergMarquardtOptimizer LMoptimizer(graphWithPrior, initial);
  gtsam::Values LMresult = LMoptimizer.optimize();

  // std::cout << "Centralized error x 2 " << 2*graph.error(result) << std::endl;
  std::cout << "Centralized error LM " << graph.error(LMresult) << std::endl;

  double total_memory_consumption = 0;
  // Save the results
  for(size_t sub_id =0; sub_id < sub_results.size(); sub_id++){
    std::string filepath = output_dir + "/" + boost::lexical_cast<std::string>(sub_id) + ".g2o";
    gtsam::writeG2o(sub_graphs[sub_id], sub_results[sub_id], filepath);

    // Save the average sub-graph memory consumption
    total_memory_consumption += (18+3)*sizeof(double)*sub_graphs[sub_id].size(); // Assuming each edge takes the size of 3 doubles corresponding to delta_x, delta_y, delta_theta
    total_memory_consumption += 3*sizeof(double)*sub_results[sub_id].size(); // Assuming each vertex takes the size of 3 doubles corresponding to x,y,theta
  }
  double avg_memory_consumption = total_memory_consumption*1.0f/sub_results.size();
  double full_graph_memory_consumption = (18+3)*sizeof(double)*graph.size() + 3*sizeof(double)*initial.size();

  filepath = output_dir + "/memoryConsumption.txt";
  std::fstream memoryConsumption(filepath.c_str(), std::fstream::out);
  memoryConsumption << avg_memory_consumption << "," << full_graph_memory_consumption <<  std::endl;
  memoryConsumption.close();

  filepath = output_dir + "/centralizedGraph.g2o";
  gtsam::writeG2o(graph, GNresult, filepath);

  filepath = output_dir + "/centralizedGraph_with_initial.g2o";
  gtsam::writeG2o(graph, initial, filepath);

  filepath = output_dir + "/admmGraph.g2o";
  gtsam::writeG2o(graph, ADMMresult, filepath);
}
