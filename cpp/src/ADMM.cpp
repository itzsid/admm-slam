#include <ADMM.h>

#define USE_L2_NORM 1

using namespace std;
using namespace gtsam;

/********************************************************************************************/
void
ADMM::logResult(int iter, double p_res, double d_res, double time_taken){

  // Create directories if they do not exist to save intermediate graphs
  logDir_ = outputDir_ + ("/intermediateGraphs/");
  if( !(boost::filesystem::exists(logDir_))){
    boost::filesystem::create_directory(logDir_);
  }

  subgraphLogDir_ = outputDir_ + ("/intermediateSubGraphs/");
  if( !(boost::filesystem::exists(subgraphLogDir_))){
    boost::filesystem::create_directory(subgraphLogDir_);
  }

  // Pack results into a single Value structure
  Values intermediateResult;
  for(size_t i=0; i<subinitials_.size(); i++){
    for (const Values::ConstKeyValuePair &keyValue: subinitials_[i]){
      if(!intermediateResult.exists(keyValue.key))
        intermediateResult.insert(keyValue.key, keyValue.value);
    }

    if(logCount_ > logCountThresh_){
    string subgraph_filepath = subgraphLogDir_ + boost::lexical_cast<string>(iter) + "_" + boost::lexical_cast<string>(i)  + ".g2o";
    //writeG2o(subgraphs_[i], subinitials_[i], subgraph_filepath);
    }
  }

  if(logCount_ > logCountThresh_){
  string filepath = logDir_ + boost::lexical_cast<string>(iter) + ".g2o";
  //writeG2o(fullGraph_, intermediateResult, filepath);
  logCount_ = 0;
  }

  double admmError = 2*fullGraph_.error(intermediateResult);
  *residualLog_ << iter << ", " << p_res << ", " << d_res << ", " << rho_ << ", " << admmError <<  "," << time_taken << endl;

  if(verbosity_ >= DEBUG)
    cout << "ADMM Error: " << admmError << endl;
}

/********************************************************************************************/
// Flagged Initialization
void
ADMM::FlaggedInitialization(){

  GaussNewtonParams params;
  params.setVerbosity("ERROR");
  params.setRelativeErrorTol(-1e+10);
  params.setAbsoluteErrorTol(-1e+10);
  params.setMaxIterations(3);
  int num_subgraphs = subgraphs_.size();
  int nrSeparators = separators_.size()/3;

  SharedNoiseModel rhoNoiseModel = noiseModel::Isotropic::Variance(3,2/rho_);

  vector<Values> initial_old(subinitials_.size(), Values());
  for(int sub_id =0; sub_id < num_subgraphs; sub_id++)
    initial_old[sub_id] = Values(subinitials_[sub_id]);

  vector<int> initFlag(subinitials_.size(), 0); // 0: subgraph is not optimized yet, 1: subgraph to be optimized in next iteration, 2: subgraph optimized
  initFlag[0] = 1;

  /// Keep optimizing un-initialized subgraphs
  while(1){
    bool optimizedSomething = false; // check if it has optimized atelast one sub-graph
    for(int sub_id =0; sub_id < num_subgraphs; sub_id++){
      if(initFlag[sub_id] == 1){
        if (verbosity_ >= DEBUG)
          printf("Flagged Initialization of %d\n", sub_id);
        optimizedSomething = true;

        // Create the local problem
        NonlinearFactorGraph sub_graph = subgraphs_[sub_id];
        Values sub_initial = initial_old[sub_id];

        // add penalty factors for each constraint (i.e., for each separator)
        for(int i=0; i< nrSeparators; i++){

          if (separators_[3*i+1] == sub_id){ // behaves like f_x
            int key = separators_[3*i]; // separator node symbol in the current subgraph

            int graph_index = separators_[3*i+2]; // index of the neighboring subgraph

            if(initFlag[graph_index] ==2){ // Add the BiasedPriorFactor if the neighboring graph is already initialized
              Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
              Pose2 pose_z = initial_old[graph_index].at<Pose2>(key);
              sub_graph.add(BiasedPriorFactor<Pose2, Vector>(key, pose_z, -u_k, rhoNoiseModel));
            }
            else{
              initFlag[graph_index] = 1; // Optimize it in the next iteration
            }
          }
          else if (separators_[3*i + 2] == sub_id) // behaves like g_z
          {
            int key = separators_[3*i]; //separator node symbol in the current subgraph
            int graph_index = separators_[3*i+1]; //index of the neighboring subgraph

            if(initFlag[graph_index] ==2){ // Add the BiasedPriorFactor if the neighboring graph is already initialized

              Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
              Pose2 pose_x = initial_old[graph_index].at<Pose2>(key);
              sub_graph.add(BiasedPriorFactor<Pose2, Vector>(key, pose_x, u_k, rhoNoiseModel));
            }
            else{
              initFlag[graph_index] = 1; // Optimize it in the next iteration
            }
          }
        }

        if(solver_ == LAGO){
          subinitials_[sub_id] = lago::initialize(sub_graph, false);
        }
        else if(solver_ == GN){
          GaussNewtonOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
          subinitials_[sub_id] = optimizer_subgraph.optimize();
        }
        else if(solver_ == LM){
          cout << endl << "----------------" << endl;
          LevenbergMarquardtParams params;
          params.setlambdaInitial(1e-9);
          params.setVerbosityLM("TRYLAMBDA");

          LevenbergMarquardtOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
          subinitials_[sub_id] = optimizer_subgraph.optimize();
          cout << endl << "----------------" << endl;
        }
        else{
          printf("ERROR: Specify solver type\n");
        }

        initFlag[sub_id] = 2; // subGraph initialized
      }
    }

    if(!optimizedSomething){ // Break if we haven't optimized anything
      if (verbosity_ >= DEBUG)
        printf("Done initializing\n");
      break;
    }
  } // end of loop over subgraphs

}


/********************************************************************************************/
// ADMM Scaled Iteration
pair<double, double>
ADMM::iterate(int iter){

  vector<VectorValues> subgraph_gradient(subgraphs_.size(), VectorValues());

  GaussNewtonParams params;
  if(iter == 0)
  {
    params.setRelativeErrorTol(-1e+10);
    params.setAbsoluteErrorTol(-1e+10);
    //params.setVerbosity("ERROR");
    params.setMaxIterations(3);
  }

  int num_subgraphs = subgraphs_.size();
  int nrSeparators = separators_.size()/3;

  // The penalty parameter plays the role of an information content
  SharedNoiseModel rhoNoiseModel = noiseModel::Isotropic::Variance(3,2/rho_);

  vector<Values> initial_old(subinitials_.size(), Values());
  for(int sub_id =0; sub_id < num_subgraphs; sub_id++)
    initial_old[sub_id] = Values(subinitials_[sub_id]);

  /// Optimize each subgraph independently, but possibly sequentially (if parallel = false)
  for(int sub_id =0; sub_id < num_subgraphs; sub_id++){
    //    cout << "SUB ID " << sub_id << endl;
    // update sequentially
    if (!isParallel_)
      initial_old = subinitials_; // we use the latest estimate for all subgraphs

    // Create the local problem
    NonlinearFactorGraph sub_graph = subgraphs_[sub_id];
    Values sub_initial = initial_old[sub_id];

    // if(sub_id != 0){
    // add penalty factors for each constraint (i.e., for each separator)
    for(int i=0; i< nrSeparators; i++){
      if (separators_[3*i+1] == sub_id){ // behaves like f_x
        int key = separators_[3*i]; // separator node symbol in the current subgraph

        int graph_index = separators_[3*i+2]; // index of the neighboring subgraph
        Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
        Pose2 pose_z = initial_old[graph_index].at<Pose2>(key);
        sub_graph.add(BiasedPriorFactor<Pose2, Vector>(key, pose_z, -u_k, rhoNoiseModel));
      }
      else if (separators_[3*i + 2] == sub_id) // behaves like g_z
      {
        int key = separators_[3*i]; //separator node symbol in the current subgraph

        int graph_index = separators_[3*i+1]; //index of the neighboring subgraph
        Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
        Pose2 pose_x = initial_old[graph_index].at<Pose2>(key);
        sub_graph.add(BiasedPriorFactor<Pose2, Vector>(key, pose_x, u_k, rhoNoiseModel));
      }
    }


    if(solver_ == LAGO){
      subinitials_[sub_id] = lago::initialize(sub_graph, false);
    }
    else if(solver_ == GN){
      GaussNewtonOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
      subinitials_[sub_id] = optimizer_subgraph.optimize();
    }
    else if(solver_ == LM){
      LevenbergMarquardtParams params;
      params.setlambdaInitial(1e-7);
      LevenbergMarquardtOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
      subinitials_[sub_id] = optimizer_subgraph.optimize();
    }
    else{
      printf("ERROR: Specify solver type\n");
    }

    // Subgraph gradient to be used later to compute dual residual
    subgraph_gradient[sub_id] = subgraphs_[sub_id].linearize(subinitials_[sub_id])->gradientAtZero();

  } // end of loop over subgraphs

  // Dual update & computation of primal and dual residual for stopping conditions
  double p_res = 0, d_res = 0;

  NonlinearFactorGraph constraintsGraph; // to compure d_res
  for(int i=0; i<nrSeparators; i++){ // for each separator
    int key = separators_[3*i];
    int sub_map1 = separators_[3*i+1];
    int sub_map2 = separators_[3*i+2];
    Pose2 pose_x = subinitials_[sub_map1].at<Pose2>(key);
    Pose2 pose_z = subinitials_[sub_map2].at<Pose2>(key);

    Matrix J_x, J_z; // Jacobian matrix w.r.t x and z
    Pose2 between_x_z = pose_x.between(pose_z, J_x, J_z);
    Vector b = Pose2::Logmap(between_x_z);

    if(computeSubgraphGradient_){
      // add the corresponding jacobians to the subgraph gradient of the corresponding subgraphs
      // 1/2 is used to compensate the definition of the cost function in GTSAM being multiplied by 1/2
      Vector y_k = (rho_/2) * Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
      Vector scaled_Jx = y_k.transpose()*J_x;
      Vector scaled_Jz = y_k.transpose()*J_z;

      if(subgraph_gradient[sub_map1].exists(key)){
        subgraph_gradient[sub_map1].at(key) += scaled_Jx;
      }
      else{
        subgraph_gradient[sub_map1].insert(key, scaled_Jx);
      }

      if(subgraph_gradient[sub_map2].exists(key)){
        subgraph_gradient[sub_map2].at(key) += scaled_Jz;
      }
      else{
        subgraph_gradient[sub_map2].insert(key, scaled_Jz);
      }
    }
    else{
      // kind of dual residual
      Pose2 pose_z_old = initial_old[sub_map2].at<Pose2>(key);
      pose_z  =  subinitials_[sub_map2].at<Pose2>(key);
      Vector bz = Pose2::Logmap(pose_z_old.between(pose_z));

      Values valuesForGradient;
      valuesForGradient.insert(0,pose_x); valuesForGradient.insert(1,pose_z);
      BetweenFactor<Pose2> factor = BetweenFactor<Pose2>(0, 1, Pose2(), rhoNoiseModel);
      d_res = d_res + factor.linearize(valuesForGradient)->gradientAtZero().norm();
      // d_res = d_res + rho * bz.norm(); // old version
    }

    // primal residual (Body11fnt, pag 18)
#if USE_L2_NORM  == 0
    double bnorm = b.norm();
    if (bnorm > p_res)
      p_res = bnorm;
#else
    p_res = p_res + b.norm();
#endif

    // update of scaled dual variables, component-wise
    uall_k[3*i]   = uall_k[3*i]   + b[0];
    uall_k[3*i+1] = uall_k[3*i+1] + b[1];
    uall_k[3*i+2] = uall_k[3*i+2] + b[2];
  }

  if (computeSubgraphGradient_){

    // Iterate over subgraphs and add \delta f(sub_id)
    for(int sub_id =0; sub_id < num_subgraphs; sub_id++){
      double gradient_norm = subgraph_gradient[sub_id].norm();
      d_res = d_res + gradient_norm*gradient_norm;
    }
    d_res = sqrt(d_res);
  }

  // return result of current iteration, and primal/dual residuals
  return make_pair(p_res, d_res);
}

/********************************************************************************************/
// ADMM - return sub_initials (optimized), and a vector of dual variables
pair<vector<Values>, vector<double> >
ADMM::optimize(){

  //Config
  double p_res, d_res; // primal and dual residual, used to check stopping conditions

  // Residual log
  string residualLog = outputDir_ + "/residualLog.txt";
  residualLog_->open(residualLog.c_str(), fstream::out);


  if(verbosity_ >= DEBUG){
    cout << "========================================" << endl;
    cout << "Parameters:  "
            "\n-rho: " << rho_ << "(initial)" <<
            "\n-mu: "  << mu_ <<
            "\n-tau: "  << tau_ <<
            "\n-maxIter: "  << maxIter_ <<
            "\n-isParallel: "  << isParallel_ <<
            "\n-adaptivePenalty: "  << adaptivePenalty_ <<
            "\n-min_p_res_: "  << min_p_res_ <<
            "\n-min_d_res_: "  << min_d_res_ <<
            "\n-num_subgraphs: " << subgraphs_.size() <<
            "\n-solver-type: " << solver_ << endl;
  }

  logCount_ = 0, logCountThresh_ = 10;

  // Start ADMM Timer
  ADMM::Time admm_start = boost::posix_time::microsec_clock::local_time ();

  // Flagged initialization
  if (useFlaggedInitialization_)
    FlaggedInitialization();


  // run ADMM iterations
  for(int iter = 0; iter < maxIter_; iter++){

    // Start iter timer
    tic();

    if(verbosity_ >= DEBUG)
      cout << "============== Iter: " << iter << " ==============" << endl;

    // run single ADMM iteration
    boost::tie(p_res, d_res)  = iterate(iter);

    // Stop timer
    double time_duration = toc();

    // update penalty parameter, when using adaptivePenalty
    if(adaptivePenalty_ && p_res > mu_ * d_res)
      rho_ = rho_ * tau_; // increase penalty, to try to reduce primal infeasibility
    if(adaptivePenalty_ && d_res > mu_ * p_res)
      rho_ = rho_ / tau_; // reduce penalty

    // log the graph after every iteration
    if (verbosity_ >= LOG){
      logResult(iter, p_res, d_res, time_duration);

    }

    logCount_++;

    // print debug info
    if(verbosity_ >= DEBUG){
      printf("p_res(iter) : %g (%d) < %g: min_p_res_ \n",p_res,iter,min_p_res_);
      printf("d_res(iter) : %g (%d) < %g: min_d_res_ \n",d_res,iter,min_d_res_);
      cout << "rho: " << rho_ << " (tau=" << tau_ << ", mu=" << mu_ << ")" << endl;
      if(iter == maxIter_-1)
        cout << "Reached maxIter: " << maxIter_-1 << endl;
    }

    //check stopping conditions
    if(p_res < min_p_res_ && d_res < min_d_res_){

      if(verbosity_ >= DEBUG){
        cout << "========================================" << endl;
        printf("p_res(iter) : %g (%d) < %g: min_p_res_ \n",p_res,iter,min_p_res_);
        printf("d_res(iter) : %g (%d) < %g: min_d_res_ \n",d_res,iter,min_d_res_);
      }

      // Break if stopping condition has reached
      break;
    }
  }

  if(verbosity_ >= DEBUG){
    // Stop ADMM Timer
    ADMM::Time admm_end = boost::posix_time::microsec_clock::local_time ();
    boost::posix_time::time_duration time_diff = admm_end - admm_start;
    cout <<  "Total time: " <<  time_diff.total_microseconds() << endl;
  }

  residualLog_->close();

  // unscale dual variables
  vector<double> y_k(uall_k.size(), 0.0f);
  for(size_t i = 0; i < y_k.size(); i++)
    y_k[i] =uall_k[i]*rho_; // unscaled dual variables

  // return ADMM primal and dual solution
  return make_pair(subinitials_, y_k);
}
