// ADMM
#include <ADMM.h>
#include <ADMMUtils.h>

// GTSAM
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <fstream>
#include <sstream>

using namespace std;
using namespace gtsam;

// To be used when you already have the partitioning (bunch of g2o files + separator file)
int main(int argc, char* argv[])
{

  int num_subgraphs;
  string output_dir; // output_dir
  string input_dir; // input dir

  double rho = 0.1; // penalty parameter in augmented lagrangian
  double mu = 10.0; // parameter used to decide whether to increase or not the rho (adaptive penalty)
  double tau = 2.0; // if we have to increase rho, we do rho <- rho * tau, otherwise rho <- rho / tau
  int maxIter = 1000; // max number of ADMM iterations
  bool isParallel = false; // if true ADMM runs in parallel mode, this is usually less stable
  bool adaptivePenalty = true;
  bool useFlaggedInitialization = false;
  bool computeSubgraphGradient = true;
  int solver = 0; // GN
  double min_p_res = 0.1f;
  double min_d_res = 0.1f;

  // Verbosity
  int verbosity = 0;

  try{
    // Parse program options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("input_dir,i",po::value<string>(&input_dir)->required(), "Input director (required)")
        ("num_subgraphs,n", po::value<int>(&num_subgraphs)->required(), "Num Subgraphs (required)")
        ("result_dir,r", po::value<string>(&output_dir)->required(), "Results Directory (required)")
        ("rho_val", po::value<double>(&rho), "rho value (default: 0.1)")
        ("mu_val,mu", po::value<double>(&mu), "mu value (default: 10.0)")
        ("tau_val,tau", po::value<double>(&tau), "tau value (default: 2.0)")
        ("max_iter", po::value<int>(&maxIter), "Maximum number of iterations (default: 1000)")
        ("adaptive_penalty,adpt_pen", po::value<bool>(&adaptivePenalty), "To use adaptive penalty or not (default: true)")
        ("use_flagged_init", po::value<bool>(&useFlaggedInitialization), "To use Flagged Initialization or not (default: true)")
        ("is_parallel", po::value<bool>(&isParallel), "To use parallel multiblock ADMM or not (default: false)")
        ("compute_subgraph_gradient", po::value<bool>(&computeSubgraphGradient), "To compute subgraph gradient  or not (default: true)")
        ("min_p_res", po::value<double>(&min_p_res), "min_p_res value (default: 0.01f)")
        ("min_d_res", po::value<double>(&min_d_res), "min_d_res value (default: 0.01f)")
        ("solver", po::value<int>(&solver), "Specify solver type (default: GN)")
        ("verbosity", po::value<int>(&verbosity), "Specify verbosity level (default: SILENT)");



    po::variables_map vm;
    try{
      po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
      if ( vm.count("help")  ){ // --help option
        cout << "Partition and Run ADMM" << endl << desc << endl;
        return 0;
      }
      po::notify(vm); // throws on error, so do after help in case
    }
    catch(po::error& e){
      cerr << "ERROR: " << e.what() << endl << endl;
      cerr << desc << endl;
      return 1;
    }
  }
  catch(exception& e){
    cerr << "Unhandled Exception reached the top of main: "
              << e.what() << ", application will now exit" << endl;
    return 2;
  }

  // Create the output directory if it does not exist
  if( !(boost::filesystem::exists(output_dir))){
    boost::filesystem::create_directory(output_dir);
  }

  // Subgraph and Subinitials
  vector<NonlinearFactorGraph> sub_graphs;
  vector<Values> sub_initials;
  PriorFactor<Pose2> posePrior;

  // Read subgraphs
  for(int sub_id = 0; sub_id < num_subgraphs; sub_id++)
  {
    string filepath = input_dir + boost::lexical_cast<string>(sub_id) + string(".g2o");
    GraphAndValues readGraph = readG2o(filepath);
    NonlinearFactorGraph graph = *(readGraph.first);
    Values initial = *(readGraph.second);
    if (sub_id == 0){

      posePrior = PriorFactor<Pose2>(0, initial.at<Pose2>(0),noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05,  5*M_PI/180)));
      graph.add(posePrior);
    }

    sub_graphs.push_back(graph);
    sub_initials.push_back(initial);
  }

  // Read separators
  string separator_file = input_dir + "separators.txt";
  vector < vector <string> > separators_str;
  ifstream infile;
  infile.open(separator_file.c_str(), ifstream::in);

  while (infile){
    string s;
    if (!getline( infile, s )) break;

    istringstream ss(s);
    vector <string> record;

    while (ss){
      string s;
      if (!getline( ss, s, ',' )) break;
      record.push_back(s);
    }

    separators_str.push_back( record );
  }
  if (!infile.eof()){
    cerr << "File still not read completely!\n";
  }

  // Load the separators
  vector <int>  separators;
  for(size_t i=0; i < separators_str.size(); i++){
    for(size_t j=0; j< separators_str[i].size(); j++){
      separators.push_back(boost::lexical_cast<int>(separators_str[i][j]));
    }
  }


  // Read centralized graph from g2o file
  string filepath = input_dir + "/fullGraph.g2o";
  GraphAndValues readGraph = readG2o(filepath);
  NonlinearFactorGraph graph = *(readGraph.first);
  Values initial = *(readGraph.second);


  // Write down the G2o files
  for(size_t sub_id =0; sub_id < sub_initials.size(); sub_id++){
    string filepath = output_dir + "/" + boost::lexical_cast<string>(sub_id) + "_initial.g2o";
    writeG2o(sub_graphs[sub_id], sub_initials[sub_id], filepath);
  }

  filepath = output_dir + "/separators.txt";
  fstream separatorFile(filepath.c_str(), fstream::out);
  for(size_t i = 0; i < separators.size(); i += 3){
    separatorFile << separators[i] <<  "," << separators[i+1] <<  "," << separators[i+2] << endl;
  }
  separatorFile.close();

  // Run ADMM
  // Construct ADMM
  ADMM admm(rho,
            mu,
            tau,
            maxIter,
            isParallel,
            adaptivePenalty,
            useFlaggedInitialization,
            computeSubgraphGradient,
            min_p_res,
            min_d_res,
            output_dir);

  admm.setSolver(ADMM::Solver(solver));
  admm.setVerbosity(ADMM::Verbosity(verbosity));

  // Load structures
  admm.load(sub_graphs, sub_initials, separators, graph);

  cout << "Running on " << input_dir << endl;
  // Run ADMM
  pair<vector<Values>, vector<double> > primalDualResult = admm.optimize();
  vector<Values> sub_results = primalDualResult.first;

  // Pack ADMM results into a single Value structure, to check centralized error
  logResults(sub_graphs, sub_results, graph, initial, posePrior, output_dir);


  return 0;
}
