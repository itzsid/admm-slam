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

bool isDebug = false;

using namespace std;
using namespace gtsam;


// Main script, takes a g2o file as input and the number of partitions, and does
// 1) (Edge) Partitioning of the graph using metis
// 2) Decentralized optimization using the Alternate Direction Method of Multipliers
int main(int argc, char* argv[])
{
  //////////////////////////////////////////////////////////////////////////////////////
  //Command line arguments
  //////////////////////////////////////////////////////////////////////////////////////
  size_t num_subgraphs; // input dir
  string input_g2o; // g2o file
  string output_dir; // output_dir

  // Optimizer Config
  double rho = 0.1; // penalty parameter in augmented lagrangian
  double mu = 10.0; // parameter used to decide whether to increase or not the rho (adaptive penalty)
  double tau = 2.0; // if we have to increase rho, we do rho <- rho * tau, otherwise rho <- rho / tau
  double min_p_res = 0.1f;
  double min_d_res = 0.1f;
  int maxIter = 1000; // max number of ADMM iterations
  int solver = 0; // GN
  bool isParallel = false; // if true ADMM runs in parallel mode, this is usually less stable
  bool adaptivePenalty = true;
  bool useFlaggedInitialization = false;
  bool computeSubgraphGradient = true;

  // Partitioning
  bool useMetisPartitioning = true;
  bool useLineGraph = true;
  bool orderSubgraphs = true;

  // Verbosity
  int verbosity = 0;

  try{
    // Parse program options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("input_g2o,i",po::value<string>(&input_g2o)->required(), "Input g2o file (required)")
        ("num_subgraphs,n", po::value<size_t>(&num_subgraphs)->required(), "Num Subgraphs (required)")
        ("result_dir,r", po::value<string>(&output_dir)->required(), "Results Directory (required)")
        ("rho_val", po::value<double>(&rho), "rho value (default: 0.1)")
        ("mu_val,mu", po::value<double>(&mu), "mu value (default: 10.0)")
        ("tau_val,tau", po::value<double>(&tau), "tau value (default: 2.0)")
        ("max_iter", po::value<int>(&maxIter), "Maximum number of iterations (default: 1000)")
        ("adaptive_penalty,adpt_pen", po::value<bool>(&adaptivePenalty), "To use adaptive penalty or not (default: true)")
        ("use_metis_partitioning", po::value<bool>(&useMetisPartitioning), "To use metis partiioning or not (default: true)")
        ("order_subgraphs", po::value<bool>(&orderSubgraphs), "To order subgraphs or not (default: true)")
        ("use_line_graph", po::value<bool>(&useLineGraph), "To use Line Graph or not (default: true)")
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

  cout << "Running on " << input_g2o << endl;
  //////////////////////////////////////////////////////////////////////////////////////
  // Read centralized graph from g2o file
  //////////////////////////////////////////////////////////////////////////////////////
  GraphAndValues readGraph = readG2o(input_g2o);
  NonlinearFactorGraph graph = *(readGraph.first);
  Values initial = *(readGraph.second);

  // g2o file comes without prior - the following prior must be included after using Metis
  PriorFactor<Pose2> posePrior =
      PriorFactor<Pose2>(0, initial.at<Pose2>(0),
                         noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05,  5*M_PI/180)));



  //////////////////////////////////////////////////////////////////////////////////////
  // (Vertex) Partitioning using metis. This can be useful for GTSAM, but ADMM needs *edge* partitioning
  //////////////////////////////////////////////////////////////////////////////////////
  vector<NonlinearFactorGraph> sub_graphs;
  vector<Values> sub_initials;
  boost::tie(sub_graphs, sub_initials) = partitionGraph(graph,
                                                        initial,
                                                        posePrior,
                                                        num_subgraphs,
                                                        useMetisPartitioning,
                                                        useLineGraph, orderSubgraphs);

  //////////////////////////////////////////////////////////////////////////////////////
  /// Creating Separator vector
  //////////////////////////////////////////////////////////////////////////////////////
  vector<int> separators = createSeparators(sub_initials, initial);


  // Write down the G2o files
  for(size_t sub_id =0; sub_id < sub_initials.size(); sub_id++){
    string filepath = output_dir + "/" + boost::lexical_cast<string>(sub_id) + "_initial.g2o";
    writeG2o(sub_graphs[sub_id], sub_initials[sub_id], filepath);
  }

  // Write down the separators
  string separatorFile = output_dir + "/separators.txt";
  fstream separatorFileStream(separatorFile.c_str(), fstream::out);
  for(size_t i = 0; i < separators.size(); i += 3){
    separatorFileStream << separators[i] <<  "," << separators[i+1] <<  "," << separators[i+2] << endl;
  }
  separatorFileStream.close();


  //////////////////////////////////////////////////////////////////////////////////////
  /// ADMM
  //////////////////////////////////////////////////////////////////////////////////////
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

  // Run ADMM
  pair<vector<Values>, vector<double> > primalDualResult;
  try{
    primalDualResult = admm.optimize();
  }
  catch (...){
    cout << "Trying with Odometry Partition" << endl;
    boost::tie(sub_graphs, sub_initials) = partitionGraph(graph,
                                                          initial,
                                                          posePrior,
                                                          num_subgraphs,
                                                          useMetisPartitioning,
                                                          useLineGraph, orderSubgraphs);
    vector<int> separators = createSeparators(sub_initials, initial);
    admm.load(sub_graphs, sub_initials, separators, graph);
    primalDualResult = admm.optimize();
  }

  vector<Values> sub_results = primalDualResult.first;

  //////////////////////////////////////////////////////////////////////////////////////
  /// Analysis
  //////////////////////////////////////////////////////////////////////////////////////
  logResults(sub_graphs, sub_results, graph, initial, posePrior, output_dir);

  return 0;
}
