#ifndef ADMM_H
#define ADMM_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/lago.h>

#include <boost/tuple/tuple.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <BiasedPriorFactor.h>

#include <fstream>
#include <sstream>

/** @brief The ADMM class */
class ADMM
{
  public:

    // Typedefs
    typedef boost::posix_time::ptime Time;

    /** @brief The Solver enum for switching between different Nonlinear solvers */
    enum Solver{
      GN, // Gauss Newton
      LM, // Levenberg Marquardt
      LAGO // LAGO
    };

    /** @brief Verbosity defines the verbosity levels */
    enum Verbosity{SILENT, // Print or output nothing
                   LOG, // Save traces
                   DEBUG // Also print debug statements
                  };

    /**
     * @brief ADMM
     * @param rho
     * @param mu
     * @param tau
     * @param maxIter
     * @param isParallel     
     * @param adaptivePenalty     
     * @param useFlaggedInitialization     
     * @param computeSubgraphGradient
     * @param min_p_res
     * @param min_d_res
     * @param outputDir
     */
    ADMM(double rho = 0.1,
         double mu = 10.0,
         double tau = 2.0,
         int maxIter = 1000,
         bool isParallel = false,         
         bool adaptivePenalty = true,         
         bool useFlaggedInitialization = true,         
         bool computeSubgraphGradient = true,
         double min_p_res = 0.01,
         double min_d_res = 0.01,
         std::string outputDir = std::string("../../results/"))
    {
      rho_ = rho;
      mu_ = mu;
      tau_ = tau;
      maxIter_ = maxIter;
      isParallel_ = isParallel;      
      adaptivePenalty_ = adaptivePenalty;      
      outputDir_ = outputDir;
      useFlaggedInitialization_ = useFlaggedInitialization;      
      solver_ = GN;
      computeSubgraphGradient_ = computeSubgraphGradient;
      min_p_res_ = min_p_res;
      min_d_res_ = min_d_res;
      verbosity_ = SILENT;
      residualLog_.reset(new std::fstream());
    }


    /**
     * @brief FlaggedInitialization is used for initializing all the subgraphs by propagating from the subgraph with the prior outwards in a breadth first manner
     * @param rho
     */
    void FlaggedInitialization();



    /**
     * @brief iterate
     * @param uall_k
     * @param subgraph_gradient
     * @param iter
     * @return
     */
    std::pair<double, double> iterate(int iter);


    /**
     * @brief Decentralized optimization using the Alternate Direction Method of Multipliers (ADMM)
     * @return return optimized solution, and a vector of dual variables
     */
    std::pair<std::vector<gtsam::Values>, std::vector<double> >
    optimize();


     /** @brief setSolver defines the ADMM solver */
    void setSolver(ADMM::Solver solver){solver_ = solver;}

    /** @brief setVerbosity sets the verbosity level */
    void setVerbosity(Verbosity verbosity){verbosity_ = verbosity;}

    /** @brief logResults aggregates the result and logs it in a file */
    void logResult(int iter, double p_res, double d_res, double time_taken);

    /**
     * @brief loads the subgraphs, subinitials and separators
     * @param subgraphs is the vector of subgraphs
     * @param subinitials is the vector of subinitials
     * @param separators is the separators, in the form <sepkey0, subgraph_id01, subgraph_id02, sepkey1, subgraph_id11, subgraph_id12, ...>
     * @param fullGraph is just for visualization purposes
     */
    void load(std::vector< gtsam::NonlinearFactorGraph > subgraphs,
                   std::vector<gtsam::Values> subinitials,
                   std::vector<int> separators,
                   gtsam::NonlinearFactorGraph fullGraph = gtsam::NonlinearFactorGraph()){
      subgraphs_ = subgraphs;
      subinitials_ = subinitials;
      separators_ = separators;
      fullGraph_ = fullGraph;
      uall_k = std::vector<double>(separators_.size(), 0.0f); // scaled dual variables: u_k = y_k/rho (scaling by 1/rho won't have any effect initially since y_k = 0)
    }


    /** @brief tic starts the timer */
    void tic(){
           start = boost::posix_time::microsec_clock::local_time ();
    }

    /** @brief toc finishes the timer and returns the duration  */
    double toc(){
      // Iter end timer
      end = boost::posix_time::microsec_clock::local_time ();
      boost::posix_time::time_duration time_diff = end - start;
      double time_duration = time_diff.total_microseconds();
      return time_duration;
    }

  protected:

    // Structures
    std::vector< gtsam::NonlinearFactorGraph > subgraphs_; // the vector of subgraphs
    std::vector<gtsam::Values> subinitials_; // the vector of subinitials
    std::vector<int> separators_; // separators in the form <sepkey0, subgraph_id01, subgraph_id02, sepkey1, subgraph_id11, subgraph_id12, ...>
    gtsam::NonlinearFactorGraph fullGraph_; // full graph for visualization purposes
    std::vector<double> uall_k; // scaled dual variables: u_k = y_k/rho (scaling by 1/rho won't have any effect initially since y_k = 0)

    // Optimizer config
    double rho_; // penalty parameter in augmented lagrangian
    double mu_;  // parameter used to decide by how much to increase or decrease rho
    double tau_; // if we have to increase rho, we do rho <- rho * tau, otherwise rho <- rho / tau
    int maxIter_; // max number of ADMM iterations
    bool isParallel_; // if true ADMM runs in parallel mode, this is usually less stable    
    bool adaptivePenalty_;  // parameter used to decide whether to increase or not the rho (adaptive penalty)
    bool useFlaggedInitialization_; // use flagged initialization    
    bool computeSubgraphGradient_; // to compute subgraph gradient

    ADMM::Solver solver_;
    ADMM::Time start,end;

    // Stopping Criterion
    double min_p_res_;
    double min_d_res_;    

    // Verbosity and Traces
    Verbosity verbosity_; // Verbosity level
    std::string logDir_; // Log directory
    std::string subgraphLogDir_;  // Subgraph log directory
    std::string outputDir_;
    boost::shared_ptr <std::fstream> residualLog_; // Filestrewam pointer to residual log
    size_t logCount_, logCountThresh_;


};
#endif
