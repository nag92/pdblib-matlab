# pbdlib-sandbox-matlab

PbDlib is a collection of source codes for robot programming by demonstration (learning from demonstration). It includes various functionalities at the crossroad of statistical learning, dynamical systems, optimal control and Riemannian geometry.<br>
PbDlib can be used in applications requiring task adaptation, human-robot skill transfer, safe controllers based on minimal intervention principle, as well as for probabilistic motion analysis and synthesis in multiple coordinate systems.<br>
The codes are compatible with both Matlab and GNU Octave. Other versions of the library in Python and C++ are also available at http://www.idiap.ch/software/pbdlib/ (currently, the Matlab version has the most functionalities).<br> 

### Usage

Examples starting with `demo_` can be run as examples. The corresponding publications related to these examples are listed below. 

### List of examples

All the examples are located in the main folder, and the functions are located in the `m_fcts` folder.

| Filename | Ref. | Description |
|----------|------|-------------|
| [benchmark/benchmark_DS_GP_GMM01.m](./demos/benchmark_DS_GP_GMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on Gaussian process regression, with trajectory model (Gaussian mixture model encoding), and DS-GMR used for reproduction |
| [benchmark/benchmark_DS_GP_raw01.m](./demos/benchmark_DS_GP_raw01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on Gaussian process regression, with raw trajectory, and spring-damper system used for reproduction |
| [benchmark/benchmark_DS_PGMM01.m](./demos/benchmark_DS_PGMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on parametric Gaussian mixture model, and DS-GMR used for reproduction |
| [benchmark/benchmark_DS_TP_GMM01.m](./demos/benchmark_DS_TP_GMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| [benchmark/benchmark_DS_TP_GP01.m](./demos/benchmark_DS_TP_GP01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian process (nonparametric task-parameterized method) |
| [benchmark/benchmark_DS_TP_LWR01.m](./demos/benchmark_DS_TP_LWR01.m) | [[1]](#ref-1) | Benchmark of task-parameterized locally weighted regression (nonparametric task-parameterized method) |
| [benchmark/benchmark_DS_TP_MFA01.m](./demos/benchmark_DS_TP_MFA01.m) | [[1]](#ref-1) | Benchmark of task-parameterized mixture of factor analyzers (TP-MFA), with DS-GMR used for reproduction |
| [benchmark/benchmark_DS_TP_trajGMM01.m](./demos/benchmark_DS_TP_trajGMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| [demo_affineTransform01.m](./demos/demo_affineTransform01.m) | [[1]](#ref-1) | Miscellaneous affine transformations of raw data as pre-processing step to train a task-parameterized model |
| [demo_AR01.m](./demos/demo_AR01.m) | [[]](#) | Multivariate autoregressive (AR) model parameters estimation with least-squares |
| [demo_AR_HSMM01.m](./demos/demo_AR_HSMM01.m) | [[]](#) | Multivariate autoregressive (AR) model implemented as a hidden semi-Markov model with lognormal duration model |
| [demo_Bezier01.m](./demos/demo_Bezier01.m) | [[5]](#ref-5) | Bezier curves as a superposition of Bernstein polynomials |
| [demo_Bezier02.m](./demos/demo_Bezier02.m) | [[5]](#ref-5) | Bezier curves fitting |
| [demo_Bezier_illustr01.m](./demos/demo_Bezier_illustr01.m) | [[5]](#ref-5) | Fitting Bezier curves of different degrees |
| [demo_Bezier_illustr02.m](./demos/demo_Bezier_illustr02.m) | [[5]](#ref-5) | Illustration of linear, quadratic and cubic Bezier curves |
| [demo_covariance01.m](./demos/demo_covariance01.m) | [[1]](#ref-1) | Covariance computation in matrix form |
| [demo_DMP01.m](./demos/demo_DMP01.m) | [[1]](#ref-1) | Dynamic movement primitive (DMP) encoding with radial basis functions |
| [demo_DMP02.m](./demos/demo_DMP02.m) | [[1]](#ref-1) | Generalization of dynamic movement primitive (DMP) with polynomial fitting using radial basis functions |
| [demo_DMP_batchLQR01.m](./demos/demo_DMP_batchLQR01.m) | [[1]](#ref-1) | Emulation of DMP with a spring system controlled by batch LQR |
| [demo_DMP_GMR01.m](./demos/demo_DMP_GMR01.m) | [[1]](#ref-1) | Emulation of a standard dynamic movement primitive (DMP) model by using a Gaussian mixture model (GMM) with diagonal covariance matrix, and retrieval computed through Gaussian mixture regression (GMR) |
| [demo_DMP_GMR02.m](./demos/demo_DMP_GMR02.m) | [[1]](#ref-1) | Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture model (GMM) representation, with full covariance matrices coordinating the different variables in the feature space, where after learning (i.e., autonomous organization of the basis functions (position and spread), Gaussian mixture regression (GMR) is used to regenerate the nonlinear force profile |
| [demo_DMP_GMR03.m](./demos/demo_DMP_GMR03.m) | [[1]](#ref-1) | Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture model (GMM) representation, with full covariance matrices coordinating the different variables in the feature space, where after learning (i.e., autonomous organization of the basis functions (position and spread), Gaussian mixture regression (GMR) is used to regenerate the path of a spring-damper system, resulting in a nonlinear force profile |
| [demo_DMP_GMR04.m](./demos/demo_DMP_GMR04.m) | [[1]](#ref-1) | Enhanced dynamic movement primitive (DMP) model trained with EM by using a Gaussian mixture model (GMM) representation, with full covariance matrices coordinating the different variables in the feature space, and by using the task-parameterized model formalism, where after learning (i.e., autonomous organization of the basis functions (position and spread), Gaussian mixture regression (GMR) is used to regenerate the path of a spring-damper system, resulting in a nonlinear force profile |
| [demo_DMP_GMR_illustr01.m](./demos/demo_DMP_GMR_illustr01.m) | [[1]](#ref-1) | Illustration of DMP with GMR to regenerate the nonlinear force profile | 
| [demo_DMP_GMR_LQR01.m](./demos/demo_DMP_GMR_LQR01.m) | [[1]](#ref-1) | Same example as demo_DMP_GMR04, but with LQR used to refine the parameters of the spring-damper system based on the covariance information retrieved by GMR |
| [demo_DPmeans_online01.m](./demos/demo_DPmeans_online01.m) | [[6]](#ref-6) | Online clustering with DP-Means algorithm |
| [demo_DPmeans_online02.m](./demos/demo_DPmeans_online02.m) | [[6]](#ref-6) | Online clustering with DP-Means algorithm, with stochastic samples |
| [demo_DTW01.m](./demos/demo_DTW01.m) | [[1]](#ref-1) | Trajectory realignment through dynamic time warping (DTW) |
| [demo_ergodicControl_1D01.m](./demos/demo_ergodicControl_1D01.m) | [[5]](#ref-5) | 1D ergodic control with spectral multiscale coverage (SMC) algorithm |
| [demo_ergodicControl_2D01.m](./demos/demo_ergodicControl_2D01.m) | [[5]](#ref-5) | 2D ergodic control with spectral multiscale coverage (SMC) algorithm |
| [demo_ergodicControl_3D01.m](./demos/demo_ergodicControl_3D01.m) | [[5]](#ref-5) | 3D ergodic control with spectral multiscale coverage (SMC) algorithm |
| [demo_ergodicControl_nD01.m](./demos/demo_ergodicControl_nD01.m) | [[5]](#ref-5) | nD ergodic control with spectral multiscale coverage (SMC) algorithm |
| [demo_Gaussian01.m](./demos/demo_Gaussian01.m) | [[5]](#ref-5) | Use of Chi-square values to determine the percentage of data within the contour of a multivariate normal distribution |
| [demo_Gaussian02.m](./demos/demo_Gaussian02.m) | [[5]](#ref-5) | Conditional probability with a multivariate normal distribution |
| [demo_Gaussian03.m](./demos/demo_Gaussian03.m) | [[5]](#ref-5) | Gaussian conditioning with uncertain inputs |
| [demo_Gaussian04.m](./demos/demo_Gaussian04.m) | [[5]](#ref-5) | Gaussian estimate of a GMM with the law of total covariance |
| [demo_Gaussian05.m](./demos/demo_Gaussian05.m) | [[5]](#ref-5) | Stochastic sampling with multivariate Gaussian distribution |
| [demo_Gaussian06.m](./demos/demo_Gaussian06.m) | [[5]](#ref-5) | Gaussian reformulated as zero-centered Gaussian with augmented covariance |
| [demo_Gaussian_illustr01.m](./demos/demo_Gaussian_illustr01.m) | [[5]](#ref-5) | Illustration of Gaussian conditioning with uncertain inputs |
| [demo_GaussProd4nullspace_2D01.m](./demos/demo_GaussProd4nullspace_2D01.m) | [[]](#) | 2D illustration of using a product of Gaussians to compute the hierarchy of two tasks |
| [demo_GaussProd4nullspace_3D01.m](./demos/demo_GaussProd4nullspace_3D01.m) | [[]](#) | 3D illustration of using a product of Gaussians to compute the hierarchy of three tasks |
| [demo_GaussProd_interp_illustr01.m](./demos/demo_GaussProd_interp_illustr01.m) | [[]](#) | Smooth transition between hierarchy constraints by relying on SPD geodesics |
| [demo_GMM01.m](./demos/demo_GMM01.m) | [[1]](#ref-1) | Gaussian mixture model (GMM) parameters estimation |
| [demo_GMM02.m](./demos/demo_GMM02.m) | [[1]](#ref-1) | GMM with different covariance structures |
| [demo_GMM_augmSigma01.m](./demos/demo_GMM_augmSigma01.m) | [[1]](#ref-1) | Gaussian mixture model (GMM) parameters estimation with zero means and augmented covariances |
| [demo_GMM_EM01.m](./demos/demo_GMM_EM01.m) | [[1]](#ref-1) | Problem of local optima in EM for GMM parameters estimation |
| [demo_GMM_HDDC01.m](./demos/demo_GMM_HDDC01.m) | [[1]](#ref-1) | High Dimensional Data Clustering (HDDC, or HD-GMM) model from Bouveyron (2007) |
| [demo_GMM_logGMM01.m](./demos/demo_GMM_logGMM01.m) | [[1]](#ref-1) | Multivariate lognormal mixture model parameters estimation with EM algorithm |
| [demo_GMM_logNormal01.m](./demos/demo_GMM_logNormal01.m) | [[]]() | Conditional probability with multivariate lognormal distribution |
| [demo_GMM_MFA01.m](./demos/demo_GMM_MFA01.m) | [[1]](#ref-1) | Mixture of factor analyzers (MFA) |
| [demo_GMM_MPPCA01.m](./demos/demo_GMM_MPPCA01.m) | [[1]](#ref-1) | Mixture of probabilistic principal component analyzers (MPPCA) |
| [demo_GMM_profileGMM01.m](./demos/demo_GMM_profileGMM01.m) | [[1]](#ref-1) | Univariate velocity profile fitting with a Gaussian mixture model (GMM) and a weighted EM algorithm, with an approach sharing links with radial basis function networks (RBFN), by adapting the Gaussian basis functions with EM based on the distribution profiles instead of considering a simple clustering of the input space |
| [demo_GMM_profileGMM_multivariate01.m](./demos/demo_GMM_profileGMM_multivariate01.m) | [[1]](#ref-1) | Multivariate velocity profile fitting with a Gaussian mixture model (GMM) and a weighted EM algorithm |
| [demo_GMM_profileLogGMM01.m](./demos/demo_GMM_profileLogGMM01.m) | [[1]](#ref-1) | Univariate velocity profile fitting with a lognormal mixture model (GMM) and a weighted EM algorithm |
| [demo_GMM_profileLogGMM_multivariate01.m](./demos/demo_GMM_profileLogGMM_multivariate01.m) | [[1]](#ref-1) | Multivariate velocity profile fitting with a Gaussian mixture model (GMM) and a weighted EM algorithm |
| [demo_GMM_semiTied01.m](./demos/demo_GMM_semiTied01.m) | [[1]](#ref-1) | Semi-tied Gaussian Mixture Model by tying the covariance matrices of a GMM with a set of common basis vectors |
| [demo_GMR01.m](./demos/demo_GMR01.m) | [[5]](#ref-5) | Gaussian mixture model (GMM) and time-based Gaussian mixture regression (GMR) used for reproduction |
| [demo_GMR02.m](./demos/demo_GMR02.m) | [[5]](#ref-5) | GMR computed with precision matrices instead of covariances |
| [demo_GMR03.m](./demos/demo_GMR03.m) | [[5]](#ref-5) | Chain rule with Gaussian conditioning |
| [demo_GMR_3Dviz01.m](./demos/demo_GMR_3Dviz01.m) | [[5]](#ref-5) | 3D visualization of a Gaussian mixture model (GMM) with time-based Gaussian mixture regression (GMR) used for reproduction |
| [demo_GMR_augmSigma01.m](./demos/demo_GMR_augmSigma01.m) | [[5]](#ref-5) | GMR with Gaussians reparamterized to have zero means and augmented covariances |
| [demo_GMR_DS01.m](./demos/demo_GMR_DS01.m) | [[2]](#ref-2) | Gaussian mixture model (GMM), with Gaussian mixture regression(GMR) and dynamical systems used for reproduction, with decay variable used as input (as in DMP) |
| [demo_GMR_polyFit01.m](./demos/demo_GMR_polyFit01.m) | [[5]](#ref-5) | Polynomial fitting with multivariate GMR |
| [demo_GMR_probTraj01.m](./demos/demo_GMR_probTraj01.m) | [[5]](#ref-5) | Probabilistic trajectory generation with GMR obtained from normally distributed GMM centers |
| [demo_GMR_SEDS01.m](./demos/demo_GMR_SEDS01.m) | [[2]](#ref-2) | Continuous autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| [demo_GMR_SEDS_augmSigma01.m](./demos/demo_GMR_SEDS_augmSigma01.m) | [[2]](#ref-2) | Continuous autonomous dynamical system with augmented state-space encoding and constrained optimization similar to the SEDS approach |
| [demo_GMR_SEDS_discrete01.m](./demos/demo_GMR_SEDS_discrete01.m) | [[2]](#ref-2) | Discrete autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| [demo_GMR_SEDS_discrete_augmSigma01.m](./demos/demo_GMR_SEDS_discrete_augmSigma01.m) | [[2]](#ref-2) | Discrete autonomous dynamical system with augmented state-space encoding and constrained optimization similar to the SEDS approach |
| [demo_GMR_wrapped01.m](./demos/demo_GMR_wrapped01.m) | [[5]](#ref-5) | Wrapped GMM and wrapped GMR in 2D (with only the first dimension being periodic) |
| [demo_GPR01.m](./demos/demo_GPR01.m) | [[2]](#ref-2) | Gaussian process regression (GPR) |
| [demo_GPR02.m](./demos/demo_GPR02.m) | [[2]](#ref-2) | GPR with stochastic samples from the prior and the posterior |
| [demo_GPR03.m](./demos/demo_GPR03.m) | [[2]](#ref-2) | GPR with periodic kernel function |
| [demo_GPR04.m](./demos/demo_GPR04.m) | [[2]](#ref-2) | GPR with Matern kernel function |
| [demo_GPR05.m](./demos/demo_GPR05.m) | [[2]](#ref-2) | GPR for motion generation with new targets |
| [demo_GPR_anim01.m](./demos/demo_GPR_anim01.m) | [[2]](#ref-2) | Gaussian process regression (GPR) with prior distribution formed so that it can be smoothly animated for illustration purpose |
| [demo_GPR_closedShape01.m](./demos/demo_GPR_closedShape01.m) | [[2]](#ref-2) | Closed shape modeling with Gaussian process regression (GPR) |
| [demo_GPR_closedShape02.m](./demos/demo_GPR_closedShape02.m) | [[2]](#ref-2) | Gaussian process implicit surface (GPIS) representation with thin-plate covariance function in GPR |
| [demo_GPR_GMR_illustr01.m](./demos/demo_GPR_GMR_illustr01.m) | [[]]() | Illustration of the different notions of variance for GPR and GMR |
| [demo_GPR_linTrend01.m](./demos/demo_GPR_linTrend01.m) | [[2]](#ref-2) | Gaussian process regression (GPR) with linear trend |
| [demo_GPR_paramOptim01.m](./demos/demo_GPR_paramOptim01.m) | [[]]() | GPR with optimization of the kernel parameters |
| [demo_GPR_recursive01.m](./demos/demo_GPR_recursive01.m) | [[]]() | Recursive computation of Gaussian process regression (GPR) |
| [demo_GPR_TP01.m](./demos/demo_GPR_TP01.m) | [[1]](#ref-1) | Use of GPR as a task-parameterized model, with DS-GMR used to retrieve continuous movements |
| [demo_grabData01.m](./demos/demo_grabData01.m) | [[1]](#ref-1) | Collect movement data from mouse cursor |
| [demo_gradientDescent01.m](./demos/demo_gradientDescent01.m) | [[]](#) | Optimization with gradient descent for 1D input (Newton's method) |
| [demo_gradientDescent02.m](./demos/demo_gradientDescent02.m) | [[]](#) | Optimization with gradient descent for 2D input (Newton's method) |
| [demo_gradientDescent03.m](./demos/demo_gradientDescent03.m) | [[]](#) | Optimization with gradient descent for 1D input and 2D output (Gauss-Newton algorithm) |
| [demo_gradientDescent04.m](./demos/demo_gradientDescent04.m) | [[]](#) | Optimization with gradient descent for 2D input and 2D output depicting a planar robot IK problem (Gauss-Newton algorithm) |
| [demo_HMM01.m](./demos/demo_HMM01.m) | [[2]](#ref-2) | Hidden Markov model (HMM) with single Gaussian as emission distribution |
| [demo_HMM02.m](./demos/demo_HMM02.m) | [[2]](#ref-2) | Emulation of HSMM with a standard HMM |
| [demo_HMM_Viterbi01.m](./demos/demo_HMM_Viterbi01.m) | [[2]](#ref-2) | Viterbi decoding in HMM to estimate best state sequence from observations |
| [demo_HSMM01.m](./demos/demo_HSMM01.m) | [[2]](#ref-2) | Variable duration model implemented as a hidden semi-Markov model (HSMM), by encoding the state duration after EM |
| [demo_HSMM02.m](./demos/demo_HSMM02.m) | [[2]](#ref-2) | Same as demo_HSMM01.m but with a log-normal duration model |
| [demo_HSMM_adaptiveDuration01.m](./demos/demo_HSMM_adaptiveDuration01.m) | [[9]](#ref-9) | Hidden semi-Markov model with adaptive duration, used with a controller based on discrete finite-horizon LQR updated online (with position and velocity tracking) |
| [demo_HSMM_adaptiveDuration_infHor01.m](./demos/demo_HSMM_adaptiveDuration_infHor01.m) | [[9]](#ref-9) | Hidden semi-Markov model with adaptive duration, used with a controller based on discrete infinite-horizon LQR (with position and velocity tracking) |
| [demo_HSMM_MPC01.m](./demos/demo_HSMM_MPC01.m) | [[2]](#ref-2) | Use of HSMM (with lognormal duration model) and batch LQR (with position only) for motion synthesis |
| [demo_HSMM_online01.m](./demos/demo_HSMM_online01.m) | [[2]](#ref-2) | Online HSMM |
| [demo_ID01.m](./demos/demo_ID01.m) | [[1]](#ref-1) | Inverse dynamics with dynamically consistent nullspace (requires the robotics toolbox) |
| [demo_IK01.m](./demos/demo_IK01.m) | [[1]](#ref-1) | Inverse kinematics with nullspace control (requires the robotics toolbox) |
| [demo_IK02.m](./demos/demo_IK02.m) | [[1]](#ref-1) | Inverse kinematics with two arms and nullspace control (requires the robotics toolbox) |
| [demo_IK_nullspaceAsProduct01.m](./demos/demo_IK_nullspaceAsProduct01.m) | [[]](#) | 3-level nullspace control formulated as product of Gaussians |
| [demo_IK_nullspace_TPGMM01.m](./demos/demo_IK_nullspace_TPGMM01.m) | [[1]](#ref-1) | IK with nullspace treated with task-parameterized GMM (bimanual tracking task, version with 4 frames) |
| [demo_IK_pointing_TPGMM01.m](./demos/demo_IK_pointing_TPGMM01.m) | [[1]](#ref-1) | Task-parameterized GMM to encode pointing direction by considering inverse kinematics (example with one object, requires the robotics toolbox) |
| [demo_IK_quat01.m](./demos/demo_IK_quat01.m) | [[1]](#ref-1) | Inverse kinematics for orientation data with quaternion references (version with quaternion Jacobian) |
| [demo_IK_quat02.m](./demos/demo_IK_quat02.m) | [[1]](#ref-1) | Inverse kinematics for orientation data with quaternion references (version with standard Jacobian) |
| [demo_IK_weighted01.m](./demos/demo_IK_weighted01.m) | [[]](#) | Inverse kinematics with nullspace control, by considering weights in joint space and in task space |
| [demo_ILC01.m](./demos/demo_ILC01.m) | [[1]](#ref-1) | Iterative correction of errors for a recurring movement with ILC |
| [demo_IPRA01.m](./demos/demo_IPRA01.m) | [[1]](#ref-1) | Gaussian mixture model (GMM) learned with iterative pairwise replacement algorithm (IPRA) |
| [demo_kernelPCA01.m](./demos/demo_kernelPCA01.m) | [[]](#) | Kernel PCA, with comparison to PCA |
| [demo_LS01.m](./demos/demo_LS01.m) | [[1]](#ref-1) | Multivariate ordinary least squares |
| [demo_LS_IRLS01.m](./demos/demo_LS_IRLS01.m) | [[]](#) | Iteratively reweighted least squares |
| [demo_LS_IRLS_logisticRegression01.m](./demos/demo_LS_IRLS_logisticRegression01.m) | [[]](#) | Logistic regression computed with iteratively reweighted least squares (IRLS) algorithm |
| [demo_LS_IRLS_logisticRegression02.m](./demos/demo_LS_IRLS_logisticRegression02.m) | [[]](#) | Logistic regression with multivariate inputs computed with IRLS algorithm |
| [demo_LS_polFit01.m](./demos/demo_LS_polFit01.m) | [[1]](#ref-1) | Polynomial fitting with least squares |
| [demo_LS_recursive01.m](./demos/demo_LS_recursive01.m) | [[1]](#ref-1) | Recursive computation of least squares estimate (implementation with block data) |
| [demo_LS_recursive02.m](./demos/demo_LS_recursive02.m) | [[1]](#ref-1) | Recursive computation of least squares estimate with one datapoint at a time |
| [demo_LS_weighted01.m](./demos/demo_LS_weighted01.m) | [[1]](#ref-1) | Weighted least squares regression |
| [demo_LWR01.m](./demos/demo_LWR01.m) | [[5]](#ref-5) | Locally weighted regression (LWR) with radial basis functions and local polynomial fitting |
| [demo_manipulabilityTracking_mainTask01.m](./demos/demo_manipulabilityControl_mainTask01.m) | [[10]](#ref-10) | Tracking of a desired manipulability ellipsoid as the main task |
| [demo_manipulabilityTracking_mainTask02.m](./demos/demo_manipulabilityControl_mainTask02.m) | [[10]](#ref-10) | Tracking of a desired manipulability ellipsoid as the main task using precision matrices as gain |
| [demo_manipulabilityTracking_secondaryTask01.m](./demos/demo_manipulabilityControl_secondTask01.m) | [[10]](#ref-10) | Tracking of a desired manipulability ellipsoid as the secondary task with position tracking as main task |
| [demo_manipulabilityTransfer01.m](./demos/demo_manipulabilityTransfer01.m) | [[7]](#ref-7) | Use of robot redundancy to track desired manipulability ellipsoid |
| [demo_manipulabilityTransfer02.m](./demos/demo_manipulabilityTransfer02.m) | [[7]](#ref-7) | Learning and reproduction of manipulability ellipsoid profiles |
| [demo_manipulabilityTransfer03.m](./demos/demo_manipulabilityTransfer03.m) | [[7]](#ref-7) | Learning and reproduction of manipulability ellipsoid profiles (numerical version) |
| [demo_MPC01.m](./demos/demo_MPC01.m) | [[2]](#ref-2) | Batch solution of linear quadratic optimal control (unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data (see also demo_MPC_iterativeLQR01.m) |
| [demo_MPC02.m](./demos/demo_MPC02.m) | [[2]](#ref-2) | Same as demo_MPC01 but with a GMM encoding of only position data |
| [demo_MPC03.m](./demos/demo_MPC03.m) | [[2]](#ref-2) | Control of a spring attached to a point with batch LQR |
| [demo_MPC04.m](./demos/demo_MPC04.m) | [[2]](#ref-2) | Control of a spring attached to a point with batch LQR (with augmented state space) |
| [demo_MPC_augmSigma01.m](./demos/demo_MPC_augmSigma01.m) | [[2]](#ref-2) | Batch LQR with augmented covariance to transform a tracking problem to a regulation problem |
| [demo_MPC_constrained01.m](./demos/demo_MPC_constrained01.m) | [[2]](#ref-2) | Constrained batch LQT by using quadratic programming solver, with an encoding of position and velocity data |
| [demo_MPC_fullQ01.m](./demos/demo_MPC_fullQ01.m) | [[2]](#ref-2) | Batch LQR exploiting full Q matrix by constraining the motion to pass through a common point at different time steps |
| [demo_MPC_fullQ02.m](./demos/demo_MPC_fullQ02.m) | [[2]](#ref-2) | Batch LQR exploiting full Q matrix by constraining the motion of two agents |
| [demo_MPC_fullQ03.m](./demos/demo_MPC_fullQ03.m) | [[2]](#ref-2) | Batch LQR exploiting full Q matrix by constraining the motion of two agents in a ballistic task |
| [demo_MPC_infHor01.m](./demos/demo_MPC_infHor01.m) | [[2]](#ref-2) | Discrete infinite horizon linear quadratic regulation (with precision matrix only on position) |
| [demo_MPC_infHor02.m](./demos/demo_MPC_infHor02.m) | [[2]](#ref-2) | Discrete infinite horizon linear quadratic regulation (with precision matrix on position and velocity) |
| [demo_MPC_infHor03.m](./demos/demo_MPC_infHor03.m) | [[2]](#ref-2) | Continuous infinite horizon linear quadratic tracking, by relying on a GMM encoding of position and velocity data |
| [demo_MPC_infHor04.m](./demos/demo_MPC_infHor04.m) | [[2]](#ref-2) | Discrete infinite horizon linear quadratic tracking, by relying on a GMM encoding of position and velocity data |
| [demo_MPC_infHor_bicopter01.m](./demos/demo_MPC_infHor_bicopter01.m) | [[2]](#ref-2) | Infinite horizon LQR applied on a planar UAV with an iterative linearization of the system plant |
| [demo_MPC_infHor_incremental01.m](./demos/demo_MPC_infHor_incremental01.m) | [[2]](ref-2) | Infinite horizon LQR with an iterative re-estimation of the system plant linear system (example with planar UAV) |
| [demo_MPC_infHor_incremental02.m](./demos/demo_MPC_infHor_incremental02.m) | [[2]](ref-2) | Infinite horizon LQR with an iterative re-estimation of the system plant linear system (example with torque-limited pendulum) |
| [demo_MPC_iterativeLQR01.m](./demos/demo_MPC_iterativeLQR01.m) | [[2]](#ref-2) | Iterative computation of linear quadratic tracking (with feedback and feedforward terms) |
| [demo_MPC_iterativeLQR02.m](./demos/demo_MPC_iterativeLQR02.m) | [[2]](#ref-2) | Same as demo_MPC_iterativeLQR01.m, by relying on a GMM encoding of position and velocity data, including comparison with batch LQT |
| [demo_MPC_iterativeLQR03.m](./demos/demo_MPC_iterativeLQR03.m) | [[2]](#ref-2) | Same as demo_MPC_iterativeLQR01.m, by relying on a GMM encoding of only position data |
| [demo_MPC_iterativeLQR04.m](./demos/demo_MPC_iterativeLQR04.m) | [[2]](#ref-2) | Control of a spring attached to a point with iterative linear quadratic tracking (with feedback and feedforward terms) |
| [demo_MPC_iterativeLQR_augmSigma01.m](./demos/demo_MPC_iterativeLQR_augmSigma01.m) | [[2]](#ref-2) | Iterative LQR with augmented covariance to transform the tracking problem to a regulation problem |
| [demo_MPC_iterativeLQR_augmSigma_online01.m](./demos/demo_MPC_iterativeLQR_augmSigma_online01.m) | [[2]](#ref-2) | Same as demo_iterativeLQR_augmSigma01 but recomputed in an online manner |
| [demo_MPC_Lagrangian01.m](./demos/demo_MPC_Lagrangian01.m) | [[2]](#ref-2) | Batch LQR with Lagrangian in matrix form to force first and last point to coincide in order to form periodic motion |
| [demo_MPC_MP01.m](./demos/demo_MPC_MP01.m) | [[2]](#ref-2) | Batch LQR using sparse movement primitives with radial basis functions |
| [demo_MPC_MP02.m](./demos/demo_MPC_MP02.m) | [[2]](#ref-2) | Batch LQR using sparse movement primitives with Fourier basis functions |
| [demo_MPC_noInitialState01.m](./demos/demo_MPC_noInitialState01.m) | [[2]](#ref-2) | Batch LQR solution finding the optimal initial state together with the optimal control commands |
| [demo_MPC_nullspace01.m](./demos/demo_MPC_nullspace01.m) | [[2]](#ref-2) | Batch LQR with nullspace formulation |
| [demo_MPC_nullspace_online01.m](./demos/demo_MPC_nullspace_online01.m) | [[2]](#ref-2) | Online batch LQR with nullspace formulation |
| [demo_MPC_online01.m](./demos/demo_MPC_online01.m) | [[2]](#ref-2) | MPC recomputed in an online manner with a time horizon |
| [demo_MPC_online02.m](./demos/demo_MPC_online02.m) | [[2]](#ref-2) | MPC recomputed in an online manner with a time horizon, by relying on a GMM encoding of position and velocity data (with animation) |
| [demo_MPC_online03.m](./demos/demo_MPC_online03.m) | [[2]](#ref-2) | Obstacle avoidance with MPC recomputed in an online manner |
| [demo_MPC_robot01.m](./demos/demo_MPC_robot01.m) | [[2]](#ref-2) | Robot tracking task with online batch LQR, by linearization of the system plant at each time step |
| [demo_MPC_skillsRepr01.m](./demos/demo_MPC_skillsRepr01.m) | [[2]](#ref-2) | Representation of skills combined in parallel and in series through a batch LQT formulation |
| [demo_MPC_viapoints01.m](./demos/demo_MPC_viapoints01.m) | [[2]](#ref-2) | Keypoint-based motion through MPC, with a GMM encoding of position and velocity |
| [demo_MPC_viapoints02.m](./demos/demo_MPC_viapoints02.m) | [[2]](#ref-2) | Same as demo_MPC_viapoints01 with only position encoding |
| [demo_MPC_viapoints03.m](./demos/demo_MPC_viapoints03.m) | [[2]](#ref-2) | Equivalence between cubic Bezier curve and batch LQR with double integrator |
| [demo_MPC_viapoints_withProd01.m](./demos/demo_MPC_viapoints_withProd01.m) | [[2]](#ref-2) | Batch LQT with viapoints computed as a product of trajectory distributions in control space |
| [demo_PCA01.m](./demos/demo_PCA01.m) | [[1]](#ref-1) | Principal component analysis (PCA) |
| [demo_Procrustes01.m](./demos/demo_Procrustes01.m) | [[]](#) | SVD solution of orthogonal Procrustes problem |
| [demo_proMP01.m](./demos/demo_proMP01.m) | [[5]](#ref-5) | Conditioning on trajectory distributions with Probabilistic movement primitives to estimate trajectory distribution|
| [demo_proMP_Fourier01.m](./demos/demo_proMP_Fourier01.m) | [[]](#) | ProMP with Fourier basis functions (1D example) |
| [demo_proMP_Fourier02.m](./demos/demo_proMP_Fourier02.m) | [[]](#) | ProMP with Fourier basis functions (2D example) |
| [demo_proMP_Fourier_sampling01.m](./demos/demo_proMP_Fourier_sampling01.m) | [[]](#) | Stochastic sampling with Fourier movement primitives (1D example) |
| [demo_proMP_Fourier_sampling02.m](./demos/demo_proMP_Fourier_sampling02.m) | [[]](#) | Stochastic sampling with Fourier movement primitives (2D example) |
| [demo_regularization01.m](./demos/demo_regularization01.m) | [[1]](#ref-1) | Regularization of GMM parameters with minimum admissible eigenvalue |
| [demo_regularization02.m](./demos/demo_regularization02.m) | [[1]](#ref-1) | Regularization of GMM parameters with the addition of a small circular covariance |
| [demo_Riemannian_Gdp_vecTransp01.m](./demos/demo_Riemannian_Gdp_vecTransp01.m) | [[4]](#ref-4) | Parallel transport on Grassmann manifold |
| [demo_Riemannian_Hd_GMM01.m](./demos/demo_Riemannian_Hd_GMM01.m) | [[4]](#ref-3) | GMM on d-hyperboloid manifold |
| [demo_Riemannian_Hd_interp01.m](./demos/demo_Riemannian_Hd_interp01.m) | [[3]](#ref-3) | Interpolation on d-hyperboloid manifold |
| [demo_Riemannian_pose_GMM01.m](./demos/demo_Riemannian_pose_GMM01.m) | [[3]](#ref-3) | GMM to encode 3D position and orientation as unit quaternion by relying on Riemannian manifold |
| [demo_Riemannian_S1_interp01.m](./demos/demo_Riemannian_S1_interp01.m) | [[]](#) | Interpolation on 1-sphere manifold (formulation with imaginary numbers) |
| [demo_Riemannian_S1_interp02.m](./demos/demo_Riemannian_S1_interp02.m) | [[]](#) | Interpolation on 1-sphere manifold (formulation with imaginary numbers, parameterization of x as angle) |
| [demo_Riemannian_S2_batchLQR01.m](./demos/demo_Riemannian_S2_batchLQR01.m) | [[]](#) | Linear quadratic tracking on a sphere by relying on Riemannian manifold and batch LQR recomputed in an online manner, based on GMM encoding of movement |
| [demo_Riemannian_S2_batchLQR02.m](./demos/demo_Riemannian_S2_batchLQR02.m) | [[]](#) | LQT on a sphere by relying on Riemannian manifold (recomputed in an online manner), based on GMM encoding of movement, by using only position data (-> velocity commands) |
| [demo_Riemannian_S2_batchLQR03.m](./demos/demo_Riemannian_S2_batchLQR03.m) | [[]](#) | LQT on a sphere by relying on Riemannian manifold (recomputed in an online manner), by using only position data (-> velocity commands) |
| [demo_Riemannian_S2_batchLQR_Bezier01.m](./demos/demo_Riemannian_S2_batchLQR_Bezier01.m) | [[8]](#ref-8) | Bezier interpolation on a sphere by relying on Riemannian manifold and batch LQR |
| [demo_Riemannian_S2_GaussProd01.m](./demos/demo_Riemannian_S2_GaussProd01.m) | [[3]](#ref-3) | Product of Gaussians on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_S2_GMM01.m](./demos/demo_Riemannian_S2_GMM01.m) | [[3]](#ref-3) | GMM for data on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_S2_GMR01.m](./demos/demo_Riemannian_S2_GMR01.m) | [[3]](#ref-3) | GMR with input and output data on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_S2_GMR02.m](./demos/demo_Riemannian_S2_GMR02.m) | [[3]](#ref-3) | GMR with time as input and spherical data as output by relying on Riemannian manifold |
| [demo_Riemannian_S2_GMR03.m](./demos/demo_Riemannian_S2_GMR03.m) | [[3]](#ref-3) | GMR with 3D Euclidean data as input and spherical data as output by relying on Riemannian manifold |
| [demo_Riemannian_S2_GMR04.m](./demos/demo_Riemannian_S2_GMR04.m) | [[3]](#ref-3) | GMR with input data on a sphere and output data in Eudlidean space by relying on Riemannian manifold |
| [demo_Riemannian_S2_infHorLQR01.m](./demos/demo_Riemannian_S2_infHorLQR01.m) | [[3]](#ref-3) | Linear quadratic regulation on a sphere by relying on Riemannian manifold and infinite-horizon LQR |
| [demo_Riemannian_S2_TPGMM01.m](./demos/demo_Riemannian_S2_TPGMM01.m) | [[3]](#ref-3) | TP-GMM for data on a sphere by relying on Riemannian manifold (with single frame) |
| [demo_Riemannian_S2_TPGMM02.m](./demos/demo_Riemannian_S2_TPGMM02.m) | [[3]](#ref-3) | TP-GMM for data on a sphere by relying on Riemannian manifold (with two frames) |
| [demo_Riemannian_S2_vecTransp01.m](./demos/demo_Riemannian_S2_vecTransp01.m) | [[3]](#ref-3) | Parallel transport on a sphere |
| [demo_Riemannian_S3_GMM01.m](./demos/demo_Riemannian_S3_GMM01.m) | [[3]](#ref-3) | GMM for unit quaternion data by relying on Riemannian manifold |
| [demo_Riemannian_S3_GMR01.m](./demos/demo_Riemannian_S3_GMR01.m) | [[3]](#ref-3) | GMR with unit quaternions as input and output data by relying on Riemannian manifold |
| [demo_Riemannian_S3_GMR02.m](./demos/demo_Riemannian_S3_GMR02.m) | [[3]](#ref-3) | GMR with time as input and unit quaternion as output by relying on Riemannian manifold |
| [demo_Riemannian_S3_infHorLQR01.m](./demos/demo_Riemannian_S3_infHorLQR01.m) | [[3]](#ref-3) | Linear quadratic regulation of unit quaternions by relying on Riemannian manifold and infinite-horizon LQR |
| [demo_Riemannian_S3_interp01.m](./demos/demo_Riemannian_S3_interp01.m) | [[3]](#ref-3) | Interpolation of unit quaternions by relying on Riemannian manifold, providing the same result as SLERP interpolation |
| [demo_Riemannian_S3_vecTransp01.m](./demos/demo_Riemannian_S3_vecTransp01.m) | [[3]](#ref-3) | Parallel transport for unit quaternions |
| [demo_Riemannian_Sd_GaussProd01.m](./demos/demo_Riemannian_Sd_GaussProd01.m) | [[3]](#ref-3) | Product of Gaussians on a d-sphere by relying on Riemannian manifold |
| [demo_Riemannian_Sd_GMM01.m](./demos/demo_Riemannian_Sd_GMM01.m) | [[3]](#ref-3) | GMM for data on a 1-sphere (circle) by relying on Riemannian manifold (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_Sd_GMM02.m](./demos/demo_Riemannian_Sd_GMM02.m) | [[3]](#ref-3) | GMM for data on a 2-sphere (ball) by relying on Riemannian manifold (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_Sd_GMMR01.m](./demos/demo_Riemannian_Sd_GMR01.m) | [[3]](#ref-3) | Retrieval of periodic motion with GMR, by considering input data on a 1-sphere and output data in Euclidean space |
| [demo_Riemannian_Sd_GMMR02.m](./demos/demo_Riemannian_Sd_GMR02.m) | [[3]](#ref-3) | GMR with input data on a 2-sphere (ball) and output data in Euclidean space by relying on Riemannian manifold |
| [demo_Riemannian_Sd_interp01.m](./demos/demo_Riemannian_Sd_interp01.m) | [[3]](#ref-3) | Interpolation on a n-sphere (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_Sd_interp02.m](./demos/demo_Riemannian_Sd_interp02.m) | [[3]](#ref-3) | Interpolation on a 3-sphere and comparison with SLERP (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_Sd_MPC01.m](./demos/demo_Riemannian_Sd_MPC01.m) | [[3]](#ref-3) | Linear quadratic tracking on Sn by relying on Riemannian manifold and batch LQR recomputed in an online manner, based on GMM encoding of movement |
| [demo_Riemannian_Sd_MPC_infHor01.m](./demos/demo_Riemannian_Sd_MPC_infHor01.m) | [[3]](#ref-3) | Linear quadratic regulation on S3 by relying on Riemannian manifold and infinite-horizon LQR |
| [demo_Riemannian_Sd_vecTransp01.m](./demos/demo_Riemannian_Sd_vecTransp01.m) | [[3]](#ref-3) | Parallel transport on a n-sphere (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_Sd_vecTransp02.m](./demos/demo_Riemannian_Sd_vecTransp02.m) | [[3]](#ref-3) | Vector transport on a n-sphere using Schild's ladder algorithm (formulation with tangent space of same dimension as manifold) |
| [demo_Riemannian_SE2_GMM01.m](./demos/demo_Riemannian_SE2_GMM01.m) | [[3]](#ref-3) | GMM on SE(2) manifold |
| [demo_Riemannian_SE2_interp01.m](./demos/demo_Riemannian_SE2_interp01.m) | [[3]](#ref-3) | Interpolation on SE(2) manifold |
| [demo_Riemannian_SOd_interp01.m](./demos/demo_Riemannian_SOd_interp01.m) | [[3]](#ref-3) | Interpolation on SO(d) manifold |
| [demo_Riemannian_SPD_GMM01.m](./demos/demo_Riemannian_SPD_GMM01.m) | [[4]](#ref-4) | GMM for covariance data by relying on Riemannian manifold |
| [demo_Riemannian_SPD_GMM_tensor01.m](./demos/demo_Riemannian_SPD_GMM_tensor01.m) | [[4]](#ref-4) | GMM for covariance data by relying on Riemannian manifold |
| [demo_Riemannian_SPD_GMR01.m](./demos/demo_Riemannian_SPD_GMR01.m) | [[4]](#ref-4) | GMR with time as input and covariance data as output by relying on Riemannian manifold |
| [demo_Riemannian_SPD_GMR02.m](./demos/demo_Riemannian_SPD_GMR02.m) | [[4]](#ref-4) | GMR with time as input and position vector as output with comparison between computation in vector and matrix forms |
| [demo_Riemannian_SPD_GMR03.m](./demos/demo_Riemannian_SPD_GMR03.m) | [[4]](#ref-4) | GMR with vector as input and covariance data as output by relying on Riemannian manifold |
| [demo_Riemannian_SPD_GMR_tensor01.m](./demos/demo_Riemannian_SPD_GMR_tensor01.m) | [[4]](#ref-4) | GMR with time as input and covariance data as output by relying on Riemannian manifold |
| [demo_Riemannian_SPD_interp01.m](./demos/demo_Riemannian_SPD_interp01.m) | [[4]](#ref-4) | Covariance interpolation on Riemannian manifold (comparison with linear interpolation, with Euclidean interpolation on Cholesky decomposition, and with Wasserstein interpolation) |
| [demo_Riemannian_SPD_interp02.m](./demos/demo_Riemannian_SPD_interp02.m) | [[4]](#ref-4) | Covariance interpolation on Riemannian manifold from a GMM with augmented covariances |
| [demo_Riemannian_SPD_interp03.m](./demos/demo_Riemannian_SPD_interp03.m) | [[4]](#ref-4) | Trajectory morphing through covariance interpolation on Riemannian manifold (with augmented Gaussian trajectory distribution) |
| [demo_Riemannian_SPD_vecTransp01.m](./demos/demo_Riemannian_SPD_vecTransp01.m) | [[4]](#ref-4) | Verification of angle conservation in parallel transport on the symmetric positive definite |
| [demo_Riemannian_SPD_vecTransp02.m](./demos/demo_Riemannian_SPD_vecTransp02.m) | [[4]](#ref-4) | Vector transport on the Symmetric Positive Definite matrices (SPD) manifold using Schild's ladder algorithm |
| [demo_search01.m](./demos/demo_search01.m) | [[1]](#ref-1) | EM-based stochastic optimization |
| [demo_spring01.m](./demos/demo_spring01.m) | [[1]](#ref-1) | Influence of the damping ratio in mass-spring-damper systems |
| [demo_stdPGMM01.m](./demos/demo_stdPGMM01.m) | [[1]](#ref-1) | Parametric Gaussian mixture model (PGMM) used as a task-parameterized model, with DS-GMR employed to retrieve continuous movements |
| [demo_TPGMM01.m](./demos/demo_TPGMM01.m) | [[1]](#ref-1) | Task-parameterized Gaussian mixture model (TP-GMM) encoding |
| [demo_TPGMM_bimanualReaching01.m](./demos/demo_TPGMM_bimanualReaching01.m) | [[1]](#ref-1) | Time-invariant task-parameterized GMM applied to a bimanual reaching task |
| [demo_TPGMM_teleoperation01.m](./demos/demo_TPGMM_teleoperation01.m) | [[1]](#ref-1) | Time-invariant task-parameterized GMM applied to a teleoperation task (position only) |
| [demo_TPGMR01.m](./demos/demo_TPGMR01.m) | [[1]](#ref-1) | Task-parameterized Gaussian mixture model (TP-GMM), with GMR used for reproduction (without dynamical system) |
| [demo_TPGP01.m](./demos/demo_TPGP01.m) | [[1]](#ref-1) | Task-parameterized Gaussian process regression (TP-GPR) |
| [demo_TPMPC01.m](./demos/demo_TPMPC01.m) | [[1]](#ref-1) | Task-parameterized probabilistic model encoding position data, with MPC (batch version of LQR) used to track the associated stepwise reference path |
| [demo_TPproMP01.m](./demos/demo_TPproMP01.m) | [[1]](#ref-1) | Task-parameterized probabilistic movement primitives (TP-ProMP) |
| [demo_TPtrajDistrib01.m](./demos/demo_TPtrajDistrib01.m) | [[1]](#ref-1) | Task-parameterized model with trajectory distribution and eigendecomposition |
| [demo_TPtrajGMM01.m](./demos/demo_TPtrajGMM01.m) | [[1]](#ref-1) | Task-parameterized model with trajectory-GMM encoding |
| [demo_trajDistrib01.m](./demos/demo_trajDistrib01.m) | [[1]](#ref-1) | Stochastic sampling with Gaussian trajectory distribution |
| [demo_trajDistrib_differencingMatrix01.m](./demos/demo_trajDistrib_differencingMatrix01.m) | [[]]() | Conditioning on trajectory distribution constructed by differencing matrix, with via-point passing example |
| [demo_trajGMM01.m](./demos/demo_trajGMM01.m) | [[1]](#ref-1) | Trajectory synthesis using a GMM with dynamic features (trajectory GMM) |
| [demo_trajGMM02.m](./demos/demo_trajGMM02.m) | [[1]](#ref-1) | Trajectory synthesis with a GMM with dynamic features (trajectory GMM), where the GMM is learned from trajectory examples ||
| [demo_trajHSMM01.m](./demos/demo_trajHSMM01.m) | [[1]](#ref-1) | Trajectory synthesis with an HSMM with dynamic features (trajectory-HSMM) |
| [demo_trajHSMM_adaptiveDuration01.m](./demos/demo_trajHSMM_adaptiveDuration01.m) | [[1]](#ref-1) | Hidden semi-Markov model with adaptive duration |
| [demo_trajHSMM_adaptiveDuration_online01.m](./demos/demo_trajHSMM_adaptiveDuration_online01.m) | [[1]](#ref-1) | Online trajectory retrieval method built on an HSMM with adaptive duration and a trajectory-GMM representation |


### References

If you find PbDLib useful for your research, please cite the related publications!

#### Ref. [1] 
**GMM, TP-GMM, MFA, MPPCA, GPR, trajGMM**<br>
[Link to publication](http://calinon.ch/papers/Calinon-JIST2015.pdf)
```
@article{Calinon16JIST,
	author="Calinon, S.",
	title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
	journal="Intelligent Service Robotics",
	publisher="Springer Berlin Heidelberg",
	year="2016",
	volume="9",
	number="1",
	pages="1--29",
	doi="10.1007/s11370-015-0187-9",
}
```

#### Ref. [2] 
**MPC, LQR, HMM, HSMM**<br>
[Link to publication](http://calinon.ch/papers/Calinon-Lee-learningControl.pdf)
```
@incollection{Calinon19chapter,
	author="Calinon, S. and Lee, D.",
	title="Learning Control",
	booktitle="Humanoid Robotics: a Reference",
	publisher="Springer",
	editor="Vadakkepat, P. and Goswami, A.", 
	year="2019",
	doi="10.1007/978-94-007-7194-9_68-1",
	pages="1--52"
}
```

#### Ref. [3] 
**Riemannian manifolds (S2,S3)**<br>
[Link to publication](http://calinon.ch/papers/Zeestraten-RAL2017.pdf)
```
@article{Zeestraten17RAL,
	author="Zeestraten, M. J. A. and Havoutis, I. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
	title="An Approach for Imitation Learning on {R}iemannian Manifolds",
	journal="{IEEE} Robotics and Automation Letters ({RA-L})",
	year="2017",
	month="June",
	volume="2",
	number="3",
	pages="1240--1247"
	doi="10.1109/LRA.2017.2657001",
}
```

#### Ref. [4] 
**Riemannian manifolds (S+)**<br>
[Link to publication](http://calinon.ch/papers/Jaquier-IROS2017.pdf)
```
@inproceedings{Jaquier17IROS,
	author="Jaquier, N. and Calinon, S.", 
	title="Gaussian Mixture Regression on Symmetric Positive Definite Matrices Manifolds: Application to Wrist Motion Estimation with {sEMG}",
	booktitle="Proc. {IEEE/RSJ} Intl Conf. on Intelligent Robots and Systems ({IROS})",
	year="2017",
	month="September",
	address="Vancouver, Canada",
	pages="59--64"
}
```

#### Ref. [5] 
**Ergodic control, Bezier curves, LWR, GMR, probabilistic movement primitives**<br>
[Link to publication](http://calinon.ch/papers/Calinon-MM2019.pdf)
```
@incollection{Calinon19MM,
	author="Calinon, S.",
	title="Mixture Models for the Analysis, Edition, and Synthesis of Continuous Time Series",
	booktitle="Mixture Models and Applications",
	publisher="Springer",
	editor="Bouguila, N. and Fan, W.", 
	year="2019"
}
```

#### Ref. [6] 
**DP-means**<br>
[Link to publication](http://calinon.ch/papers/Bruno-AURO2017.pdf)
```
@article{Bruno17AURO,
	author="Bruno, D. and Calinon, S. and Caldwell, D. G.",
	title="Learning Autonomous Behaviours for the Body of a Flexible Surgical Robot",
	journal="Autonomous Robots",
	year="2017",
	month="February",
	volume="41",
	number="2",
	pages="333--347",
	doi="10.1007/s10514-016-9544-6"
}
```

#### Ref. [7] 
**Manipulability ellispoids**<br>
[Link to publication](http://calinon.ch/papers/Rozo-IROS2017.pdf)
```
@inproceedings{Rozo17IROS,
	author="Rozo, L. and Jaquier, N. and Calinon, S. and Caldwell, D. G.", 
	title="Learning Manipulability Ellipsoids for Task Compatibility in Robot Manipulation",
	booktitle="Proc. {IEEE/RSJ} Intl Conf. on Intelligent Robots and Systems ({IROS})",
	year="2017",
	month="September",
	address="Vancouver, Canada",
	pages="3183--3189"
}
```

#### Ref. [8] 
**Keypoint-based motion through MPC**<br>
[Link to publication](http://calinon.ch/papers/Berio-GI2017.pdf)
```
@inproceedings{Berio17GI,
	author="Berio, D. and Calinon, S. and Fol Leymarie, F.",
	title="Generating Calligraphic Trajectories with Model Predictive Control",
	booktitle="Proc. 43rd Conf. on Graphics Interface",
	year="2017",
	month="May",
	address="Edmonton, AL, Canada",
	pages="132--139",
	doi="10.20380/GI2017.17"
}
```

#### Ref. [9] 
**HSMM with adaptive time duration**<br>
[Link to publication](http://calinon.ch/papers/Rozo-Frontiers2016.pdf)
```
@article{Rozo16Frontiers,
	author="Rozo, L. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
	title="Learning Controllers for Reactive and Proactive Behaviors in Human-Robot Collaboration",
	journal="Frontiers in Robotics and {AI}",
	year="2016",
	month="June",
	volume="3",
	number="30",
	pages="1--11",
	doi="10.3389/frobt.2016.00030"
}
```

#### Ref. [10] 
**Manipulability ellipsoids control**<br>
[Link to publication](http://calinon.ch/paper3066.htm)
```
@incollection{@article{Jaquier18RSS,
	author="Jaquier, N. and Rozo, L. and Caldwell, D. G. and Calinon, S.",
	title="Geometry-aware Tracking of Manipulability Ellipsoids",
	year="2018",
	booktitle = "Robotics: Science and Systems ({R:SS})",
	address = "Pittsburgh, USA"
}
```

### Gallery

[demo\_GMR\_3Dviz01.m](./demos/demo_GMR_3Dviz01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_GMR_3Dviz01.png)

***

[demo\_GMR\_polyFit02.m](./demos/demo_GMR_polyFit02.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_GMRpolyFit02.png)

***

[demo\_HMM01.m](./demos/demo_HMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_HMM01.png)

***

[demo\_MPC\_iterativeLQR01.m](./demos/demo_MPC_iterativeLQR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_iterativeLQR01.png)

***

[demo\_Riemannian\_SPD\_GMR01.m](./demos/demo_Riemannian_SPD_GMR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_Riemannian_cov_GMR01.png)

***

[demo\_Riemannian\_SPD\_interp01.m](./demos/demo_Riemannian_SPD_interp01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_Riemannian_cov_interp01.png)

***

[demo\_Riemannian\_SPD\_interp02.m](./demos/demo_Riemannian_SPD_interp02.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_Riemannian_cov_interp02.png)

***

[demo\_Riemannian\_SPD\_interp03.m](./demos/demo_Riemannian_SPD_interp03.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_Riemannian_cov_interp03.png)

***

[demo\_Riemannian\_S2\_GMM01.m](./demos/demo_Riemannian_S2_GMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_Riemannian_sphere_GMM01.png)

***

[demo\_GMM\_semiTied01.m](./demos/demo_GMM_semiTied01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_semitiedGMM01.png)

***

[demo\_TPbatchLQR01.m](./demos/demo_TPbatchLQR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_TPbatchLQR01.png)

***

[demo\_TPGMR01.m](./demos/demo_TPGMR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_TPGMR01.png)

***

[demo\_TPproMP01.m](./demos/demo_TPproMP01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_TPproMP01.png)

***

[demo\_trajHSMM01.m](./demos/demo_trajHSMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-sandbox-matlab/raw/master/images/demo_trajHSMM01.png)


### License

The Matlab/GNU Octave version of PbDlib is maintained by Sylvain Calinon, Idiap Research Institute. 

Copyright (c) 2015-2019 Idiap Research Institute, http://idiap.ch/

PbDlib is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

PbDlib is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

