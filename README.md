# pbdlib-matlab

PbDlib is a set of tools combining statistical learning, dynamical systems and optimal control approaches for programming-by-demonstration applications (see http://www.idiap.ch/software/pbdlib/ for details). 

The Matlab/GNU Octave version is currently maintained by Sylvain Calinon, Idiap Research Institute. A C++ version of the library (with currently fewer functionalities) is available at https://gitlab.idiap.ch/rli/pbdlib

### References

Did you find PbDLib useful for your research? Please acknowledge the authors in any academic publications that used parts of these codes.

* Tutorial (covers GMM, TP-GMM, MFA, MPPCA, GMR, LWR, GPR, MPC, LQR, trajGMM):
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

* HMM, HSMM:
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

* Semi-tied covariances in mixture models:
@article{Tanwani16RAL,
  author="Tanwani, A. K. and Calinon, S.",
  title="Learning Robot Manipulation Tasks with Task-Parameterized Semi-Tied Hidden Semi-{M}arkov Model",
  journal="{IEEE} Robotics and Automation Letters ({RA-L})",
  year="2016",
  month="January",
  volume="1",
  number="1",
  pages="235--242",
	doi="10.1109/LRA.2016.2517825"
}

* DP-means:
@article{Bruno16AURO,
  author="Bruno, D. and Calinon, S. and Caldwell, D. G.",
  title="Learning Autonomous Behaviours for the Body of a Flexible Surgical Robot",
  journal="Autonomous Robots",
  year="2016",
  volume="",
  number="",
  pages="",
	doi="10.1007/s10514-016-9544-6",
}

### Compatibility

The codes are compatible with both Matlab and GNU Octave.

### Usage

Examples starting with `demo_` can be run from Matlab/GNU Octave.

### License

Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/

Maintained by Sylvain Calinon, http://calinon.ch/

PbDlib is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 as
published by the Free Software Foundation.

PbDlib is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

### List of examples

All the examples are located in the main folder, and the functions are located in the `m_fcts` folder.

| Filename | Description |
|----------|-------------|
| benchmark_DS_GP_GMM01 | Benchmark of task-parameterized model based on Gaussian process regression, with trajectory model (Gaussian mixture model encoding), and DS-GMR used for reproduction |
| benchmark_DS_GP_raw01 | Benchmark of task-parameterized model based on Gaussian process regression, with raw trajectory, and spring-damper system used for reproduction |
| benchmark_DS_PGMM01 | Benchmark of task-parameterized model based on parametric Gaussian mixture model, and DS-GMR used for reproduction |
| benchmark_DS_TP_GMM01 | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| benchmark_DS_TP_GP01 | Benchmark of task-parameterized Gaussian process (nonparametric task-parameterized method) |
| benchmark_DS_TP_LWR01 | Benchmark of task-parameterized locally weighted regression (nonparametric task-parameterized method) |
| benchmark_DS_TP_MFA01 | Benchmark of task-parameterized mixture of factor analyzers (TP-MFA), with DS-GMR used for reproduction |
| benchmark_DS_TP_trajGMM01 | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| demo_affineTransform01 | Affine transformations of raw data as pre-processing step to train a task-parameterized model |
| demo_batchLQR01 | Controller retrieval through a batch solution of linear quadratic optimal control (unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data (see also demo_iterativeLQR01) |
| demo_batchLQR02 | Same as demo_batchLQR01 but with only position data |
| demo_batchLQR_augmSigma01 | Batch LQR with augmented covariance to transform a tracking problem to a regulation problem |
| demo_DMP01 | Dynamic movement primitive (DMP) encoding with radial basis functions |
| demo_DMP02 | Generalization of dynamic movement primitive (DMP) with polynomial fitting using radial basis functions |
| demo_DMP_GMR01 | Emulation of a standard dynamic movement primitive (DMP) by using a GMM with diagonal covariance matrix, and retrieval computed through Gaussian mixture regression (GMR) | 
| demo_DMP_GMR02 | Same as demo_DMP_GMR01 but with full covariance matrices coordinating the different variables | 
| demo_DMP_GMR03 | Same as demo_DMP_GMR02 but with GMR used to regenerate the path of a spring-damper system instead of encoding the nonlinear forcing term | 
| demo_DMP_GMR04 | Same as demo_DMP_GMR03 by using the task-parameterized model formalism | 
| demo_DMP_GMR_LQR01 | Same as demo_DMP_GMR04 but with LQR used to refine the parameters of the spring-damper system | 
| demo_DMP_GMR_LQR02 | Same as demo_DMP_GMR_LQR01 with perturbations added to show the benefit of full covariance to coordinate disturbance rejection | 
| demo_DPMeans_Online01 | Online clustering with DP-means algorithm |
| demo_DSGMR01 | Gaussian mixture model (GMM), with Gaussian mixture regression(GMR) and dynamical systems used for reproduction, with decay variable used as input (as in DMP) |
| demo_DTW01 | Trajectory realignment through dynamic time warping (DTW) |
| demo_GMM01 | Gaussian mixture model (GMM) parameters estimation |
| demo_GMM02 | GMM with different covariance structures |
| demo_GMR01 | GMM and time-based Gaussian mixture regression (GMR) used for reproduction |
| demo_GMR02 | GMR computed with precision matrices instead of covariances |
| demo_GMR03 | Chain rule with Gaussian conditioning |
| demo_GMR_3Dviz01 | 3D visualization of a GMM with time-based GMR used for reproduction |
| demo_GMR_polyFit01 | Polynomial fitting with multivariate GMR |
| demo_GPR01 | Gaussian process regression (GPR) |
| demo_GPR02 | GPR with stochastic samples from the prior and the posterior |
| demo_GPR03 | GPR with periodic kernel function |
| demo_GPR_TP01 | Use of GPR as a task-parameterized model, with DS-GMR used to retrieve continuous movements |
| demo_grabData01 | Collect movement data from mouse cursor |
| demo_HDDC01 | High Dimensional Data Clustering (HDDC, or HD-GMM) |
| demo_HMM01 | Hidden Markov model (HMM) with single Gaussian as emission distribution |
| demo_HMM_Viterbi01 | Viterbi decoding in HMM to estimate best state sequence from observations |
| demo_HSMM01 | Variable duration model implemented as a hidden semi-Markov model (HSMM), by encoding the state duration after EM |
| demo_iterativeLQR01 | Controller retrieval through an iterative solution of linear quadratic optimal control (finite horizon, unconstrained linear MPC), by relying on a GMM encoding of position and velocity data (see also demo_batchLQR01) |
| demo_iterativeLQR02 | Same as demo_iterativeLQR01 with only position data |
| demo_MFA01 | Mixture of factor analyzers (MFA) |
| demo_MPPCA01 | Mixture of probabilistic principal component analyzers (MPPCA) |
| demo_regularization01 | Regularization of GMM parameters with minimum admissible eigenvalue |
| demo_regularization02 | Regularization of GMM parameters with the addition of a small circular covariance |
| demo_SEDS01 | Continuous autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| demo_SEDS_discrete01 | Discrete autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| demo_semitiedGMM01 | Semi-tied Gaussian Mixture Model by tying the covariance matrices of a GMM with a set of common basis vectors |
| demo_stdPGMM01 | Parametric Gaussian mixture model (PGMM) used as a task-parameterized model, with DS-GMR employed to retrieve continuous movements |
| demo_testDampingRatio01 | Test with critically damped system and ideal underdamped system |
| demo_testLQR01 | Test of linear quadratic regulation (LQR) with different variance in the data |
| demo_testLQR02 | Test of LQR with evaluation of the damping ratio found by the system |
| demo_testLQR03 | Comparison of LQR with finite and infinite time horizons |
| demo_testLQR04 | Demonstration of the coordination capability of linear quadratic optimal control (unconstrained linear MPC) when combined with full precision matrices |
| demo_TPbatchLQR01 | Task-parameterized GMM encoding position and velocity data, combined with a batch solution of linear quadratic optimal control (unconstrained linear MPC) |
| demo_TPbatchLQR02 | Batch solution of a linear quadratic optimal control acting in multiple frames, which is equivalent to TP-GMM combined with LQR |
| demo_TPGMM01 | Task-parameterized Gaussian mixture model (TP-GMM) encoding |
| demo_TPGMR01 | TP-GMM with GMR used for reproduction (without dynamical system) |
| demo_TPGMR_DS01 | Dynamical system with constant gains used with a task-parameterized model |
| demo_TPGMR_LQR01 | Finite horizon LQR used with a task-parameterized model | 
| demo_TPGMR_LQR02 | Infinite horizon LQR used with a task-parameterized model | 
| demo_TPGP01 | Task-parameterized Gaussian process regression (TP-GPR) |
| demo_TPHDDC01 | Task-parameterized high dimensional data clustering (TP-HDDC) |
| demo_TPMFA01 | Task-parameterized mixture of factor analyzers (TP-MFA), without motion retrieval |
| demo_TPMPC01 | Task-parameterized model encoding position data, with MPC used to track the associated stepwise reference path |
| demo_TPMPC02 | Same as demo_TPMPC01 with a generalized version of MPC used to track associated stepwise reference paths in multiple frames |
| demo_TPMPPCA01 | Task-parameterized mixture of probabilistic principal component analyzers (TP-MPPCA) |
| demo_TPproMP01 | Task-parameterized probabilistic movement primitives (TP-ProMP) |
| demo_TPtrajDistrib01 | Task-parameterized model with trajectory distribution and eigendecomposition |
| demo_TPtrajGMM01 | Task-parameterized model with trajectory-GMM encoding |
| demo_trajDistrib01 | Stochastic sampling with Gaussian trajectory distribution |
| demo_trajGMM01 | Reproduction of trajectory with a GMM with dynamic features (trajectory-GMM) |
| demo_trajHSMM01 | Trajectory synthesis with an HSMM with dynamic features (trajectory-HSMM) |
| demo_trajMFA01 | Trajectory model with either a mixture of factor analysers (MFA), a mixture of probabilistic principal component analyzers (MPPCA), or a high-dimensional data clustering approach (HD-GMM) |
| demoIK_nullspace_TPGMM01 | IK with nullspace treated with task-parameterized GMM (bimanual tracking task, version with 4 frames) |
| demoIK_pointing_TPGMM01 | Task-parameterized GMM to encode pointing direction by considering nullspace constraint (4 frames) (example with two objects and robot frame, starting from the same initial pose (nullspace constraint), by using a single Euler orientation angle and 3 DOFs robot) |
