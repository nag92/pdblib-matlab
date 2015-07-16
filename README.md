# pbdlib-matlab

PbDLib is a set of tools combining statistical learning, dynamical systems and optimal control approaches for programming-by-demonstration applications.

The Matlab/GNU Octave version is currently maintained by Sylvain Calinon, http://calinon.ch.

More information can be found on: http://programming-by-demonstration.org/lib/

### Compatibility

The codes are compatible with both Matlab or GNU Octave.

### Usage

Examples starting with `demo_` can be run from Matlab/GNU Octave.

### References

Did you find PbDLib useful for your research? Please acknowledge the authors in any academic publications that used some parts of these codes.

```
@article{Calinon15,
	author="Calinon, S.",
	title="A tutorial on task-parameterized movement learning and retrieval",
	year="2015",
}
```
```
@inproceedings{Calinon14ICRA,
	author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
	title="A task-parameterized probabilistic model with minimal intervention control",
	booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
	year="2014",
	month="May-June",
	address="Hong Kong, China",
	pages="3339--3344"
}
```

### Dataset

The folder "data" contains a dataset of 2D handwriting movements from LASA-EPFL (http://lasa.epfl.ch), collected within the context of the AMARSI European Project. Reference: S.M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.

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
| demo_batchLQR01 | Controller retrieval through a batch solution of linear quadratic optimal control (unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data (see also demo_iterativeLQR01) |
| demo_DSGMR01 | Gaussian mixture model (GMM), with Gaussian mixture regression(GMR) and dynamical systems used for reproduction, with decay variable used as input (as in DMP) |
| demo_DTW01 | Trajectory realignment through dynamic time warping (DTW) |
| demo_GMM01 | Gaussian mixture model (GMM) parameters estimation |
| demo_GMR01 | Gaussian mixture model (GMM) and time-based Gaussian mixture regression (GMR) used for reproduction |
| demo_GPR01 | Use of Gaussian process regression (GPR) as a task-parameterized model, with DS-GMR used to retrieve continuous movements |
| demo_HDDC01 | High Dimensional Data Clustering (HDDC, or HD-GMM) model from Bouveyron (2007) |
| demo_iterativeLQR01 | Controller retrieval through an iterative solution of linear quadratic optimal control (finite horizon, unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data (see also demo_batchLQR01) |
| demo_MFA01 | Mixture of factor analyzers (MFA) |
| demo_MPPCA01 | Mixture of probabilistic principal component analyzers (MPPCA) |
| demo_stdPGMM01 | Parametric Gaussian mixture model (PGMM) used as a task-parameterized model, with DS-GMR employed to retrieve continuous movements |
| demo_testLQR01 | Test of the linear quadratic regulation with different variance in the data |
| demo_testLQR02 | Test of the linear quadratic regulation (LQR) with evaluation of the damping ratio found by the system |
| demo_testLQR03 | Comparison of linear quadratic regulators (LQR) with finite and infinite time horizons |
| demo_TPbatchLQR01 | Task-parameterized GMM encoding position and velocity data, combined with a batch solution of linear quadratic optimal control (unconstrained linear MPC) |
| demo_TPbatchLQR02 | Batch solution of a linear quadratic optimal control (unconstrained linear MPC) acting in multiple frames, which is equivalent to TP-GMM combined with LQR |
| demo_TPGMM01 | Task-parameterized Gaussian mixture model (TP-GMM) encoding |
| demo_TPGMR01 | Task-parameterized Gaussian mixture model (TP-GMM), with GMR used for reproduction (without dynamical system) |
| demo_TPGMR_DS01 | Dynamical system with constant gains used with a task-parameterized model |
| demo_TPGMR_LQR01 | Finite horizon LQR used with a task-parameterized model  
| demo_TPGMR_LQR02 | Infinite horizon LQR used with a task-parameterized model  
| demo_TPGP01 | Task-parameterized Gaussian process regression (TP-GPR) |
| demo_TPHDDC01 | Task-parameterized high dimensional data clustering (TP-HDDC) |
| demo_TPMFA01 | Task-parameterized mixture of factor analyzers (TP-MFA), without motion retrieval |
| demo_TPMPPCA01 | Task-parameterized mixture of probabilistic principal component analyzers (TP-MPPCA) |
| demo_TPtrajGMM01 | Task-parameterized model with trajectory-GMM encoding |
| demo_trajGMM01 | Reproduction of trajectory with a GMM with dynamic features (trajectory GMM) |
| demoIK_nullspace_TPGMM01 | IK with nullspace treated with task-parameterized GMM (bimanual tracking task, version with 4 frames) |
| demoIK_pointing_TPGMM01 | Task-parameterized GMM to encode pointing direction by considering nullspace constraint (4 frames) (example with two objects and robot frame, starting from the same initial pose (nullspace constraint), by using a single Euler orientation angle and 3 DOFs robot) |
