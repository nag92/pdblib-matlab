# pbdlib-matlab

### Compatibility

	The codes have been tested with both Matlab and GNU Octave.

### Usage

	Examples starting with 'demo_' can be run from Matlab/GNU Octave.

### References

	Calinon, S. (in preparation). A tutorial on task-parameterized movement learning and retrieval.

	Calinon, S., Bruno, D. and Caldwell, D.G. (2014). A task-parameterized probabilistic model with minimal intervention 
	control. Proc. of the IEEE Intl Conf. on Robotics and Automation (ICRA).

### Description

	Demonstration of a task-parameterized probabilistic model encoding movements in the form of virtual spring-damper systems
	acting in multiple frames of reference. Each candidate coordinate system observes a set of demonstrations from its own
	perspective, by extracting an attractor path whose variations depend on the relevance of the frame through the task. 
	This information is exploited to generate a new attractor path corresponding to new situations (new positions and
	orientation of the frames), while the predicted covariances are exploited by a linear quadratic regulator (LQR) to 
	estimate the stiffness and damping feedback terms of the spring-damper systems, resulting in a minimal intervention 
	control strategy.

### Datasets
	
	The folder "data" contains a dataset of 2D handwriting movements from LASA-EPFL (http://lasa.epfl.ch), collected within 
	the context of the AMARSI European Project. Reference: S.M. Khansari-Zadeh and A. Billard, "Learning Stable Non-Linear 
	Dynamical Systems with Gaussian Mixture Models", IEEE Transaction on Robotics, 2011.

### Contact

	Sylvain Calinon
	http://programming-by-demonstration.org/lib/
		
	This source code is given for free! In exchange, we would be grateful if you cite the following reference in any 
	academic publication that uses this code or part of it:

	@inproceedings{Calinon14ICRA,
		author="Calinon, S. and Bruno, D. and Caldwell, D. G.",
		title="A task-parameterized probabilistic model with minimal intervention control",
		booktitle="Proc. {IEEE} Intl Conf. on Robotics and Automation ({ICRA})",
		year="2014",
		month="May-June",
		address="Hong Kong, China",
		pages="3339--3344"
	}

