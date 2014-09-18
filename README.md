# pbdlib-matlab

### Compatibility

	The codes have been tested with both Matlab and GNU Octave.

### Usage

	Unzip the file and run 'demo_TPGMR_LQR01' (finite horizon LQR), 'demo_TPGMR_LQR02' (infinite horizon LQR) or
	'demo_DSGMR01' (dynamical system with constant gains) in Matlab.  
	'demo_testLQR01', 'demo_testLQR02' and 'demo_testLQR03' can also be run as additional examples of LQR.

### Reference  

	Calinon, S., Bruno, D. and Caldwell, D.G. (2014). A task-parameterized probabilistic model with minimal intervention 
	control. Proc. of the IEEE Intl Conf. on Robotics and Automation (ICRA).

### Description

	Demonstration a task-parameterized probabilistic model encoding movements in the form of virtual spring-damper systems
	acting in multiple frames of reference. Each candidate coordinate system observes a set of demonstrations from its own
	perspective, by extracting an attractor path whose variations depend on the relevance of the frame through the task. 
	This information is exploited to generate a new attractor path corresponding to new situations (new positions and
	orientation of the frames), while the predicted covariances are exploited by a linear quadratic regulator (LQR) to 
	estimate the stiffness and damping feedback terms of the spring-damper systems, resulting in a minimal intervention 
	control strategy.

### Authors

	Sylvain Calinon and Danilo Bruno, 2014
	http://programming-by-demonstration.org/
		
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

