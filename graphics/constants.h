#define INCREMENT   10

#define VARS            4       //number of variables per particle (3 states and a W)
#define NUM_PARTICLES   10000    //number of particles in our distribution
#define NUM_PARTICLES_F 10000.0   //float version of particles for average

#define X            0       //labels for variable positions in particle array
#define Y            1
#define Q            2
#define W            3

#define PI              3.14159

#define MAX_X           800.0   //max values for position and heading given the environment
#define MAX_Y           600.0
#define MAX_Q           360 //180.0

#define MAX_X_INT           640
#define MAX_Y_INT           480
#define MAX_Q_INT           360 //180

#define ENCODER_NOISE   0.5    //amount of noise in encoders (as a fraction of the 
#define POSITION_NOISE  10.0   //amount of noise in position in mm

#define MAX_ANGLE 180
#define ANGLE_STEP 90
#define OBS_SIZE_X 60
#define OBS_SIZE_Y 60
#define SONAR_SIGMA 2
#define MAX_RANGE 253
#define CUTOFF_RANGE 300

#define FIRST_STEP 10
#define SECOND_STEP 5

#define NUM_OBSTACLES 5

#define CONVERGED 10