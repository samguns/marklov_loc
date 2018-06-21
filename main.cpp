#include <iostream>
#include <algorithm>
#include <vector>

#include "helper_functions.h"

using namespace std;

//initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev
std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                     float position_stdev);
float motion_model(float pseudo_position, float movement, std::vector<float> priors,
                   int map_size, int control_stdev);
std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions, float pseudo_position);
//observation model: calculates likelihood prob term based on landmark proximity
float observation_model(std::vector<float> landmark_positions, std::vector<float> observations,
                        std::vector<float> pseudo_ranges, float distance_max,
                        float observation_stdev);

static void validate_initialize_priors();
static void validate_normpdf();
static void validate_motion_model();
static void validate_pseudo_range_estimator();
static void validate_observation_model();
static void validate_marklov_filter();

int main() {

  // validate_initialize_priors();
  // validate_normpdf();
  // validate_motion_model();
  // validate_pseudo_range_estimator();
  // validate_observation_model();
  validate_marklov_filter();

  return 0;

};

std::vector<float> initialize_priors(int map_size, std::vector<float> landmark_positions,
                                     float position_stdev) {

//initialize priors assumimg vehicle at landmark +/- 1.0 meters position stdev

  //set all priors to 0.0
  std::vector<float> priors(map_size, 0.0);

  int nof_possible_pos = 2 * position_stdev + 1;
  int nof_landmarks = landmark_positions.size();
  float possible_pos = nof_possible_pos * nof_landmarks;
  float prob = 1.0 / possible_pos;

  for (int i = 0; i < nof_landmarks; i++) {
    float landmark_idx = landmark_positions[i];
    priors[landmark_idx] = prob;

    for (int j = position_stdev; j > 0; j--) {
      priors[landmark_idx - j] = prob;
      priors[landmark_idx + j] = prob;
    }
  }


  return priors;
}

static void validate_initialize_priors() {
  //set standard deviation of position:
  float position_stdev = 1.0f;

  //set map horizon distance in meters
  int map_size = 25;

  //initialize landmarks
  std::vector<float> landmark_positions {5, 10, 20};

  // initialize priors
  std::vector<float> priors = initialize_priors(map_size, landmark_positions,
                                                position_stdev);

  //print values to stdout
  for (unsigned int p = 0; p < priors.size(); p++) {
    std::cout << priors[p] << endl;
  }
}

static void validate_normpdf() {
  float value = 2;


  float parameter = 1.0; //set as control parameter or observation measurement
  float stdev = 1.0; //position or observation standard deviation

  float prob = Helpers::normpdf(value, parameter, stdev);

  std::cout << prob << endl;
}

static void validate_motion_model() {
  //set standard deviation of control:
  float control_stdev = 1.0f;

  //set standard deviation of position:
  float position_stdev = 1.0f;

  //meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  //number of x positions on map
  int map_size = 25;

  //initialize landmarks
  std::vector<float> landmark_positions {5, 10, 20};

  // initialize priors
  std::vector<float> priors = initialize_priors(map_size, landmark_positions,
                                                position_stdev);

  //step through each pseudo position x (i)
  for (unsigned int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);

    //get the motion model probability for each x position
    float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                     priors, map_size, control_stdev);

    //print to stdout
    std::cout << pseudo_position << "\t" << motion_prob << endl;
  }
}

//TODO, implement the motion model: calculates prob of being at an estimated position at time t
float motion_model(float pseudo_position, float movement, std::vector<float> priors,
                   int map_size, int control_stdev) {

  //initialize probability
  float position_prob = 0.0f;

  //YOUR CODE HERE
  for (int i = 0; i < map_size; i++) {
    float delta_position = pseudo_position - float(i);
    float p_transition = Helpers::normpdf(delta_position, movement, control_stdev);

    position_prob += p_transition * priors[i];
  }


  return position_prob;
}

static void validate_pseudo_range_estimator() {
  //set standard deviation of control:
  float control_stdev = 1.0f;

  //meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  //number of x positions on map
  int map_size = 25;

  //define landmarks
  std::vector<float> landmark_positions {5, 10, 12, 20};

  //step through each pseudo position x (i)
  for (unsigned int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);
    //get pseudo ranges
    std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, pseudo_position);

    //print to stdout
    if (pseudo_ranges.size() >0) {
      for (unsigned int s = 0; s < pseudo_ranges.size(); ++s) {
        std::cout << "x: " << i << "\t" << pseudo_ranges[s] << endl;
      }
      std::cout << "-----------------------" << endl;
    }
  }
}

std::vector<float> pseudo_range_estimator(std::vector<float> landmark_positions, float pseudo_position) {

  //define pseudo observation vector:
  std::vector<float> pseudo_ranges;

  //loop over number of landmarks and estimate pseudo ranges:
  for (size_t i = 0; i < landmark_positions.size(); i++) {
    float pseudo_range = landmark_positions[i] - pseudo_position;
    if (pseudo_range > 0) {
      pseudo_ranges.push_back(pseudo_range);
    }
  }


  //sort pseudo range vector:
  sort(pseudo_ranges.begin(), pseudo_ranges.end());

  return pseudo_ranges;
}

static void validate_observation_model() {
  //set observation standard deviation:
  float observation_stdev = 1.0f;

  //number of x positions on map
  int map_size = 25;

  //set distance max
  float distance_max = map_size;

  //define landmarks
  std::vector<float> landmark_positions {5, 10, 12, 20};

  //define observations
  std::vector<float> observations {5.5, 13, 15};

  //step through each pseudo position x (i)
  for (unsigned int i = 0; i < map_size; ++i) {
    float pseudo_position = float(i);

    //get pseudo ranges
    std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions,
                                                              pseudo_position);

    //get observation probability
    float observation_prob = observation_model(landmark_positions, observations,
                                               pseudo_ranges, distance_max,
                                               observation_stdev);

    //print to stdout
    std::cout << observation_prob << endl;
  }
}

//calculates likelihood prob term based on landmark proximity
float observation_model(std::vector<float> landmark_positions, std::vector<float> observations,
                        std::vector<float> pseudo_ranges, float distance_max,
                        float observation_stdev) {

  float distance_prob = 1.0f;

  for (float observation : observations) {
    float probs;
    if (pseudo_ranges.empty()) {
      probs = Helpers::normpdf(observation, numeric_limits<const float>::infinity(), observation_stdev);
    } else {
      float pseudo_range_estimate = pseudo_ranges.front();
      pseudo_ranges.erase(pseudo_ranges.begin());
      probs = Helpers::normpdf(observation, pseudo_range_estimate, observation_stdev);
    }

    distance_prob *= probs;
  }

  return distance_prob;
}

static void validate_marklov_filter() {
  //set standard deviation of control:
  float control_stdev = 1.0f;

  //set standard deviation of position:
  float position_stdev = 1.0f;

  //meters vehicle moves per time step
  float movement_per_timestep = 1.0f;

  //set observation standard deviation:
  float observation_stdev = 1.0f;

  //number of x positions on map
  int map_size = 25;

  //set distance max
  float distance_max = map_size;

  //define landmarks
  std::vector<float> landmark_positions {3, 9, 14, 23};

  //define observations vector, each inner vector represents a set of observations
  //for a time step
  std::vector<std::vector<float> > sensor_obs {{1,7,12,21}, {0,6,11,20}, {5,10,19}, {4,9,18},
                                               {3,8,17}, {2,7,16}, {1,6,15}, {0,5,14}, {4,13},
                                               {3,12},{2,11},{1,10},{0,9},{8},{7},{6},{5},{4},{3},{2},{1},{0},
                                               {}, {}, {}};

  std::vector<float> priors = initialize_priors(map_size, landmark_positions,
                                                position_stdev);

  //UNCOMMENT TO SEE THIS STEP OF THE FILTER
  std::cout << "-----------PRIORS INIT--------------" << endl;

  for (unsigned int p = 0; p < priors.size(); p++){
      std::cout << priors[p] << endl;
  }

  //initialize posteriors
  std::vector<float> posteriors(map_size, 0.0);

  //specify time steps
  int time_steps = sensor_obs.size();

  //declare observations vector
  std::vector<float> observations;

  //cycle through time steps
  for (unsigned int t = 0; t < time_steps; t++){

    //UNCOMMENT TO SEE THIS STEP OF THE FILTER

    std::cout << "---------------TIME STEP---------------" << endl;
    std::cout << "t = " << t << endl;

    std::cout << "-----Motion----------OBS----------------PRODUCT--" << endl;


    if (!sensor_obs[t].empty()) {
      observations = sensor_obs[t];
    } else {
      observations = {float(distance_max)};
    }

    //step through each pseudo position x (i)
    for (unsigned int i = 0; i < map_size; ++i) {
      float pseudo_position = float(i);

      // get the motion model probability for each x position
      float motion_prob = motion_model(pseudo_position, movement_per_timestep,
                                       priors, map_size, control_stdev);


      // get pseudo ranges
      std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions, pseudo_position);


      // get observation probability
      float observation_prob = observation_model(landmark_positions, observations,
                                                 pseudo_ranges, distance_max,
                                                 observation_stdev);


      // calculate the ith posterior and pass to posteriors vector
      posteriors[i] = motion_prob * observation_prob;


      //UNCOMMENT TO SEE THIS STEP OF THE FILTER
      std::cout << motion_prob << "\t" << observation_prob << "\t"
      << "\t"  << motion_prob * observation_prob << endl;



    }

    //UNCOMMENT TO SEE THIS STEP OF THE FILTER
    std::cout << "----------RAW---------------" << endl;

    for (unsigned int p = 0; p < posteriors.size(); p++) {
        std::cout << posteriors[p] << endl;
    }



    // normalize posteriors (see helpers.h for a helper function)
    posteriors = Helpers::normalize_vector(posteriors);

    //print to stdout
    std::cout << posteriors[t] <<  "\t" << priors[t] << endl;

    //UNCOMMENT TO SEE THIS STEP OF THE FILTER
    std::cout << "----------NORMALIZED---------------" << endl;

    // update priors
    priors = posteriors;


    //UNCOMMENT TO SEE THIS STEP OF THE FILTER
    for (unsigned int p = 0; p < posteriors.size(); p++) {
        std::cout << posteriors[p] << endl;
    }


    //print final posteriors vector to stdout
    for (unsigned int p = 0; p < posteriors.size(); p++) {
      std::cout << posteriors[p] << endl;
    }
  }
}