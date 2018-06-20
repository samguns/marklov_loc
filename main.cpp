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

static void validate_initialize_priors();
static void validate_normpdf();
static void validate_motion_model();

int main() {

  // validate_initialize_priors();
  // validate_normpdf();
  validate_motion_model();

  return 0;

};

//TODO: Complete the initialize_priors function
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