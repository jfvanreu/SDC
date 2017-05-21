/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <map>

#include "particle_filter.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    // set number of particles. We don't want too few or too many of them. Since we are in the neighborhood given by the GPS, not as many particles are necessary.
    num_particles = 100;
    
    //initialize the weights vector
    weights = std::vector<double>(num_particles);
    
    // declare random generator
    default_random_engine gen;

    // initialize particles based on GPS coordinates. Consider GPS noise.
    
    // Print GPS coordinates
    cout << "GPS coordinates: (" << x << " ," << y << ", " << theta << ")";

    
    // This line creates a normal (Gaussian) distribution for x, y and theta.
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // We generate num_particles random particles (x,y, theta) in the GPS coordinates neighborood  based on the Gaussian distribution that we just created.
    
    // Create a particle p
    Particle p;
    
    
    for (int i=0; i < num_particles; i++) {
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;
        weights[i]=1; //initialize the weights vector
        // Print our particle coordinates + weight
        //cout << "Particle: (" << p.x << ", " << p.y << ", " << p.theta << ", " << p.weight << ")" << endl;
        
        //Add our latest particle to the particle vector
        particles.push_back(p);
    }
    
    //set initialization is complete.
    is_initialized=true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // random Gaussian noise initialization
    std::default_random_engine gen;
    
    for (int i=0; i < num_particles; i++) {
        
        if (fabs(yaw_rate) < 0.00001) {  // if yaw_rate is near 0
            particles[i].x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
            particles[i].y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
            //theta remains unchanged so no action on it
            
        } else { //if yaw_rate is not close to zero
            particles[i].x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta+yaw_rate*delta_t) - sin(particles[i].theta));
            particles[i].y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t));
            particles[i].theta = particles[i].theta + yaw_rate*delta_t;
        }
        
        //create the distibutions around our new x,y and theta values
        std::normal_distribution<double> dist_x(particles[i].x, std_pos[0]);
        std::normal_distribution<double> dist_y(particles[i].y, std_pos[1]);
        std::normal_distribution<double> dist_theta(particles[i].theta, std_pos[2]);
        
        //generate a random number according to the distribution to include the noise.
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    // We need to associate the predicted landmarks with the observed landmarks. We use the dist helper function to compute the euclidian distance between points. We assume that the predicted measurements and observations are already in the same space.
    
    // we ended up not using this function. The arguments didn't make sense either.

}

/**
 * filterLandmarks method filters out the landmarks that are not within the sensor range  */

void filterLandmarks(double sensor_range, const Map& map_landmarks, std::vector<Map::single_landmark_s>&, LandmarkObs tobs) {
    //check if tobs is within sensor_range for x axis
    
    //check if tobs is within sensor_range for y axis
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
    
    // we update the weights of EACH particle, this means that we convert the observations from vehicle's space into the Map space using each particle as reference point.
    
    double min_distance;
    double current_dist;
    Map::single_landmark_s lm;
    
    //initialize mu values to zero. Does it make sense?
    double mu_x = 0.0, mu_y = 0.0;
    
    float prob = 1.0f;
    double std_x = std_landmark[0];
    double std_y = std_landmark[0];
    
    //create range variable to accelerate nearest neighbor search
    double x_range_min;
    double x_range_max;
    double y_range_min;
    double y_range_max;
    
    for (int i= 0; i < num_particles; i++) {

        //apply the observations to the particle and convert from car's space into map's space
        //std::vector<LandmarkObs> t_obs;
        LandmarkObs tobs;
        prob = 1.0;
        particles[i].weight=1;
        for (int j = 0; j < observations.size(); j++) {
            tobs.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
            tobs.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);
            
            //initialize x and y ranges
            x_range_min = tobs.x - sensor_range;
            x_range_max = tobs.x + sensor_range;
            y_range_min = tobs.y - sensor_range;
            y_range_max = tobs.y + sensor_range;

            //associate each obs (predicted observation) for this particle to a known map landmark
            min_distance = 1000; //initialize to a high number
            for (int l = 0; l < map_landmarks.landmark_list.size(); l++) {
                lm = map_landmarks.landmark_list[l];
                
                //only compute distance if landmarks within the sensor range
                
                if ((lm.x_f >= x_range_min && lm.x_f <= x_range_max) && (lm.y_f >= y_range_min && lm.y_f <= y_range_max)) {
                    current_dist = dist(tobs.x, tobs.y, lm.x_f, lm.y_f);
                    if (current_dist < min_distance) {
                        min_distance = current_dist;
                        tobs.id = lm.id_i;
                        mu_x = lm.x_f;
                        mu_y = lm.y_f;
                    }
                }
              }
            //compute probability associated to this t_obs
            prob = (1/(2*M_PI*std_x*std_y))*exp(-((pow(tobs.x-mu_x,2)/(2*pow(std_x,2)))+((pow(tobs.y-mu_y,2)/(2*pow(std_y,2))))));
            //update weight for current particle as a product of the weights for each t-obs.
            particles[i].weight *= prob;
        }
        // cout << "Weight for Particle " << i << ":" << particles[i].weight << endl;
        weights[i] = particles[i].weight;
    }
    
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles;
    resampled_particles.resize(num_particles);
    //std::map<int, int> m;
    for(int n=0; n < num_particles; n++) {
        resampled_particles[n] = particles[d(gen)];
    }
    particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
