
/*
* Copywrite Zachary Kratochvil 2023
* All rights reserved.
*/

// imports

#include "ros/ros.h"

#include <sstream>
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <chrono>
#include <ctime>    
#include <vector>

// message imports
#include "freenove_ros/SensorDistance.h"

// ultrasonic sensing and publishing logic
class Ultrasonic_ROS{

	public:
		static const unsigned CHANGE_LOW = 0;
    	static const unsigned CHANGE_HIGH = 1;
    	static const unsigned TIMEOUT = 2;

		static const unsigned trigger_pin = 27;
	    static const unsigned echo_pin = 22;

	    static inline unsigned smoothing;
	    static inline std::vector<double> distances_cm;

	    static inline double start_time;

		static void initialize(unsigned smoothing_param){
			// initialize class params
			smoothing = smoothing_param;
			distances_cm.assign(smoothing, -1);
			start_time = std::numeric_limits<double>::max();

			// intialize GPIO
	        if (gpioInitialise() < 0){
			   ROS_ERROR("%s", "Failed to initialize GPIO.");
			};
			gpioSetMode(trigger_pin, PI_OUTPUT);
			gpioSetMode(echo_pin, PI_INPUT);

			gpioSetAlertFunc(echo_pin, &process_distance);
		};

		static void start(unsigned rate){

			ROS_ERROR("%s", "ready to publish");

			ros::Rate ros_rate = ros::Rate(rate);
			
			// measure
			while (ros::ok()){

				ROS_ERROR("%s", "measuring");
				
				send_trigger_pulse();
			
				ros::spinOnce();
				ros_rate.sleep();
			};
		};

	    static void send_trigger_pulse(){
	        gpioWrite(trigger_pin, 1);
	        sleep(0.00015);
	        gpioWrite(trigger_pin, 0);
	    };

	    static void process_distance(int pin, int level, unsigned int tick){

	    	switch(level){
	    		case CHANGE_HIGH:
	    		{
	    			start_time = get_seconds();
	    			return;
    			}
	    		case CHANGE_LOW:
    			{
	    			double end = get_seconds();
	    			double pulse_len = end - start_time;
	    			push(&distances_cm, pulse_len/0.000058);
	    			break;
	    		}
	    		case TIMEOUT:
	    			push(&distances_cm, 0);
	    			start_time = std::numeric_limits<double>::max();
	    			break;
	    	};

	    	std::vector<unsigned short> valids;
	    	valids.assign(smoothing, 1);
	    	unsigned num_valids = smoothing;
	    	for(unsigned i = 0; i < smoothing; i++){
	    		if(distances_cm.at(i) < 0){
	    			valids.at(i) = 0;
	    			num_valids--;
	    		};
	    	};


	    	if(num_valids == 0){
	            //currently silent when no valid data
	        }
	        else{
	        	for(unsigned i = 0; i < distances_cm.size(); i++){
	        		distances_cm.at(i) = distances_cm.at(i)*valids.at(i);
	        	}
	        	std::sort(distances_cm.begin(), distances_cm.end());

	        	unsigned i;
	        	for(; i <= distances_cm.size(); i++){
	        		if(distances_cm.at(i) > 0){
	        			break;
	        		};
	        	};

	            unsigned middle = std::floor((i + distances_cm.size())/2);
	            double median = distances_cm.at(middle);
	            publish(median);
	        };  
	    };

	    static void push(std::vector<double> *v, double val){
	    	for(unsigned i = smoothing-1; i >= 0; i--){
				v->at(i) = v->at(i - 1);
			};
			v->at(0) = val;
	    };

		static double get_seconds(){
			auto current_time = std::chrono::system_clock::now();
			auto duration = std::chrono::duration<double>(current_time.time_since_epoch());
			return duration.count();
		};

		static void publish(double distance_cm){
			ros::Time timestamp = ros::Time(get_seconds());
				
			freenove_ros::SensorDistance msg = freenove_ros::SensorDistance();
			msg.cm = distance_cm;
			msg.timestamp = timestamp;
			//ultraPub.publish(msg);
		};
};
		
/*
	rate to publish (Hz). Note actual rate will be
	limited by physical speed of sound:
	5m max distance from sensor, 5 measurements,
	means sound travels 2x5x5=50m. At 343m/s
	this takes .146 seconds.
*/
int main(int argc, char **argv){

	ros::init(argc, argv, "ultrasonic_publisher");
	ros::NodeHandle n;
	//ultraPub = n.advertise<freenove_ros::SensorDistance>("ultrasonic_distance", 1);

	/*
    try{
    	Ultrasonic_ROS::initialize(5);
    	Ultrasonic_ROS::start(100);
    }
    catch(int e){
    	gpioTerminate();
    	return 1;
    }

*/
    return 0;
};
