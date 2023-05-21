
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

/*
* code partially from original Freenove code
* otherwise by Zachary Kratochvil
*/

class Ultrasonic{
	private:
		unsigned smoothing = 5;
		unsigned trigger_pin = 27;
	    unsigned echo_pin = 22;

	public:
	    Ultrasonic(){
	        /*
	        if(new_smoothing != NULL){
	        	smoothing = *new_smoothing;
	        };
	        */
	        smoothing = 5;

	        // intialize GPIO
	        if (gpioInitialise() < 0){
			   //rospy.logerr("GPIO initialization failed.");
			};
			gpioSetMode(trigger_pin, PI_OUTPUT);
			gpioSetMode(echo_pin, PI_INPUT);
		};

	    void send_trigger_pulse(){
	        gpioWrite(trigger_pin, 1);
	        sleep(0.00015);
	        gpioWrite(trigger_pin, 0);
	    };
	     
	    // take median of self.smoothing many measurements
	    double get_distance(){
	    	vector<double> distance_cm = vect(smoothing,-1);
	        for(unsigned i = 0; i < std::vector::size(distance_cm); i++){}
	            
	            # take and store measurement
	            self.send_trigger_pulse()
	            
	            timeout = GPIO.wait_for_edge(self.echo_pin, GPIO.RISING, timeout=35)
	            if timeout is None:
	                continue
	            start = time.time()
	            
	            timeout = GPIO.wait_for_edge(self.echo_pin, GPIO.FALLING, timeout=35)
	            if timeout is None:
	                continue
	            finish = time.time()

	            pulse_len = finish-start
	            distance_cm[i] = pulse_len/0.000058
	        }
	        
	        # not accurate above 5m
	        valids = np.nonzero((distance_cm > 0) & (distance_cm < 500))[0]
	        if len(valids) == 0:
	            return -1
	        else:
	            distance_cm=sorted(distance_cm[valids])
	            middle = len(distance_cm) // 2
	            median = distance_cm[middle]
	            return median
	            */
};
          
  
  /*      
// Test logic follows:
int main(){

    cout << "Program is starting ... ";
    
    Ultrasonic ultrasonic=Ultrasonic(NULL);
    ultrasonic.send_trigger_pulse();
    while(true){
            //cout << ultrasonic.get_distance());
            sleep(10);
    };

    return 0;
};
*/


class Ultrasonic_ROS{

	private:
		ros::NodeHandle n;
		ros::Publisher ultraPub;
		Ultrasonic sensor;

	public:
		Ultrasonic_ROS(){
			ultraPub = n.advertise<freenove_ros::SensorDistance>("ultrasonic_distance", 1);

			sensor = Ultrasonic();
		};

		double get_seconds(){
			auto current_time = std::chrono::system_clock::now();
			auto duration = std::chrono::duration<double>(current_time.time_since_epoch());
			return duration.count();
		};

		void start(unsigned rate){

			ROS_ERROR("%s", "ready to publish");

			ros::Rate ros_rate = ros::Rate(rate);
			
			// publish
			while (ros::ok()){

				ROS_ERROR("%s", "measuring");
				
				double start = get_seconds();
				double distance = sensor.get_distance();
				double end = get_seconds();
				
				double avg_time = (start+end)/2;
				ros::Time timestamp = ros::Time(avg_time);
				
				freenove_ros::SensorDistance msg = freenove_ros::SensorDistance();
				msg.cm = 0; //distance;
				msg.timestamp = timestamp;
				ultraPub.publish(msg);

				ros::spinOnce();
				ros_rate.sleep();
			};
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

    Ultrasonic_ROS sensor = Ultrasonic_ROS();
    sensor.start(100);

    return 0;
};
