# Freenove_ROS
ROS implementation of Freenove's 4WD smart car software.

## Sensors

### ADC
The ADC provides ambient light readings through the adc_publisher.py file and AmbientLight message. It also provides battery level readings as Float32 messages. To publish readings, run:  
` sudo rosrun freenove_ros adc_publisher.py `
  
Then subscribe to the ` adc_light_sensor ` or ` adc_batter_level ` topic.  
  
AmbientLight values are percentages of max physical ADC read voltage capability. Battery levels are in Volts.


## Actuators

### Buzzer
The buzzer is activated by buzzer_subscriber.py, topic buzzer, message OnOffDuration. To listen, run:
` sudo rosrun freenove_ros buzzer_subscriber.py `  
  
When publishing, True enables buzzer, False disables, and duration is the nubmer of seconds to apply this state for. If False and 0 it simply cancels the current buzz.


## References
Original source: https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi