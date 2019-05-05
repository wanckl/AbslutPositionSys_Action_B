## AbslutPositionSys_Action_B

------------------------------
#### My graduate design to fork a OPS for *Robocon compitition* or indoor 2D move device, which is a embeded sys on *STM32F405*, calc and output current *position info* in the coordinate system created on the start point.

+ dbug_log: 05/05/2019   
   modify AS5045 cycle counter to zero.   
   simplly set PID parament for temperature control. 

+ dbug_log: 05/04/2019  
   get sin graph form AS5045 and linen it.   
   successfully count tonggle freq from AS5045 data and int for position OK.   
   waitting for PID control for imu device. 
   
+ dbug_log: 05/02/2019   
   modify dmp driver to tow mpu6050.   
   find that mpu6050 with ***dmp can't init at a not horizental flat***.   

+ dbug_log: 05/01/2019   
   find bug @pwm configor, modified and test OK.  
   midify usart1 rx interrupt to recive instructions.   
   use mpu6050 dmp driver for one sensor successful. 

+ dbug_log: 04/30/2019  
   add tim2 interrupt to provide a time base for imu calc, test OK.  
   modify Heat_Res from GPIO to PWM drive mode, to get a smoth control, test OK. 
   
+ dbug_log: 04/29/2019   
   successfully read the sensor AS5045's data,  
   it means all hardware is ready, then fit them togather.

+ dbug_log: 04/27/2019   
   successfully read origin data from MPU6050 imu sensor.  
   waitting for data-processing and AS5045.
