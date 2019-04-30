## AbslutPositionSys_Action_B

------------------------------
#### My graduate design to fork a OPS for *Robocon compitition* or indoor 2D move device, which is a embeded sys on *STM32F405*, calc and output current *position info* in the coordinate system created on the start point.

+ dbug_log: 04/30/2019   
   add tim2 interrupt to provide a time base for imu calc, test OK.  
   modify Heat_Res from GPIO to PWM drive mode, to get a smoth control, test OK.
   
+ dbug_log: 04/29/2019   
   successfully read the sensor AS5045's data,  
   it means all hardware is ready, then fit them togather.

+ dbug_log: 04/27/2019   
   successfully read origin data from MPU6050 imu sensor.  
   waitting for data-processing and AS5045.
