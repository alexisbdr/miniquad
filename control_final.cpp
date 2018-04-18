#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include "vive.h"

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C


enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_keyboard();
void trap(int signal) ;
void safety_check();
void set_PWM(uint8_t channel, float time_on_us);
void kill_motors();
void init_pwm();
void init_motor(uint8_t channel);
void pid_update(int check);
void get_joystick();
void get_vive(); 


//global variables for Vive Sensing
float vive_yaw;
int vive_prev_version = 0;

float vive_roll;
float vive_x_estimate = 0;
float vive_roll_d = 0;
float prev_x = 0;

float vive_pitch;
float vive_y_estimate = 0;
float vive_pitch_d = 0;
float prev_y = 0;

float vive_thrust;
float vive_thrust_d = 0;
float thrust_I_term = 0;
float thrust_I_limit = 300;
float z_accel_average = 0; 
float velocity_estimate = 0; 
float prev_z;


// global struct
Position local_p;

int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
long vive_time_curr;
long vive_time_prev;
int vive_check=0; 
int prev_version=0; 
long js_time_curr;
long js_time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
//Global Variables for complementary filters
float roll_gyro_delta=0 ;
float comp_roll=0;
float pitch_gyro_delta=0 ;
float comp_pitch=0 ;
//Global variables for keyboard interface
struct Keyboard {
  int keypress;
  float pitch;
  float roll;
  float yaw;
  float thrust;
  int version;
};
Keyboard* shared_memory;
int run_program=1;
int pause=1;
//Global variables for pwm motor control
int pwm;

//add constants
#define PWM_MAX 1900
#define frequency 25000000.0
#define LED0 0x6
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4
//Values for pwm
float motor0_PWM = 0 ;
float motor1_PWM = 0 ;
float motor2_PWM = 0 ;
float motor3_PWM = 0 ;
float pitch_vel = 0 ;
float pitch_I_term = 0 ;
float roll_vel = 0 ;
float roll_I_term = 0 ;
float z_accel_sum = 0;
int z_accel_counts = 0;

float beat=0 ;

//global variables for joystick control
float thrust = 0 ;
float pitch_thrust = 0 ;
float roll_thrust = 0 ;
float yaw_thrust = 0 ;

void trap(int signal) {
	   printf("Keyboard timeout\n\r");
     kill_motors();
	   run_program=0;
}

int main (int argc, char *argv[])
{

    init_shared_memory();

    setup_imu();
    init_pwm();
  	init_motor(0);
   	init_motor(1);
  	init_motor(2);
  	init_motor(3);
    calibrate_imu();
  	delay(1000);
    setup_keyboard();
    signal(SIGINT , &trap);
    Keyboard keyboard = *shared_memory ;
    int count=0 ;
    float ht_thrust = 0; 
    float ht_yaw = 0; 
    float ht_pitch = 0; 
    float ht_roll = 0;
    float ht_key = 0;
    Position local_p = *position;
    prev_z = local_p.z;


    while(run_program==1)
    {
      // To refresh Vive data
      Position local_p = *position;
      //printf("%f %f %f %f %d \n\r" , local_p.x, local_p.y , local_p.z , local_p.yaw, local_p.version) ; 
      
      //Store the current pitch/roll before it is updated in update_filter()
      float prev_pitch = comp_pitch ;
      float prev_roll = comp_roll ;
  	  read_imu();
  	  update_filter();
      get_joystick();
      get_vive() ; 
  	  pid_update(pause);
  	  safety_check();

  	  //Store the value of current - previous pitch in global pitch_vel variable
  	  pitch_vel = comp_pitch-prev_pitch ;
      //Store the value of the current - previous roll in global roll_vel variable
      roll_vel = comp_roll-prev_roll ;

      //Dealing with inputs from joystick
  	  Keyboard keyboard = *shared_memory ; 
      
      //Kill the motors and end the program if 'A' is pressed
      if(keyboard.keypress == ' ') {
        kill_motors() ;
        run_program = 0 ;
      }

      //IMU calibration when 'Y' is pressed 
      else if(keyboard.keypress == '#') {
        calibrate_imu() ;
      }

      //Kill the motors and pause the PID update when 'B' is pressed
      else if(keyboard.keypress == '!') {
        kill_motors() ;
        pause = 0 ;
      }

      //Un-pause when 'X' is pressed
      else if(keyboard.keypress == '"') {
        pause = 1 ;
      }
      //Heartbeat timeout code, does not enter checks if pause is activated
      //Start by updating the keyboard
      timespec_get(&te,TIME_UTC);
      vive_time_curr = te.tv_nsec;
      js_time_curr = te.tv_nsec;

      if(pause == 1) { 
        /*
        if(keyboard.keypress!=ht_key || keyboard.thrust!=ht_thrust || keyboard.yaw!=ht_yaw || keyboard.pitch!=ht_pitch || keyboard.roll != ht_roll){
          js_time_prev = js_time_curr;

          ht_thrust = keyboard.thrust; 
          ht_yaw = keyboard.yaw; 
          ht_pitch = keyboard.pitch; 
          ht_roll = keyboard.roll;
          ht_key = keyboard.keypress; 
        }
        else if(keyboard.keypress==ht_key && keyboard.thrust==ht_thrust && keyboard.yaw==ht_yaw && keyboard.pitch==ht_pitch && keyboard.roll == ht_roll) {
          float js_time_diff = js_time_curr - js_time_prev ;

          if(js_time_diff<=0) {
            js_time_diff+=1000000000 ; 
          }

          js_time_diff = js_time_diff/1000000000 ;

          if(js_time_diff >= .5) { 
          kill_motors();
          printf("Ẁe lost the joystick!\n\r");
          run_program = 0;
          }
      
        }
        */

        if(local_p.version != prev_version) {
          vive_time_prev = vive_time_curr;
          prev_version = local_p.version;
        }
        else if(local_p.version == prev_version) { 

          float vive_time_diff = vive_time_curr - vive_time_prev ;

          if(vive_time_diff<=0) {
            vive_time_diff+=1000000000 ; 
          }

          vive_time_diff = vive_time_diff/1000000000 ;

          if(vive_time_diff >= .5) { 
          kill_motors();
          printf("Cannot See Vive sensor!\n\r");
          run_program = 0;
          } 

        }
      }

      if (abs(local_p.x) >= 1000 || abs(local_p.y) >= 1000)
      {
        kill_motors();
        printf("Outside of Vive Sensor!\n\r");
        run_program = 0;
      }

  	}
  	kill_motors() ; //final kill switch in case values have been stored to memory
  	return 0 ;
}
void get_vive() 
{
	Position local_p = *position;


  //Yaw Gain
	float Py_vive = 75;
  // Roll gains
	float Pr_vive = .08;
	float Dr_vive = 1.1;

  //Pitch Gains
  float Pp_vive = .08;
  float Dp_vive = 1.1;

  // Thrust Gains
  float Pz_vive = .07;
	float Iz_vive = .003;
  float Dz_vive = 2; 
	float z_height = 4200;
	float z_accel = accel_z_calibration + imu_data[5];

	// Vive Filter Updates
	vive_x_estimate = vive_x_estimate*.8 + local_p.x*.2;
  vive_y_estimate = vive_y_estimate*.8 + local_p.y*.2;
 
  //Computing Yaw
	vive_yaw = local_p.yaw*Py_vive;

  //Computing P roll, pitch and thrust
	float vive_roll_p = local_p.x*Pr_vive;
  float vive_pitch_p = local_p.y*Pp_vive;
  float vive_thrust_p = (local_p.z - z_height)*Pz_vive;

  //Computing D roll and pitch 
  z_accel_sum += z_accel;
  z_accel_counts += 1;

	if(local_p.version != vive_prev_version) {
    z_accel_average = z_accel_sum/(float)z_accel_counts; 
    z_accel_sum = 0;
    z_accel_counts = 0;

		vive_roll_d = Dr_vive*(vive_x_estimate-prev_x) ;
    vive_pitch_d = Dp_vive*(vive_y_estimate-prev_y) ;
    velocity_estimate = (velocity_estimate + z_accel_average*50.0)*.9 + (local_p.z - prev_z)*.1 ; 
    vive_thrust_d = Dz_vive*velocity_estimate ;

	}
  	prev_x =  vive_x_estimate ;
    prev_y =  vive_y_estimate ; 



  // Computing Thrust I term
    thrust_I_term += Iz_vive*(local_p.z - z_height);
  	if(thrust_I_term > thrust_I_limit) {
  		thrust_I_term = thrust_I_limit ;
  	}
  	else if(thrust_I_term <  -thrust_I_limit) {
  		thrust_I_term = - thrust_I_limit ;
  	}

  	//Desired roll/pitch
  	//printf("%f \n\r" , vive_pitch_d);
  	vive_roll = vive_roll_p + vive_roll_d;
    vive_pitch = vive_pitch_p + vive_pitch_d;
    vive_thrust = vive_thrust_p + thrust_I_term + vive_thrust_d;

    vive_prev_version = local_p.version;
    prev_z = local_p.z; 


}

void get_joystick()
{
  Keyboard keyboard = *shared_memory ;

  float neutral_power = 1400 ;
  float max_thrust = 150.0 ;
  float max_pitch = 80.0 ;
  float max_roll = 80.0;
  float max_yaw = 100.0 ;

  thrust = neutral_power - ((keyboard.thrust-128.0)*max_thrust)/112.0 ;
  pitch_thrust = -((keyboard.pitch-128.0)*max_pitch)/112.0 ;
  roll_thrust = -((keyboard.roll-128.0)*max_roll)/112.0 ;
  yaw_thrust = ((keyboard.yaw-128.0)*max_yaw)/112.0 ;
  //printf("%f\n\r" , yaw_thrust) ;
}

void kill_motors()
{
  set_PWM(0, 1000);
  set_PWM(1, 1000);
  set_PWM(2, 1000);
  set_PWM(3, 1000);
}

void pid_update(int pause)
{
  if(pause == 0) {
    kill_motors() ;
  }
  else if(pause == 1) {
    //Gains and max_vars for pitch PID control
  	float Dp = 220;
  	float Pp = 10;
  	float Ip = 0.09;
  	float pitch_I_limit = 100;

    //Gains and max_vars for roll PID control;
    float Dr = 220;
    float Pr = 9;
    float Ir = 0.09 ;
    float roll_I_limit = 100 ;

    //Gains for yaw control
    float Py = .6;


    float max_power = 1900 ;

  	//Calculating the pitch_I_term at each time step and upper bounding it
  	pitch_I_term += Ip*comp_pitch ;
  	if(pitch_I_term > pitch_I_limit) {
  		pitch_I_term = pitch_I_limit ;
  	}
  	else if(pitch_I_term <  -pitch_I_limit) {
  		pitch_I_term = - pitch_I_limit ;
  	}

    //Calculating the roll_I_term at each time step and upper bounding it
    roll_I_term += Ir*comp_roll ;
    if(roll_I_term > roll_I_limit) {
      roll_I_term = roll_I_limit ;
    }
    else if(roll_I_term <  -roll_I_limit) {
      roll_I_term = - roll_I_limit ;
    }

    float desired_roll = roll_thrust*.5 + vive_roll*.5 ;
    float desired_pitch = pitch_thrust*.5 + vive_pitch*.5 ; 
    float desired_thrust = thrust + vive_thrust; 
  
    //printf("%f %f %f %f \n\r" , motor0_PWM , motor1_PWM ,motor2_PWM, motor3_PWM);
  	//motor0_PWM = (thrust-pitch_thrust) + comp_pitch*Pp + pitch_I_term + pitch_vel*Dp ;
    //motor0_PWM = (thrust+roll_thrust) - comp_roll*Pr - roll_I_term - roll_vel*Dr ;
    motor0_PWM = (desired_thrust+desired_roll-desired_pitch-yaw_thrust) + comp_pitch*Pp + pitch_I_term + pitch_vel*Dp - comp_roll*Pr - roll_I_term - roll_vel*Dr + Py*imu_data[2] - vive_yaw;
    if(motor0_PWM > max_power) {
      motor0_PWM = max_power ;
    }
  	//motor1_PWM = (thrust+pitch_thrust) - comp_pitch*Pp - pitch_I_term - pitch_vel*Dp ;
    //motor1_PWM = (thrust+roll_thrust) - comp_roll*Pr - roll_I_term - roll_vel*Dr ;
    motor1_PWM = (desired_thrust+desired_roll +desired_pitch+yaw_thrust) - comp_pitch*Pp - pitch_I_term - pitch_vel*Dp - comp_roll*Pr - roll_I_term - roll_vel*Dr - Py*imu_data[2] +vive_yaw;
    if(motor1_PWM > max_power) {
      motor1_PWM = max_power ;
    }
  	//motor2_PWM = (thrust-pitch_thrust) + comp_pitch*Pp + pitch_I_term + pitch_vel*Dp ;
    //motor2_PWM = (thrust-roll_thrust) + comp_roll*Pr + roll_I_term + roll_vel*Dr ;
    motor2_PWM = (desired_thrust-desired_roll -desired_pitch+yaw_thrust) + comp_pitch*Pp + pitch_I_term + pitch_vel*Dp + comp_roll*Pr + roll_I_term + roll_vel*Dr - Py*imu_data[2] + vive_yaw;
    if(motor2_PWM > max_power) {
      motor2_PWM = max_power ;
    }
  	//motor3_PWM = (thrust+pitch_thrust) - comp_pitch*Pp - pitch_I_term - pitch_vel*Dp ;
    //motor3_PWM = (thrust-roll_thrust) + comp_roll*Pr + roll_I_term + roll_vel*Dr ;
    motor3_PWM = (desired_thrust-desired_roll+desired_pitch-yaw_thrust) - comp_pitch*Pp - pitch_I_term - pitch_vel*Dp + comp_roll*Pr + roll_I_term + roll_vel*Dr + Py*imu_data[2] - vive_yaw;
    if(motor3_PWM > max_power) {
      motor3_PWM = max_power ;
    }
    
    
    set_PWM(0, motor0_PWM);
  	set_PWM(1, motor1_PWM);
  	set_PWM(2, motor2_PWM);
  	set_PWM(3, motor3_PWM);
  
  
  
 /*
    printf("%f %f %f %f \n\r", motor0_PWM , motor1_PWM , motor2_PWM , motor3_PWM);
     	
    set_PWM(0, 1000);
  	set_PWM(1, 1000);
  	set_PWM(2, 1000);
  	set_PWM(3, 1000);

  	*/
 
  }

}

void safety_check() {
/*
	if(abs(imu_data[0])>300.0 || abs(imu_data[1])>300.0 || abs(imu_data[2])>300.0) {
    	kill_motors();
  		run_program=0 ;
  		printf("Gyro values exceeded\n\r");
	}
  */
  /*
	else if(abs(imu_data[3])>1.8 || abs(imu_data[4])>1.8 || abs(imu_data[5])>1.8){
		kill_motors();
		run_program=0;
		printf("Acceleration values exceeded\n\r");
	}
*/
/*
	else if(abs(imu_data[3])<.25 && abs(imu_data[4])<.25 && abs(imu_data[5])<.25){
		kill_motors() ;
		run_program=0;
		printf("All accelerometer values below limit\n\r");
	}
*/
	if(comp_roll>45.0 || comp_roll<-45.0) {
   		kill_motors();
  		run_program=0;
  		printf("Roll angle limit exceeded\n\r");
	}

	else if(comp_pitch>45.0 || comp_pitch<-45.0) {
    	kill_motors();
  		run_program=0 ;
  		printf("Pitch angle limit exceeded\n\r");
	}

}
void calibrate_imu() {
  //gyro roll_calibration
  //loops runs 1000 times and averages out the values and stores in the declared
  //floats
  float calib_x =0;
  float calib_y =0;
  float calib_z =0;
  float calib_pitch=0;
  float calib_roll=0;
  float calib_z_accel = 0;
  int i = 0 ;
  while(i<1000) {
    read_imu() ;
    calib_x = calib_x + imu_data[0] ;
    calib_y = calib_y + imu_data[1] ;
    calib_z = calib_z + imu_data[2] ;
    calib_pitch = calib_pitch + pitch_angle;
    calib_roll = calib_roll + roll_angle;
    calib_z_accel = calib_z_accel + imu_data[5];
    i++ ;
  }
  x_gyro_calibration = -calib_x/(float)i;
  y_gyro_calibration = -calib_y/(float)i;
  z_gyro_calibration = -calib_z/(float)i;
  pitch_calibration = -calib_pitch/(float)i;
  roll_calibration = -calib_roll/(float)i;
  accel_z_calibration = -calib_z_accel/(float)i;

  /*
  roll_calibration=??
  pitch_calibration=??
  accel_z_calibration=??
  */
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address= 59;//todo: set address value for accel x value
  float ax=0;
  float az=0;
  float ay=0;
  int vh,vl;

  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[3]=(float)vw*(2.0/32768.0);//  todo: convert vw from raw values to "g's"



  address=61;//todo: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[4]=(float)vw*(2.0/32768.0);//Todo: convert vw from raw valeus to "g's"


  address=63;//todo: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[5]=(float)vw*(2.0/32768.0);//todo: convert vw from raw values to g's


  address=67;//todo: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[0]=x_gyro_calibration+(float)vw*(500.0/32768.0);////todo: convert vw from raw values to degrees/second


  address=69;//todo: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
 imu_data[1]=y_gyro_calibration+(float)vw*(500.0/32768.0);////todo: convert vw from raw values to degrees/second


  address=71;////todo: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }
  imu_data[2]=z_gyro_calibration+(float)vw*(500.0/32768.0);////todo: convert vw from raw values to degrees/second

  // Calculating the pitch and roll angles

  pitch_angle = pitch_calibration - atan2(imu_data[4], -imu_data[5])*(180/M_PI);
  roll_angle = roll_calibration - atan2(imu_data[3], -imu_data[5])*(180/M_PI);

}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;

  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;

  //comp. filter for roll, pitch here:
  //First comp filter for roll

  float A = .009;
  roll_gyro_delta = imu_data[1]*imu_diff ;
  comp_roll = roll_angle*A+(1-A)*(-imu_data[1]*imu_diff+comp_roll) ;
  //Comp filter for pitch
  pitch_gyro_delta = imu_data[0]*imu_diff ;
  comp_pitch = pitch_angle*A+(1-A)*(imu_data[0]*imu_diff+comp_pitch) ;


}



//when cntrl+c pressed, kill motors


void setup_keyboard()
{
  int segment_id;
  struct shmid_ds shmbuffer;
  int segment_size;
  const int shared_segment_size = 0x6400;
  int smhkey=33222;
  /* Allocate a shared memory segment.  */
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
  /* Attach the shared memory segment.  */
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0);
  printf ("shared memory attached at address %p\n", shared_memory);
  /* Determine the segment's size. */
  shmctl (segment_id, IPC_STAT, &shmbuffer);
  segment_size  =               shmbuffer.shm_segsz;
  printf ("segment size: %d\n", segment_size);
  /* Write a string to the shared memory segment.  */
  //sprintf (shared_memory, "test!!!!.");
}

int setup_imu()
{
  wiringPiSetup ();


  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address

  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {

    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));

    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS


    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

void init_pwm()
{

    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);

    }
    else
    {

      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}



void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}


void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000)
    {
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}
