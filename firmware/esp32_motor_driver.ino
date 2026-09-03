// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h>   //Predefined ROS 2 message type
#include <rmw_microros/rmw_microros.h> //ROS Middleware for Micro-ROS, provides functions for interfacing Micro-ROS with DDS.
#include <stdio.h>                //Standard I/O library for debugging.
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/joint_state.h>
#include <math.h>

//Declare nodes to be used
rcl_node_t node;            //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Data structure that holds the execution context of Micro-ROS, including its communication state, memory management, and initialization data.
rcl_allocator_t allocator;  //Manages memory allocation.

//Declare Subscribers to be used
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

//Declare Messages to be used
sensor_msgs__msg__JointState msg_sub;
sensor_msgs__msg__JointState msg_pub;

//Define Macros to be used
//Executes fn and returns false if it fails.
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
// Executes fn, but ignores failures.
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
//  Executes a given statement (X) periodically every MS milliseconds
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

//Defines State Machine States
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state = WAITING_AGENT;

//Specifies GPIO pin 13 for controlling an LED
#define PHASEA_GPIO 14
#define PHASEB_GPIO 13
#define IN1 25
#define IN2 26
#define PWM_PIN 27 //DEFINE PWM_PIN
#define PWM_CHNL 0    //Define Channel

#define IN3 32
#define IN4 33
#define PWM_PIN2 4
#define PHASEA_GPIO2 16
#define PHASEB_GPIO2 17
#define PWM_CHNL2 1

#define RPM_MAX 134.0        // RPM máximas de salida del gearmotor
#define MAX_RAD_S 14.0
#define PULSES_PER_REV 495   // Resolucion del encoder

#define PWM_FRQ 980 //Define PWM Frequency
#define PWM_RES 8  //Define PWM Resolution
#define PWM_MAX        255

float setPoint_L = 0.0;
float setPoint_R = 0.0;
float magL = 0.0;
float magR = 0.0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;
float r = 0.05;   // radio rueda
float L = 0.19;   // distancia entre ruedas

//Create entity functions
bool create_entities();
void destroy_entities();

//Define callbacks
float left_vel;
float right_vel;
void subscription_callback(const void * msgin)
{  
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  if(msg->velocity.size < 2) return;
  left_vel  = msg->velocity.data[0];
  right_vel = msg->velocity.data[1];
  setPoint_L = constrain(left_vel / MAX_RAD_S, -1.0, 1.0);
  setPoint_R = constrain(right_vel / MAX_RAD_S, -1.0, 1.0);
  magL = abs(setPoint_L);
  magR = abs(setPoint_R);
   
}

// Encoder
volatile long countPhaseA = 0;
long pulseCount = 0;
volatile long countPhaseA2 = 0;
long pulseCount2 = 0;

// Velocidad
float rpm_raw = 0.0;
float rpm = 0.0;
float rpm_filt = 0.0;
float rpm_raw2 = 0.0;
float rpm2 = 0.0;
float rpm_filt2 = 0.0;
  
const float alphaRPM = 0.20; 

// Tiempo de muestreo
const float Ts = 0.1; // 100 ms
float clampf(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}
// ISR Encoder
volatile int dir = 1;
volatile int dir2 = 1;
void IRAM_ATTR isr() {
  if(digitalRead(PHASEB_GPIO)){
    dir = 1;
  }else{
    dir = -1;
  }countPhaseA++;
}
void IRAM_ATTR isr2() {
  if(digitalRead(PHASEB_GPIO2)){
    dir2 = 1;
  }else{
    dir2 = -1;
  }
  countPhaseA2++;
}

void control_function(){
  noInterrupts();
  pulseCount = countPhaseA;
  countPhaseA = 0;
  interrupts();

  noInterrupts();
  pulseCount2 = countPhaseA2;
  countPhaseA2 = 0;
  interrupts();

  //RPM 
  rpm_raw = (pulseCount * 60.0) / (PULSES_PER_REV * Ts);
  rpm_raw = clampf(rpm_raw,-RPM_MAX,RPM_MAX);
  rpm_filt = alphaRPM * rpm_raw + (1.0 -alphaRPM) * rpm_filt;
  rpm = rpm_filt * dir;

  rpm_raw2 = (pulseCount2 * 60.0) / (PULSES_PER_REV * Ts);
  rpm_raw2 = clampf(rpm_raw2,-RPM_MAX,RPM_MAX);
  rpm_filt2 = alphaRPM * rpm_raw2 + (1.0 -alphaRPM) * rpm_filt2;
  rpm2 = rpm_filt2 * dir2;

  //CONVERTIR A PWM
  uint32_t dutyL = (uint32_t)(magL * PWM_MAX);
  uint32_t dutyR = (uint32_t)(magR * PWM_MAX);
  if (fabs(setPoint_L) < 0.05) dutyL = 0;
  if (fabs(setPoint_R) < 0.05) dutyR = 0;

  // Controlar dirección
  if(setPoint_L > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if(setPoint_L < 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    dutyL = 0;
  }
  if(setPoint_R > 0){
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }
  else if(setPoint_R < 0){
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
  }
  else{
    dutyR = 0;
  }
  ledcWrite(PWM_CHNL, dutyL);
  ledcWrite(PWM_CHNL2, dutyR);
  msg_pub.velocity.data[0] = (rpm * 2.0 * M_PI) / 60.0; 
  msg_pub.velocity.data[1] = (rpm2 * 2.0 * M_PI) / 60.0;
  RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
}

//Setup
void setup() {
  set_microros_transports(); // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  
  // --- Dentro del setup() ---
  sensor_msgs__msg__JointState__init(&msg_sub);
  // 1. Rservar espacio para los nombres (Gazebo siempre los envía)
  msg_sub.name.capacity = 2; // Dos nombres de joints
  msg_sub.name.data = (rosidl_runtime_c__String*) malloc(msg_sub.name.capacity * sizeof(rosidl_runtime_c__String));
  // Inicializar cada string individualmente para que el RMW tenga donde escribir
  for(size_t i = 0; i < msg_sub.name.capacity; i++){
      msg_sub.name.data[i].capacity = 20; // Tamaño max de caracteres por nombre
      msg_sub.name.data[i].data = (char*) malloc(msg_sub.name.data[i].capacity * sizeof(char));
      msg_sub.name.data[i].size = 0;
  }
  // 2. Reservar espacio para los arreglos numéricos
  msg_sub.velocity.capacity = 2;
  msg_sub.velocity.data = (double*) malloc(msg_sub.velocity.capacity * sizeof(double));
  msg_sub.position.capacity = 2;
  msg_sub.position.data = (double*) malloc(msg_sub.position.capacity * sizeof(double));
  // Si no vas a usar effort, déjalo en 0 para ahorrar RAM
  msg_sub.effort.capacity = 0;
  // ... (tu código anterior de set_microros_transports y msg_sub) ...

  // Inicializar mensaje de publicación (velocidades reales)
  sensor_msgs__msg__JointState__init(&msg_pub);
  msg_pub.velocity.capacity = 2;
  msg_pub.velocity.data = (double*) malloc(msg_pub.velocity.capacity * sizeof(double));
  msg_pub.velocity.size = 2;

// CAMBIO EN SETUP PARA MSG_PUB:
  msg_pub.name.capacity = 2;
  msg_pub.name.data = (rosidl_runtime_c__String*) malloc(msg_pub.name.capacity * sizeof(rosidl_runtime_c__String));
  msg_pub.name.size = 2;

  // Asignar nombres correctamente
  msg_pub.name.data[0].data = (char*)"left_wheel_actual";
  msg_pub.name.data[0].size = strlen(msg_pub.name.data[0].data);
  msg_pub.name.data[0].capacity = msg_pub.name.data[0].size + 1;

  msg_pub.name.data[1].data = (char*)"right_wheel_actual";
  msg_pub.name.data[1].size = strlen(msg_pub.name.data[1].data);
  msg_pub.name.data[1].capacity = msg_pub.name.data[1].size + 1;


  //Setup Microcontroller Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
    // Encoder
  pinMode(PHASEA_GPIO, INPUT_PULLUP);
  pinMode(PHASEB_GPIO, INPUT_PULLUP);
  pinMode(PHASEA_GPIO2, INPUT_PULLUP);
  pinMode(PHASEB_GPIO2, INPUT_PULLUP);


  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  //Setup the PWM
  ledcAttachPin(PWM_PIN, PWM_CHNL);       //Setup Attach the Pin to the Channel  
  ledcSetup(PWM_CHNL2, PWM_FRQ, PWM_RES);
  ledcAttachPin(PWM_PIN2, PWM_CHNL2);
  // Interrupción encoder
  attachInterrupt(digitalPinToInterrupt(PHASEA_GPIO), isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(PHASEA_GPIO2), isr2, FALLING);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHNL, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(PWM_CHNL2, 0);
}

void loop() {
  switch (state) {

    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(100,control_function());
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
      
    default:
      break;
  }
}

bool create_entities()
{
   //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

   //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_control", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "motor_speeds"));

  // create zero initialised executor (no configured) to avoid memory problems
  executor = rclc_executor_get_zero_initialized_executor();
  // Initializes the Micro-ROS Executor, which manages tasks and callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  // Register suscription with executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher,&node);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}
