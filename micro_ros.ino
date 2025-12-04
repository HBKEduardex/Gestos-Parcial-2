#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <string.h>

// =========================
// PINES (TU HARDWARE)
// =========================
#define LED_AVANZAR    23
#define LED_RETRO      22
#define LED_IZQUIERDA  21
#define LED_DERECHA    19
#define LED_PARAR      18
#define LED_EXTRA      13    // indicador EMERGENCIA

#define BTN_EMERGENCIA 5     // Botón: GPIO5 <-> GND

// =========================
// CONSTANTES DE CONTROL
// =========================
#define LINEAR_SPEED   0.15   // m/s
#define ANGULAR_SPEED  0.5    // rad/s
#define TIMEOUT_MS     1000   // 1s sin gestos -> STOP

// =========================
// MACROS DE LA GUÍA
// =========================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    // podrías parpadear un LED aquí si quieres
    delay(100);
  }
}

// =========================
// OBJETOS micro-ROS
// =========================
rcl_publisher_t cmd_vel_pub;
rcl_publisher_t emergency_pub;
rcl_subscription_t gesture_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t safety_timer;

geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__String gesture_msg;
std_msgs__msg__String emergency_msg;

unsigned long last_cmd_ms = 0;
bool emergency_active = false;

// =========================
// FUNCIONES DE LEDs
// =========================
void apagarTodosLeds()
{
  digitalWrite(LED_AVANZAR, LOW);
  digitalWrite(LED_RETRO, LOW);
  digitalWrite(LED_IZQUIERDA, LOW);
  digitalWrite(LED_DERECHA, LOW);
  digitalWrite(LED_PARAR, LOW);
  digitalWrite(LED_EXTRA, LOW);
}

void setLedsForCommand(const char * cmd)
{
  apagarTodosLeds();

  if (strcmp(cmd, "forward") == 0)
  {
    digitalWrite(LED_AVANZAR, HIGH);
  }
  else if (strcmp(cmd, "backward") == 0)
  {
    digitalWrite(LED_RETRO, HIGH);
  }
  else if (strcmp(cmd, "left") == 0)
  {
    digitalWrite(LED_IZQUIERDA, HIGH);
  }
  else if (strcmp(cmd, "right") == 0)
  {
    digitalWrite(LED_DERECHA, HIGH);
  }
  else if (strcmp(cmd, "stop") == 0)
  {
    digitalWrite(LED_PARAR, HIGH);
  }
}

// =========================
// COMPARACIÓN ROBUSTA DE STRINGS
// =========================
bool equals_msg(const std_msgs__msg__String * msg, const char * txt)
{
  size_t n = strlen(txt);
  if (msg->data.size != n) return false;
  return (strncmp(msg->data.data, txt, n) == 0);
}

// =========================
void fill_twist_for_text(const char * cmd)
{
  // reset
  twist_msg.linear.x  = 0.0;
  twist_msg.linear.y  = 0.0;
  twist_msg.linear.z  = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  if (strcmp(cmd, "forward") == 0)
  {
    twist_msg.linear.x = LINEAR_SPEED;
  }
  else if (strcmp(cmd, "backward") == 0)
  {
    twist_msg.linear.x = -LINEAR_SPEED;
  }
  else if (strcmp(cmd, "left") == 0)
  {
    twist_msg.angular.z = ANGULAR_SPEED;
  }
  else if (strcmp(cmd, "right") == 0)
  {
    twist_msg.angular.z = -ANGULAR_SPEED;
  }
  // stop queda todo en cero
}

// =========================
// CALLBACK de /gesture_command
// =========================
void gesture_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *) msgin;

  // DEBUG: imprimir exactamente lo que llega
  Serial.print("[ESP32] Recibido /gesture_command: '");
  for (size_t i = 0; i < msg->data.size; i++)
  {
    Serial.print(msg->data.data[i]);
  }
  Serial.println("'");

  // Si ya hay emergencia activa, ignorar gestos
  if (emergency_active)
  {
    Serial.println("[ESP32] Emergencia activa, ignorando gesto.");
    return;
  }

  const char * cmd = "stop";

  if (equals_msg(msg, "forward"))   cmd = "forward";
  else if (equals_msg(msg, "backward")) cmd = "backward";
  else if (equals_msg(msg, "left"))     cmd = "left";
  else if (equals_msg(msg, "right"))    cmd = "right";
  else if (equals_msg(msg, "stop"))     cmd = "stop";

  Serial.print("[ESP32] Interpretado comando: ");
  Serial.println(cmd);

  // actualizar timer de seguridad
  last_cmd_ms = millis();

  // generar twist + leds
  fill_twist_for_text(cmd);
  setLedsForCommand(cmd);

  // publicar /cmd_vel
  RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &twist_msg, NULL));
}

// =========================
// TIMER: seguridad + emergencia
// =========================
void safety_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) timer;
  (void) last_call_time;

  // 1) Leer botón emergencia
  int lectura = digitalRead(BTN_EMERGENCIA);

  if (!emergency_active && lectura == LOW)
  {
    // debounce
    delay(30);
    if (digitalRead(BTN_EMERGENCIA) == LOW)
    {
      // Activar emergencia
      emergency_active = true;

      // STOP inmediato
      fill_twist_for_text("stop");
      setLedsForCommand("stop");
      digitalWrite(LED_EXTRA, HIGH);   // LED EXTRA = emergencia

      // publicar /cmd_vel
      RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &twist_msg, NULL));

      // publicar /emergency_status
      const char * txt = "EMERGENCY_ACTIVE";
      emergency_msg.data.size = strlen(txt);
      memcpy(emergency_msg.data.data, txt, emergency_msg.data.size);
      RCSOFTCHECK(rcl_publish(&emergency_pub, &emergency_msg, NULL));

      // esperar a que suelten el botón
      while (digitalRead(BTN_EMERGENCIA) == LOW)
      {
        delay(10);
      }
    }
  }

  // 2) Timeout de seguridad (si NO hay emergencia)
  if (!emergency_active)
  {
    unsigned long now = millis();
    if (now - last_cmd_ms > TIMEOUT_MS)
    {
      fill_twist_for_text("stop");
      setLedsForCommand("stop");
      RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &twist_msg, NULL));
      last_cmd_ms = now;
    }
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  // Pines
  pinMode(LED_AVANZAR, OUTPUT);
  pinMode(LED_RETRO, OUTPUT);
  pinMode(LED_IZQUIERDA, OUTPUT);
  pinMode(LED_DERECHA, OUTPUT);
  pinMode(LED_PARAR, OUTPUT);
  pinMode(LED_EXTRA, OUTPUT);
  apagarTodosLeds();

  pinMode(BTN_EMERGENCIA, INPUT_PULLUP);

  // Transporte micro-ROS (serial por USB)
  set_microros_transports();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Init support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Nodo
  RCCHECK(rclc_node_init_default(
    &node,
    "microros_gesture_controller",
    "",
    &support));

  // Publisher /cmd_vel
  RCCHECK(rclc_publisher_init_default(
    &cmd_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Publisher /emergency_status
  RCCHECK(rclc_publisher_init_default(
    &emergency_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/emergency_status"));

  // Subscriber /gesture_command
  RCCHECK(rclc_subscription_init_default(
    &gesture_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/gesture_command"));

  // Timer de seguridad (100 ms)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &safety_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    safety_timer_callback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &gesture_sub,
    &gesture_msg,
    &gesture_callback,
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &safety_timer));

  // Buffers para mensajes String
  gesture_msg.data.data = (char *) malloc(100 * sizeof(char));
  gesture_msg.data.size = 0;
  gesture_msg.data.capacity = 100;

  emergency_msg.data.data = (char *) malloc(50 * sizeof(char));
  emergency_msg.data.size = 0;
  emergency_msg.data.capacity = 50;

  last_cmd_ms = millis();

  Serial.println("micro-ROS gesture controller inicializado!");
}

// =========================
// LOOP
// =========================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  delay(10);
}
