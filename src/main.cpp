/**
 * Dual motor SimpleFOC PID tuner firmware.
 *
 * Serial protocol, newline terminated:
 *   MODE,0,angle
 *   TARGET,1,90
 *   PID,0,vel,P,0.015
 *   PID,1,angle,ramp,1000
 *   LIMIT,0,voltage,12
 *   LIMIT,1,velocity,20
 *   LIMIT,0,angleMin,10
 *   LIMIT,0,angleMax,170
 *   ENABLE,0,1
 *   ESTOP
 */

#include <SimpleFOC.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor parameters
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(6);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

#define UNDERVOLTAGE_THRES 11.1f
#define SERIAL_BAUD 230400
#define TELEMETRY_PERIOD_MS 50
#define BOARD_CHECK_PERIOD_MS 1000
#define COMMAND_BUFFER_SIZE 96
#define ANGLE_LIMIT_MIN_DEG 0.0f
#define ANGLE_LIMIT_MAX_DEG 360.0f
#define ANGLE_MIN_DEFAULT_DEG 190.0f
#define ANGLE_MAX_DEFAULT_DEG 360.0f

BLDCMotor *motors[] = {&motor, &motor1};
float target_angle_deg[] = {0.0f, 0.0f};
float target_velocity_rad_s[] = {0.0f, 0.0f};
float angle_min_deg[] = {ANGLE_MIN_DEFAULT_DEG, ANGLE_MIN_DEFAULT_DEG};
float angle_max_deg[] = {ANGLE_MAX_DEFAULT_DEG, ANGLE_MAX_DEFAULT_DEG};
bool requested_enable[] = {true, true};

uint32_t prev_board_check_millis = 0;
uint32_t prev_telemetry_millis = 0;
bool flag_under_voltage = false;
bool emergency_stop = false;
char command_buffer[COMMAND_BUFFER_SIZE];
uint8_t command_length = 0;

void board_check();
float get_vin_Volt();
void board_init();
void handle_serial();
void handle_command(char *line);
void send_telemetry();
void send_ack(const char *cmd);
void send_error(const char *cmd, const char *message);
void apply_enable_state();
BLDCMotor *select_motor(const char *token, int *motor_index);
bool equals_ignore_case(const char *left, const char *right);
bool parse_float(const char *token, float *value);
bool parse_bool(const char *token, bool *value);
const char *mode_name(BLDCMotor *selected_motor);
void print_pid(PIDController &pid);
void print_motor_json(int index);
float angle_rad_to_degrees_0_360(float angle_rad);
float degrees_to_radians(float angle_deg);
float clamp_float(float value, float minimum, float maximum);
float clamp_angle_limit_degrees(float angle_deg);
bool angle_within_limits(float angle_deg, int motor_index);
float circular_distance_degrees(float left, float right);
float clamp_angle_target_degrees(float angle_deg, int motor_index);
float motion_target_for_motor(int motor_index);
float telemetry_target_for_motor(int motor_index);

void setup()
{
  Serial.begin(SERIAL_BAUD);
  board_init();

  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL);  // AS5600_M1
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);

  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  driver.voltage_power_supply = get_vin_Volt();
  driver.init();

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();

  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.10f;
  motor1.PID_velocity.P = 0.250f;
  motor.PID_velocity.I = 0.80f;
  motor1.PID_velocity.I = 0.75f;

  motor.P_angle.P = 3.0f;
  motor1.P_angle.P = 5.0f;

  motor.voltage_limit = get_vin_Volt();
  motor1.voltage_limit = get_vin_Volt();
  motor.PID_velocity.limit = motor.voltage_limit;
  motor1.PID_velocity.limit = motor1.voltage_limit;

  motor.LPF_velocity.Tf = 0.01f;
  motor1.LPF_velocity.Tf = 0.01f;

  motor.velocity_limit = 5.0f;
  motor1.velocity_limit = 100.0f;
  motor.P_angle.limit = motor.velocity_limit;
  motor1.P_angle.limit = motor1.velocity_limit;

  motor.init();
  motor1.init();

  motor.initFOC();
  motor1.initFOC();
  apply_enable_state();

  Serial.println(F("{\"type\":\"ready\",\"message\":\"dual_pid_tuner\"}"));
}

void loop()
{
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(motion_target_for_motor(0));
  motor1.move(motion_target_for_motor(1));

  board_check();
  handle_serial();
  send_telemetry();
}

void board_init()
{
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12); // 12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES)
  {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("{\"type\":\"wait_power\",\"vin\":%.2f}\n", VIN_Volt);
  }
  Serial.printf("{\"type\":\"calibrating\",\"vin\":%.2f}\n", VIN_Volt);
}

float get_vin_Volt()
{
  return analogReadMilliVolts(13) * 8.5f / 1000.0f;
}

void board_check()
{
  uint32_t curr_millis = millis();

  if (curr_millis - prev_board_check_millis < BOARD_CHECK_PERIOD_MS)
  {
    return;
  }

  float vin_Volt = get_vin_Volt();
  bool now_under_voltage = vin_Volt < UNDERVOLTAGE_THRES;

  if (now_under_voltage)
  {
    uint8_t count = 5;
    while (count--)
    {
      vin_Volt = get_vin_Volt();
      if (vin_Volt > UNDERVOLTAGE_THRES)
      {
        now_under_voltage = false;
        break;
      }
    }
  }

  flag_under_voltage = now_under_voltage;
  apply_enable_state();
  prev_board_check_millis = curr_millis;
}

void apply_enable_state()
{
  for (uint8_t i = 0; i < 2; i++)
  {
    if (flag_under_voltage || emergency_stop || !requested_enable[i])
    {
      motors[i]->disable();
    }
    else if (!motors[i]->enabled)
    {
      motors[i]->enable();
    }
  }
}

void handle_serial()
{
  while (Serial.available())
  {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r')
    {
      if (command_length > 0)
      {
        command_buffer[command_length] = '\0';
        handle_command(command_buffer);
        command_length = 0;
      }
      continue;
    }

    if (command_length < COMMAND_BUFFER_SIZE - 1)
    {
      command_buffer[command_length++] = ch;
    }
    else
    {
      command_length = 0;
      send_error("BUFFER", "command too long");
    }
  }
}

void handle_command(char *line)
{
  char original[COMMAND_BUFFER_SIZE];
  strncpy(original, line, sizeof(original));
  original[sizeof(original) - 1] = '\0';

  char *cmd = strtok(line, ",");
  if (!cmd)
  {
    send_error("EMPTY", "missing command");
    return;
  }

  if (equals_ignore_case(cmd, "ESTOP"))
  {
    emergency_stop = true;
    requested_enable[0] = false;
    requested_enable[1] = false;
    apply_enable_state();
    send_ack(original);
    return;
  }

  char *motor_token = strtok(NULL, ",");
  int motor_index = -1;
  BLDCMotor *selected_motor = select_motor(motor_token, &motor_index);
  if (!selected_motor)
  {
    send_error(original, "invalid motor index");
    return;
  }

  if (equals_ignore_case(cmd, "MODE"))
  {
    char *mode = strtok(NULL, ",");
    if (equals_ignore_case(mode, "angle") || equals_ignore_case(mode, "position"))
    {
      selected_motor->controller = MotionControlType::angle;
      target_angle_deg[motor_index] = clamp_angle_target_degrees(target_angle_deg[motor_index], motor_index);
      send_ack(original);
    }
    else if (equals_ignore_case(mode, "velocity") || equals_ignore_case(mode, "speed"))
    {
      selected_motor->controller = MotionControlType::velocity;
      send_ack(original);
    }
    else
    {
      send_error(original, "invalid mode");
    }
    return;
  }

  if (equals_ignore_case(cmd, "TARGET"))
  {
    float value = 0.0f;
    if (!parse_float(strtok(NULL, ","), &value))
    {
      send_error(original, "invalid target");
      return;
    }

    if (selected_motor->controller == MotionControlType::angle)
    {
      target_angle_deg[motor_index] = clamp_angle_target_degrees(value, motor_index);
    }
    else if (selected_motor->controller == MotionControlType::velocity)
    {
      target_velocity_rad_s[motor_index] = value;
    }
    else
    {
      send_error(original, "unsupported target mode");
      return;
    }

    send_ack(original);
    return;
  }

  if (equals_ignore_case(cmd, "PID"))
  {
    char *loop = strtok(NULL, ",");
    char *param = strtok(NULL, ",");
    float value = 0.0f;
    if (!loop || !param || !parse_float(strtok(NULL, ","), &value))
    {
      send_error(original, "invalid pid command");
      return;
    }

    PIDController *pid = nullptr;
    if (equals_ignore_case(loop, "vel") || equals_ignore_case(loop, "velocity") || equals_ignore_case(loop, "speed"))
    {
      pid = &selected_motor->PID_velocity;
    }
    else if (equals_ignore_case(loop, "angle") || equals_ignore_case(loop, "position") || equals_ignore_case(loop, "pos"))
    {
      pid = &selected_motor->P_angle;
    }
    else
    {
      send_error(original, "invalid pid loop");
      return;
    }

    if (equals_ignore_case(param, "P"))
    {
      pid->P = value;
    }
    else if (equals_ignore_case(param, "I"))
    {
      pid->I = value;
    }
    else if (equals_ignore_case(param, "D"))
    {
      pid->D = value;
    }
    else if (equals_ignore_case(param, "limit"))
    {
      pid->limit = value;
      if (pid == &selected_motor->P_angle)
      {
        selected_motor->velocity_limit = value;
      }
    }
    else if (equals_ignore_case(param, "ramp") || equals_ignore_case(param, "output_ramp"))
    {
      pid->output_ramp = value;
    }
    else
    {
      send_error(original, "invalid pid parameter");
      return;
    }

    send_ack(original);
    return;
  }

  if (equals_ignore_case(cmd, "LIMIT"))
  {
    char *limit_name = strtok(NULL, ",");
    float value = 0.0f;
    if (!limit_name || !parse_float(strtok(NULL, ","), &value))
    {
      send_error(original, "invalid limit command");
      return;
    }

    if (equals_ignore_case(limit_name, "voltage") || equals_ignore_case(limit_name, "volt"))
    {
      selected_motor->voltage_limit = value;
      selected_motor->PID_velocity.limit = value;
    }
    else if (equals_ignore_case(limit_name, "velocity") || equals_ignore_case(limit_name, "speed"))
    {
      selected_motor->velocity_limit = value;
      selected_motor->P_angle.limit = value;
    }
    else if (equals_ignore_case(limit_name, "angleMin") || equals_ignore_case(limit_name, "angle_min") || equals_ignore_case(limit_name, "minAngle") || equals_ignore_case(limit_name, "min_angle"))
    {
      angle_min_deg[motor_index] = clamp_angle_limit_degrees(value);
      target_angle_deg[motor_index] = clamp_angle_target_degrees(target_angle_deg[motor_index], motor_index);
    }
    else if (equals_ignore_case(limit_name, "angleMax") || equals_ignore_case(limit_name, "angle_max") || equals_ignore_case(limit_name, "maxAngle") || equals_ignore_case(limit_name, "max_angle"))
    {
      angle_max_deg[motor_index] = clamp_angle_limit_degrees(value);
      target_angle_deg[motor_index] = clamp_angle_target_degrees(target_angle_deg[motor_index], motor_index);
    }
    else if (equals_ignore_case(limit_name, "ramp") || equals_ignore_case(limit_name, "output_ramp"))
    {
      selected_motor->PID_velocity.output_ramp = value;
    }
    else
    {
      send_error(original, "invalid limit");
      return;
    }

    send_ack(original);
    return;
  }

  if (equals_ignore_case(cmd, "ENABLE"))
  {
    bool enabled = false;
    if (!parse_bool(strtok(NULL, ","), &enabled))
    {
      send_error(original, "invalid enable value");
      return;
    }

    emergency_stop = false;
    requested_enable[motor_index] = enabled;
    apply_enable_state();
    send_ack(original);
    return;
  }

  send_error(original, "unknown command");
}

void send_telemetry()
{
  uint32_t curr_millis = millis();
  if (curr_millis - prev_telemetry_millis < TELEMETRY_PERIOD_MS)
  {
    return;
  }

  Serial.print(F("{\"type\":\"tel\",\"vin\":"));
  Serial.print(get_vin_Volt(), 2);
  Serial.print(F(",\"underVoltage\":"));
  Serial.print(flag_under_voltage ? 1 : 0);
  Serial.print(F(",\"estop\":"));
  Serial.print(emergency_stop ? 1 : 0);
  Serial.print(F(",\"motors\":["));
  print_motor_json(0);
  Serial.print(',');
  print_motor_json(1);
  Serial.println(F("]}"));

  prev_telemetry_millis = curr_millis;
}

void print_motor_json(int index)
{
  BLDCMotor *selected_motor = motors[index];
  Serial.print(F("{\"index\":"));
  Serial.print(index);
  Serial.print(F(",\"enabled\":"));
  Serial.print(selected_motor->enabled ? 1 : 0);
  Serial.print(F(",\"requested\":"));
  Serial.print(requested_enable[index] ? 1 : 0);
  Serial.print(F(",\"mode\":\""));
  Serial.print(mode_name(selected_motor));
  Serial.print(F("\",\"target\":"));
  Serial.print(telemetry_target_for_motor(index), 4);
  Serial.print(F(",\"angle\":"));
  Serial.print(angle_rad_to_degrees_0_360(selected_motor->shaft_angle), 2);
  Serial.print(F(",\"rawAngleRad\":"));
  Serial.print(selected_motor->shaft_angle, 4);
  Serial.print(F(",\"velocity\":"));
  Serial.print(selected_motor->shaft_velocity, 4);
  Serial.print(F(",\"velocityPID\":"));
  print_pid(selected_motor->PID_velocity);
  Serial.print(F(",\"anglePID\":"));
  print_pid(selected_motor->P_angle);
  Serial.print(F(",\"limits\":{\"voltage\":"));
  Serial.print(selected_motor->voltage_limit, 4);
  Serial.print(F(",\"velocity\":"));
  Serial.print(selected_motor->velocity_limit, 4);
  Serial.print(F(",\"ramp\":"));
  Serial.print(selected_motor->PID_velocity.output_ramp, 4);
  Serial.print(F(",\"angleMin\":"));
  Serial.print(angle_min_deg[index], 2);
  Serial.print(F(",\"angleMax\":"));
  Serial.print(angle_max_deg[index], 2);
  Serial.print(F("}}"));
}

void print_pid(PIDController &pid)
{
  Serial.print(F("{\"P\":"));
  Serial.print(pid.P, 6);
  Serial.print(F(",\"I\":"));
  Serial.print(pid.I, 6);
  Serial.print(F(",\"D\":"));
  Serial.print(pid.D, 6);
  Serial.print(F(",\"limit\":"));
  Serial.print(pid.limit, 4);
  Serial.print(F(",\"output_ramp\":"));
  Serial.print(pid.output_ramp, 4);
  Serial.print('}');
}

void send_ack(const char *cmd)
{
  Serial.print(F("{\"type\":\"ack\",\"cmd\":\""));
  Serial.print(cmd);
  Serial.println(F("\"}"));
}

void send_error(const char *cmd, const char *message)
{
  Serial.print(F("{\"type\":\"err\",\"cmd\":\""));
  Serial.print(cmd ? cmd : "");
  Serial.print(F("\",\"message\":\""));
  Serial.print(message);
  Serial.println(F("\"}"));
}

BLDCMotor *select_motor(const char *token, int *motor_index)
{
  if (!token || !motor_index)
  {
    return nullptr;
  }

  char *end = nullptr;
  long index = strtol(token, &end, 10);
  if (end == token || *end != '\0')
  {
    return nullptr;
  }

  if (index < 0 || index > 1)
  {
    return nullptr;
  }

  *motor_index = (int)index;
  return motors[*motor_index];
}

bool equals_ignore_case(const char *left, const char *right)
{
  if (!left || !right)
  {
    return false;
  }

  while (*left && *right)
  {
    if (tolower((unsigned char)*left) != tolower((unsigned char)*right))
    {
      return false;
    }
    left++;
    right++;
  }

  return *left == '\0' && *right == '\0';
}

bool parse_float(const char *token, float *value)
{
  if (!token || !value)
  {
    return false;
  }

  char *end = nullptr;
  float parsed = strtof(token, &end);
  if (end == token || *end != '\0' || !isfinite(parsed))
  {
    return false;
  }

  *value = parsed;
  return true;
}

bool parse_bool(const char *token, bool *value)
{
  if (!token || !value)
  {
    return false;
  }

  if (equals_ignore_case(token, "1") || equals_ignore_case(token, "true") || equals_ignore_case(token, "on"))
  {
    *value = true;
    return true;
  }

  if (equals_ignore_case(token, "0") || equals_ignore_case(token, "false") || equals_ignore_case(token, "off"))
  {
    *value = false;
    return true;
  }

  return false;
}

const char *mode_name(BLDCMotor *selected_motor)
{
  switch (selected_motor->controller)
  {
  case MotionControlType::velocity:
    return "velocity";
  case MotionControlType::angle:
    return "angle";
  default:
    return "other";
  }
}

float angle_rad_to_degrees_0_360(float angle_rad)
{
  float normalized = fmodf(angle_rad, _2PI);
  if (normalized < 0.0f)
  {
    normalized += _2PI;
  }
  return normalized * 180.0f / _PI;
}

float degrees_to_radians(float angle_deg)
{
  return angle_deg * _PI / 180.0f;
}

float clamp_float(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }

  if (value > maximum)
  {
    return maximum;
  }

  return value;
}

float clamp_angle_limit_degrees(float angle_deg)
{
  return clamp_float(angle_deg, ANGLE_LIMIT_MIN_DEG, ANGLE_LIMIT_MAX_DEG);
}

bool angle_within_limits(float angle_deg, int motor_index)
{
  float minimum = angle_min_deg[motor_index];
  float maximum = angle_max_deg[motor_index];

  if (minimum <= maximum)
  {
    return angle_deg >= minimum && angle_deg <= maximum;
  }

  return angle_deg >= minimum || angle_deg <= maximum;
}

float circular_distance_degrees(float left, float right)
{
  float distance = fabsf(left - right);
  while (distance > ANGLE_LIMIT_MAX_DEG)
  {
    distance -= ANGLE_LIMIT_MAX_DEG;
  }

  if (distance > 180.0f)
  {
    distance = ANGLE_LIMIT_MAX_DEG - distance;
  }

  return distance;
}

float clamp_angle_target_degrees(float angle_deg, int motor_index)
{
  float clamped = clamp_angle_limit_degrees(angle_deg);
  if (angle_within_limits(clamped, motor_index))
  {
    return clamped;
  }

  float minimum = angle_min_deg[motor_index];
  float maximum = angle_max_deg[motor_index];
  if (minimum <= maximum)
  {
    return clamp_float(clamped, minimum, maximum);
  }

  return circular_distance_degrees(clamped, minimum) <= circular_distance_degrees(clamped, maximum) ? minimum : maximum;
}

float motion_target_for_motor(int motor_index)
{
  if (motors[motor_index]->controller == MotionControlType::angle)
  {
    return degrees_to_radians(clamp_angle_target_degrees(target_angle_deg[motor_index], motor_index));
  }

  return target_velocity_rad_s[motor_index];
}

float telemetry_target_for_motor(int motor_index)
{
  if (motors[motor_index]->controller == MotionControlType::angle)
  {
    return target_angle_deg[motor_index];
  }

  return target_velocity_rad_s[motor_index];
}
