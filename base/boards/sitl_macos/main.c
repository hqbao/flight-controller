#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <SDL2/SDL.h>
#include <signal.h>
#include <platform.h>
#include <pubsub.h>
#include <messages.h>
#include <stdarg.h>
#include <rc_receiver.h>

// Defined in module_sim.c
void modules_setup(void);

#define LOOP_RATE_US 1000 // 1000Hz

static volatile int keep_running = 1;

void handle_signal(int sig) {
    (void)sig;
    keep_running = 0;
}

// Implement platform_console for SITL
void platform_console(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}

// ---------------------------------------------------------
// HAL IMPLEMENTATION (SITL)
// ---------------------------------------------------------

void platform_toggle_led(char led) {
    (void)led;
}

void platform_delay(uint32_t ms) {
    SDL_Delay(ms);
}

uint32_t platform_time_ms(void) {
    return SDL_GetTicks();
}

// Storage (File based)
// Saved in the Current Working Directory (CWD) of the executable
static const char *STORAGE_FILE = "sitl_storage.bin";

char platform_storage_read(uint16_t start, uint16_t size, uint8_t *data) {
    FILE *f = fopen(STORAGE_FILE, "rb");
    if (!f) return PLATFORM_ERROR;
    fseek(f, start, SEEK_SET);
    size_t read = fread(data, 1, size, f);
    fclose(f);
    return (read == size) ? PLATFORM_OK : PLATFORM_ERROR;
}

char platform_storage_write(uint16_t start, uint16_t size, uint8_t *data) {
    FILE *f = fopen(STORAGE_FILE, "r+b");
    if (!f) {
        f = fopen(STORAGE_FILE, "wb");
        if (!f) return PLATFORM_ERROR;
    }
    fseek(f, start, SEEK_SET);
    size_t written = fwrite(data, 1, size, f);
    fclose(f);
    return (written == size) ? PLATFORM_OK : PLATFORM_ERROR;
}

// UART
char platform_uart_send(uart_port_t port, uint8_t *data, uint16_t data_size) {
    (void)port; (void)data; (void)data_size;
    return PLATFORM_OK;
}

// Stubs for drivers
char platform_i2c_write_read_dma(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size, uint8_t *output, uint16_t output_size) { 
    (void)port; (void)address; (void)input; (void)input_size; (void)output; (void)output_size;
    return PLATFORM_OK; 
}
char platform_i2c_write_read(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size, uint8_t *output, uint16_t output_size, uint32_t timeout) { 
    (void)port; (void)address; (void)input; (void)input_size; (void)output; (void)output_size; (void)timeout;
    return PLATFORM_OK; 
}
char platform_i2c_read(i2c_port__t port, uint8_t address, uint8_t *output, uint16_t output_size) { 
    (void)port; (void)address; (void)output; (void)output_size;
    return PLATFORM_OK; 
}
char platform_i2c_write(i2c_port__t port, uint8_t address, uint8_t *input, uint16_t input_size) { 
    (void)port; (void)address; (void)input; (void)input_size;
    return PLATFORM_OK; 
}
char platform_spi_write(spi_port_t spi_port, uint8_t *input, uint8_t size) { 
    (void)spi_port; (void)input; (void)size;
    return PLATFORM_OK; 
}
char platform_spi_write_read(spi_port_t spi_port, uint8_t *input, uint16_t input_size, uint8_t *output, uint16_t output_size) { 
    (void)spi_port; (void)input; (void)input_size; (void)output; (void)output_size;
    return PLATFORM_OK; 
}

// PWM/DShot Stubs
char platform_pwm_init(pwm_port_t port) { (void)port; return PLATFORM_OK; }
char platform_pwm_send(pwm_port_t port, uint32_t data) { (void)port; (void)data; return PLATFORM_OK; }
char platform_dshot_init(dshot_port_t port) { (void)port; return PLATFORM_OK; }
char platform_dshot_send(dshot_port_t port, uint16_t data) { (void)port; (void)data; return PLATFORM_OK; }
char platform_dshot_ex_init(dshot_ex_port_t port) { (void)port; return PLATFORM_OK; }
char platform_dshot_ex_send(dshot_ex_port_t port, uint32_t data) { (void)port; (void)data; return PLATFORM_OK; }


// ---------------------------------------------------------
// RECEPTION & INJECTION TYPES
// ---------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    double timestamp;
    double accel[3];
    double gyro[3];
    double mag[3];
    double pressure;
    double range;
    double gps[3];
    double flow[2];
    double gps_vel[3]; // Add VELOCITY x, y, z (Ned, m/s)
} SensorPacket;

typedef struct {
    float motors[4];
} MotorPacket;
#pragma pack(pop)

// Globals
int sockfd;
struct sockaddr_in sim_addr;
struct sockaddr_in my_addr;
SDL_Joystick *joystick = NULL;

// Global buffer for RC Data Bus message
// Format matched to h7v1: [Cmd, SetPoint, xx, xx, Roll(4), Pitch(4), Yaw(4), Alt(4), State(1), Mode(1)]
static uint8_t g_rc_db_msg_payload[22] = {0x02, 0x00, 0, 0x0f};

// Axis values -1.0 to 1.0 (Throttle 0.0 to 1.0)
float axis_roll = 0.0f;
float axis_pitch = 0.0f;
float axis_throttle = 0.0f;
float axis_yaw = 0.0f;
float axis_ch5 = -1.0f; // Default Disarmed
float axis_ch6 = -1.0f; // Default Manual/Stabilize

void on_speed_control(uint8_t *data, size_t size) {
    (void)size;
    int *speeds = (int*)data;

    // Send Motor Packet immediately
    // Normalize 0-2500 -> 0.0-1.0
    float scale = 1.0f / 2500.0f;
    
    MotorPacket motors;
    motors.motors[0] = (float)speeds[0] * scale;
    motors.motors[1] = (float)speeds[1] * scale;
    motors.motors[2] = (float)speeds[2] * scale;
    motors.motors[3] = (float)speeds[3] * scale;

    // Clamp
    for (int i = 0; i < 4; i++) {
        if (motors.motors[i] < 0.0f) motors.motors[i] = 0.0f;
        if (motors.motors[i] > 1.0f) motors.motors[i] = 1.0f;
    }

    sendto(sockfd, &motors, sizeof(MotorPacket), 0, (const struct sockaddr *)&sim_addr, sizeof(sim_addr));
}

void handle_sensor_injection(SensorPacket *sensors) {
    // Tick Counter for Scheduling
    static uint32_t sched_tick = 0;
    sched_tick++;

    // 1. High Rate Sensors (1kHz)
    float gyro[3] = { (float)sensors->gyro[0], (float)sensors->gyro[1], (float)sensors->gyro[2] };
    publish(SENSOR_IMU1_GYRO_UPDATE, (uint8_t*)gyro, sizeof(gyro));
    
    // 2. Medium-High Rate - Accel (500Hz)
    // Run on evens (0, 2, 4...)
    if (sched_tick % 2 == 0) {
        float accel[3] = { (float)sensors->accel[0], (float)sensors->accel[1], (float)sensors->accel[2] };
        publish(SENSOR_IMU1_ACCEL_UPDATE, (uint8_t*)accel, sizeof(accel));
    }

    // 3. Medium Rate - Mag (100Hz)
    if (sched_tick % 10 == 0) {
        vector3d_t mag_vec;
        mag_vec.x = sensors->mag[0];
        mag_vec.y = sensors->mag[1];
        mag_vec.z = sensors->mag[2];
        publish(SENSOR_COMPASS, (uint8_t*)&mag_vec, sizeof(mag_vec));
    }

    // 4. Medium Rate - Barometer & Optical Flow (250Hz - close approximation to 225Hz)
    // 1000 / 4 = 250Hz.
    // 225Hz is not an integer division of 1000Hz. 
    // Using 250Hz (Div 4) is the closest standard division.
    if (sched_tick % 4 == 0) {
        // Barometer
        double baro_alt_mm = sensors->gps[2] * 1000.0;
        publish(SENSOR_AIR_PRESSURE, (uint8_t*)&baro_alt_mm, sizeof(double));

        // Optical Flow
        optflow_data_t flow;
        flow.dx = sensors->flow[0];
        flow.dy = sensors->flow[1];
        flow.z = sensors->range * 1000.0;
        flow.dt_us = 4000; // 250Hz interval = 4000us
        flow.clarity = 100;
        flow.direction = OPTFLOW_DOWNWARD;
        publish(EXTERNAL_SENSOR_OPTFLOW, (uint8_t*)&flow, sizeof(optflow_data_t));
    }

    // 5. Low Rate - GPS (10Hz)
    if (sched_tick % 100 == 0) {
        gps_position_t gps_pos;
        gps_pos.lat = (int32_t)(sensors->gps[0] * 10000000.0);
        gps_pos.lon = (int32_t)(sensors->gps[1] * 10000000.0);
        gps_pos.alt = (int32_t)(sensors->gps[2] * 1000.0);
        publish(EXTERNAL_SENSOR_GPS, (uint8_t*)&gps_pos, sizeof(gps_position_t));

        gps_velocity_t gps_vel;
        // Map NED Velocity to Standard Frame (X=North, Y=East, Z=Down)
        // Simulation Bridge sends Ground Truth Velocity in ENU (East-North-Up) or Gazebo World Frame (XYZ)
        // Gazebo: X=Forward(East?), Y=Left(North?), Z=Up
        // We will normalize in bridge to be: [VelN, VelE, VelD]
        gps_vel.velN = (int32_t)(sensors->gps_vel[0] * 1000.0); // Convert m/s to mm/s
        gps_vel.velE = (int32_t)(sensors->gps_vel[1] * 1000.0);
        gps_vel.velD = (int32_t)(sensors->gps_vel[2] * 1000.0);
        publish(EXTERNAL_SENSOR_GPS_VELOC, (uint8_t*)&gps_vel, sizeof(gps_velocity_t));
    }

    // 5. System Scheduler (Simulating Hardware Timers)
    publish(SCHEDULER_1KHZ, NULL, 0);

    if (sched_tick % 2 == 0)    publish(SCHEDULER_500HZ, NULL, 0);
    if (sched_tick % 4 == 0)    publish(SCHEDULER_250HZ, NULL, 0);
    if (sched_tick % 10 == 0)   publish(SCHEDULER_100HZ, NULL, 0);
    if (sched_tick % 20 == 0)   publish(SCHEDULER_50HZ, NULL, 0);
    if (sched_tick % 40 == 0)   publish(SCHEDULER_25HZ, NULL, 0);
    if (sched_tick % 100 == 0)  publish(SCHEDULER_10HZ, NULL, 0);
    if (sched_tick % 200 == 0)  publish(SCHEDULER_5HZ, NULL, 0);
    if (sched_tick % 1000 == 0) {
        publish(SCHEDULER_1HZ, NULL, 0);
        sched_tick = 0;
    }
}

void handle_rc_injection(float roll_f, float pitch_f, float yaw_f, float throttle_f, float ch5_f, float ch6_f) {
    // Scaling increased to 620.0f (from 500.0f) to ensure full range (+/-90 deg) 
    // is reachable even if joystick axes don't reach full +/- 1.0 (circular restriction)
    int roll = (int)lroundf(roll_f * 620.0f);
    int pitch = (int)lroundf(pitch_f * 620.0f);
    int yaw = (int)lroundf(yaw_f * 620.0f);
    int alt = (int)lroundf(throttle_f * 620.0f);

    // Clamp values to prevent overflow beyond valid internal range [-500, +500]
    if (roll > 500) roll = 500; else if (roll < -500) roll = -500;
    if (pitch > 500) pitch = 500; else if (pitch < -500) pitch = -500;
    if (yaw > 500) yaw = 500; else if (yaw < -500) yaw = -500;
    if (alt > 500) alt = 500; else if (alt < -500) alt = -500;

    // Apply Deadbands
    if (roll <= -10) roll += 10; else if (roll >= 10) roll -= 10; else roll = 0;
    if (pitch <= -10) pitch += 10; else if (pitch >= 10) pitch -= 10; else pitch = 0;
    if (yaw <= -10) yaw += 10; else if (yaw >= 10) yaw -= 10; else yaw = 0;
    if (alt <= -10) alt += 10; else if (alt >= 10) alt -= 10; else alt = 0;

    // Channel 5: State (Arm/Disarm/Land)
    // Low = 0 (Disarmed), Mid = 1 (Armed), High = 2 (Landing)
    uint8_t state = 0; 
    if (ch5_f < -0.3f) state = 0;      // Disarmed
    else if (ch5_f < 0.3f) state = 1;  // Armed
    else state = 2;                    // Landing

    // Channel 6: Flight Mode
    // Low = 0 (Manual), Mid = 1 (AltCtl), High = 2 (PosCtl/Angle)
    uint8_t mode = 0; 
    if (ch6_f < -0.3f) mode = 0;
    else if (ch6_f < 0.3f) mode = 1;
    else mode = 2;

    // Construct Payload
    memcpy(&g_rc_db_msg_payload[4], (uint8_t*)&roll, sizeof(int));
    memcpy(&g_rc_db_msg_payload[8], (uint8_t*)&pitch, sizeof(int));
    memcpy(&g_rc_db_msg_payload[12], (uint8_t*)&yaw, sizeof(int));
    memcpy(&g_rc_db_msg_payload[16], (uint8_t*)&alt, sizeof(int));
    g_rc_db_msg_payload[20] = state;
    g_rc_db_msg_payload[21] = mode;

    // Inject into Flight Stack
    platform_receive_db_message(g_rc_db_msg_payload, 22);
}

void init_network() {
    // Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Set non-blocking
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    // Reuse Address/Port
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    #ifdef SO_REUSEPORT
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt)); 
    #endif

    // Bind to 45454 to receive SENSORS
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = INADDR_ANY;
    my_addr.sin_port = htons(45454);

    if (bind(sockfd, (const struct sockaddr *)&my_addr, sizeof(my_addr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    // Prepare destination address for MOTORS
    memset(&sim_addr, 0, sizeof(sim_addr));
    sim_addr.sin_family = AF_INET;
    sim_addr.sin_port = htons(45455);
    inet_pton(AF_INET, "127.0.0.1", &sim_addr.sin_addr);
}

void init_sdl() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        // Do not exit, allows running without joystick
        return;
    }

    if (SDL_NumJoysticks() > 0) {
        joystick = SDL_JoystickOpen(0);
    }
}

void process_input() {
    SDL_JoystickUpdate(); 

    if (joystick) {
        // Mapping: 0=LeftX(Yaw), 1=LeftY(Thr), 2=RightX(Roll), 3=RightY(Pitch)
        float raw0 = SDL_JoystickGetAxis(joystick, 0) / 32768.0f;
        float raw1 = SDL_JoystickGetAxis(joystick, 1) / 32768.0f;
        float raw2 = SDL_JoystickGetAxis(joystick, 2) / 32768.0f; 
        float raw3 = SDL_JoystickGetAxis(joystick, 3) / 32768.0f;

        axis_yaw = raw0;
        // Throttle Axis (User reported inversion with -raw1, so using raw1)
        axis_throttle = raw1; 
        axis_roll = raw2;     
        axis_pitch = raw3;
        
        // Channels 5 & 6 (Aux) - Check axes count first if possible, but safe to read usually
        // Note: SDL axis indices depend on controller. Usually 4=Ry, 5=RTrigger, etc.
        // Assuming standard mapping or Ch5/Ch6 axes.
        if (SDL_JoystickNumAxes(joystick) > 4) axis_ch5 = SDL_JoystickGetAxis(joystick, 4) / 32768.0f;
        if (SDL_JoystickNumAxes(joystick) > 5) axis_ch6 = SDL_JoystickGetAxis(joystick, 5) / 32768.0f;

    } else {
        // Try to reconnect
        if (SDL_NumJoysticks() > 0) {
            joystick = SDL_JoystickOpen(0);
        }
    }
    
    // Global Deadzone logic
    if (fabs(axis_roll) < 0.1) axis_roll = 0.0f;
    if (fabs(axis_pitch) < 0.1) axis_pitch = 0.0f;
    // axis_throttle deadzone
    if (fabs(axis_throttle) < 0.1) axis_throttle = 0.0f;
    
    // Clamp to -1.0 to 1.0
    if (axis_throttle < -1.0f) axis_throttle = -1.0f;
    if (axis_throttle > 1.0f) axis_throttle = 1.0f;

    // Inject RC data into the Platform Bus
    handle_rc_injection(axis_roll, axis_pitch, axis_yaw, axis_throttle, axis_ch5, axis_ch6);
}

int main(int argc, char **argv) {
    (void)argc; (void)argv;
    init_sdl();
    init_network();

    // 0. Initialize System
    modules_setup();
    subscribe(SPEED_CONTROL_UPDATE, on_speed_control);
    
    // Force state to FLYING to enable Control Loop
    state_t initial_state = FLYING;
    publish(STATE_DETECTION_UPDATE, (uint8_t*)&initial_state, sizeof(state_t));

    SensorPacket sensors;
    struct sockaddr_in recv_addr;
    socklen_t len = sizeof(recv_addr);

    // Catch Ctrl+C
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    while (keep_running) {
        process_input();

        // 1. Receive Sensors
        int n = recvfrom(sockfd, &sensors, sizeof(SensorPacket), 0, (struct sockaddr *)&recv_addr, &len);
        
        if (n == sizeof(SensorPacket)) {
            handle_sensor_injection(&sensors);
        }

        usleep(LOOP_RATE_US);
    }

    if (joystick) SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}
