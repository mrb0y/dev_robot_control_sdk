/******************************************************************************
 * @ Dev Robot Control SDK
 * @ Copyright (c) 2024 WEILAN.Co.Ltd. All Rights Reserved.
 ******************************************************************************/
#include "robot_control/bluetooth_remote_control.h"
#include "robot_control/fsm/fsm_state.h" // For FSMStateName enum
#include <iostream>
#include <unistd.h> // For close(), read(), write(), usleep()
#include <sys/socket.h>
#include <poll.h> // For poll()

BluetoothRemoteControl::BluetoothRemoteControl(const std::string &device_address)
    : device_address_(device_address), running_(false), socket_(-1), current_mode_index_(0)
{
    // Initialize command to zero/default
    cmd_mutex_.lock();
    cmd_.vx = 0.0f;
    cmd_.vy = 0.0f;
    cmd_.wz = 0.0f;
    cmd_.mode[0] = FSMStateName::PASSIVE; // Start in Passive mode
    cmd_.mode[1] = 0;
    cmd_.mode[2] = 0;
    cmd_mutex_.unlock();
}

BluetoothRemoteControl::~BluetoothRemoteControl()
{
    stop();
}

int BluetoothRemoteControl::start()
{
    if (running_) {
        return 0; // Already running
    }
    running_ = true;
    communication_thread_ = std::thread(&BluetoothRemoteControl::communication_thread_func, this);
    std::cout << "[BluetoothRemoteControl] Started communication thread." << std::endl;
    return 0;
}

void BluetoothRemoteControl::stop()
{
    if (!running_) {
        return; // Already stopped
    }
    running_ = false;
    if (communication_thread_.joinable())
    {
        communication_thread_.join();
    }
    if (socket_ != -1)
    {
        close(socket_);
        socket_ = -1;
    }
    std::cout << "[BluetoothRemoteControl] Stopped communication thread." << std::endl;
}

void BluetoothRemoteControl::communication_thread_func()
{
    struct sockaddr_rc addr = {0};
    int status;
    char buffer[1024]; // Buffer for reading input events

    while (running_)
    {
        // --- Connection Phase ---
        if (socket_ == -1) {
            std::cout << "[BluetoothRemoteControl] Attempting to connect to " << device_address_ << "..." << std::endl;
            // Allocate socket
            socket_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
            if (socket_ == -1) {
                perror("[BluetoothRemoteControl] Socket creation failed");
                usleep(5000000); // Wait 5 seconds before retrying
                continue;
            }

            // Set the connection parameters (who to connect to)
            addr.rc_family = AF_BLUETOOTH;
            addr.rc_channel = (uint8_t)1; // RFCOMM channel 1 is common for SPP/gamepads
            str2ba(device_address_.c_str(), &addr.rc_bdaddr);

            // Connect to server
            status = connect(socket_, (struct sockaddr *)&addr, sizeof(addr));

            if (status == 0) {
                std::cout << "[BluetoothRemoteControl] Connected successfully to " << device_address_ << "." << std::endl;
                // Set socket to non-blocking? Maybe not needed if using poll/select
            } else {
                perror("[BluetoothRemoteControl] Connection failed");
                close(socket_);
                socket_ = -1;
                usleep(5000000); // Wait 5 seconds before retrying
                continue;
            }
        }

        // --- Reading Phase ---
        struct pollfd pfd;
        pfd.fd = socket_;
        pfd.events = POLLIN;

        int poll_ret = poll(&pfd, 1, 1000); // Wait up to 1 second for data

        if (poll_ret < 0) {
            perror("[BluetoothRemoteControl] Poll error");
            close(socket_);
            socket_ = -1;
            // Set commands to zero on error
            cmd_mutex_.lock();
            cmd_.vx = 0.0f; cmd_.vy = 0.0f; cmd_.wz = 0.0f;
            cmd_mutex_.unlock();
            usleep(1000000); // Wait before trying to reconnect
            continue;
        } else if (poll_ret == 0) {
            // Timeout - no data received, maybe controller is idle or disconnected?
            // Optionally, could zero commands here if timeout persists
            continue;
        } else {
            // Data available
            if (pfd.revents & POLLIN) {
                status = read(socket_, buffer, sizeof(buffer) - 1);
                if (status > 0) {
                    buffer[status] = '\0'; // Null-terminate
                    // Process the received data
                    process_input(buffer, status);
                } else {
                    // Read error or connection closed
                    if (status == 0) {
                        std::cout << "[BluetoothRemoteControl] Connection closed by peer." << std::endl;
                    } else {
                        perror("[BluetoothRemoteControl] Read error");
                    }
                    close(socket_);
                    socket_ = -1;
                    // Set commands to zero on disconnect
                    cmd_mutex_.lock();
                    cmd_.vx = 0.0f; cmd_.vy = 0.0f; cmd_.wz = 0.0f;
                    // Optionally reset mode? Or keep last commanded mode? Let's keep it for now.
                    cmd_mutex_.unlock();
                    usleep(1000000); // Wait before trying to reconnect
                    continue;
                }
            } else {
                 // Other poll event (e.g., POLLHUP, POLLERR)
                 std::cerr << "[BluetoothRemoteControl] Poll event: " << pfd.revents << std::endl;
                 close(socket_);
                 socket_ = -1;
                 // Set commands to zero on error
                 cmd_mutex_.lock();
                 cmd_.vx = 0.0f; cmd_.vy = 0.0f; cmd_.wz = 0.0f;
                 cmd_mutex_.unlock();
                 usleep(1000000); // Wait before trying to reconnect
                 continue;
            }
        }
         usleep(10000); // Small delay to prevent busy-waiting if data comes very fast
    } // end while(running_)

    // Cleanup if loop exits
    if (socket_ != -1) {
        close(socket_);
        socket_ = -1;
    }
     std::cout << "[BluetoothRemoteControl] Communication thread finished." << std::endl;
}


// NOTE: This process_input is a placeholder and highly dependent
// on the specific Bluetooth controller's data format.
// It assumes a very simple hypothetical format for demonstration.
// Real implementation would require inspecting the actual data stream
// or using a proper joystick/HID library.
void BluetoothRemoteControl::process_input(const char* buffer, int len)
{
    // Example: Assume data is like "AXIS:1:15000\nBUTTON:9:1\n"
    // This parsing is extremely basic and likely incorrect for real devices.
    // A robust solution would use sscanf, regex, or a dedicated parser.

    // For simplicity, let's just simulate receiving some values
    // In a real scenario, parse 'buffer' based on controller protocol.

    // --- Placeholder Simulation ---
    static float sim_vx = 0.0f, sim_vy = 0.0f, sim_wz = 0.0f;
    static bool sim_start_pressed = false;

    // Simulate some axis changes based on buffer content (very crude)
    if (len > 5) { // Arbitrary check
        sim_vx = (buffer[0] % 5) * 0.1f - 0.2f; // Simulate some movement
        sim_vy = (buffer[1] % 3) * 0.1f - 0.1f;
        sim_wz = (buffer[2] % 7) * 0.1f - 0.3f;
    }
    // Simulate start button press (e.g., if buffer contains 'S')
    bool current_start_state = false;
    for(int i=0; i<len; ++i) {
        if (buffer[i] == 'S') { // Hypothetical 'Start' button signal
             current_start_state = true;
             break;
        }
    }

    // --- Update Command ---
    cmd_mutex_.lock();

    // Update velocities (apply scaling if needed)
    // Assuming axis values range from -AXIS_MAX to +AXIS_MAX
    // Map to desired robot velocity range (e.g., -0.5 to 0.5 m/s, -1.0 to 1.0 rad/s)
    // These scaling factors need tuning.
    const float max_linear_vel = 0.5f;
    const float max_angular_vel = 1.0f;

    // Example mapping (replace sim_ values with actual parsed axis values)
    // cmd_.vx = (parsed_axis_y / AXIS_MAX) * max_linear_vel;
    // cmd_.vy = (parsed_axis_x / AXIS_MAX) * max_linear_vel;
    // cmd_.wz = (parsed_axis_turn / AXIS_MAX) * max_angular_vel;
    cmd_.vx = sim_vx; // Using simulated values for now
    cmd_.vy = sim_vy;
    cmd_.wz = sim_wz;


    // Check for Start button press (rising edge detection)
    if (current_start_state && !sim_start_pressed) {
        cycle_mode(); // Cycle mode on button press
        std::cout << "[BluetoothRemoteControl] Mode cycled to: " << cmd_.mode[0] << std::endl;
    }
    sim_start_pressed = current_start_state; // Store current state for next check

    cmd_mutex_.unlock();
}

void BluetoothRemoteControl::cycle_mode()
{
    // Assumes cmd_mutex_ is already locked by caller (process_input)
    current_mode_index_ = (current_mode_index_ + 1) % 5; // Cycle through 0, 1, 2, 3, 4
    switch (current_mode_index_) {
        case 0: cmd_.mode[0] = FSMStateName::PASSIVE; break;
        case 1: cmd_.mode[0] = FSMStateName::LIE_DOWN; break;
        case 2: cmd_.mode[0] = FSMStateName::STAND_UP; break;
        case 3: cmd_.mode[0] = FSMStateName::RL_MODEL; break;
        case 4: cmd_.mode[0] = FSMStateName::SOFT_STOP; break;
    }
}
