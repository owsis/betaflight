#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <cmath>
#include <iostream>

namespace gazebo
{
  // Betaflight FDM packet structure (must match Betaflight's fdm_packet)
  struct fdm_packet {
    double timestamp;                      // in seconds
    double imu_angular_velocity_rpy[3];    // rad/s
    double imu_linear_acceleration_xyz[3]; // m/s/s NED, body frame
    double imu_orientation_quat[4];        // w, x, y, z
    double velocity_xyz[3];                // m/s, earth frame
    double position_xyz[3];                // meters, NED from origin
    double pressure;                       // Pa
    // GPS data
    double latitude;                       // degrees
    double longitude;                      // degrees
    double altitude_msl;                   // meters above sea level
    double gps_velocity_ned[3];            // m/s, NED frame
    uint8_t num_satellites;                // number of satellites
    uint8_t gps_fix_type;                  // 0=no fix, 2=2D, 3=3D
  };

  // Betaflight servo packet (PWM from Betaflight) - 16 bytes
  struct servo_packet {
    float motor_speed[4];  // 16 bytes: normal [0.0, 1.0], 3D [-1.0, 1.0]
  };

  class BetaflightPlugin : public ModelPlugin
  {
    private:
      // Gazebo
      physics::ModelPtr model;
      physics::LinkPtr base_link;
      event::ConnectionPtr updateConnection;
      common::Time last_time;

      // UDP sockets
      int sock_fdm;    // Socket for sending FDM data (to port 9003)
      int sock_pwm;    // Socket for receiving PWM data (from port 9002)
      struct sockaddr_in addr_fdm;
      struct sockaddr_in addr_pwm;

      // Motors
      std::vector<physics::JointPtr> joints;
      std::vector<std::string> joint_names;
      std::vector<double> motor_multipliers;

      // Data
      fdm_packet fdm;
      servo_packet latest_servo;    // Latest valid PWM data
      servo_packet stable_servo;    // Stable/filtered PWM data actually used
      servo_packet frozen_servo;    // Frozen values during extreme change detection
      bool has_valid_servo;         // Flag to indicate if we have valid PWM data
      int extreme_change_streak;    // Counter for consecutive extreme changes
      bool in_extreme_mode;         // Flag indicating we're in extreme change handling

      // Diagnostics - track previous values to detect sudden changes
      ignition::math::Vector3d prev_angular_vel;
      ignition::math::Vector3d prev_linear_accel;
      bool has_prev_sensor_data;

      // Settings
      std::string fdm_addr;
      int fdm_port_out;  // Port to send FDM data (9003)
      int fdm_port_in;   // Port to receive PWM data (9002)

    public:
      BetaflightPlugin() : ModelPlugin()
      {
        std::cout << "[BetaflightPlugin] Constructor" << std::endl;
        sock_fdm = -1;
        sock_pwm = -1;

        // Default settings
        fdm_addr = "127.0.0.1";
        fdm_port_out = 9003;  // Send FDM to Betaflight
        fdm_port_in = 9002;   // Receive PWM from Betaflight

        memset(&fdm, 0, sizeof(fdm));
        memset(&latest_servo, 0, sizeof(latest_servo));
        memset(&stable_servo, 0, sizeof(stable_servo));
        memset(&frozen_servo, 0, sizeof(frozen_servo));
        has_valid_servo = false;
        extreme_change_streak = 0;
        in_extreme_mode = false;
        has_prev_sensor_data = false;
      }

      ~BetaflightPlugin()
      {
        if (sock_fdm >= 0) close(sock_fdm);
        if (sock_pwm >= 0) close(sock_pwm);
      }

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        
        // Try to find base_link with different naming patterns
        std::vector<std::string> possible_names = {
          "base_link",
          "iris::base_link",
          "iris::iris::base_link",
          "iris::iris_demo::iris::base_link"
        };

        for (const auto& name : possible_names)
        {
          this->base_link = this->model->GetLink(name);
          if (this->base_link)
          {
            std::cout << "[BetaflightPlugin] Found base_link: " << name << std::endl;
            break;
          }
        }

        // If still not found, try to find any link with "base_link" in its name
        if (!this->base_link)
        {
          for (auto link : this->model->GetLinks())
          {
            std::string link_name = link->GetScopedName();
            if (link_name.find("base_link") != std::string::npos)
            {
              this->base_link = link;
              std::cout << "[BetaflightPlugin] Found base_link by search: " << link_name << std::endl;
              break;
            }
          }
        }

        if (!this->base_link)
        {
          gzerr << "[BetaflightPlugin] base_link not found! Available links:" << std::endl;
          for (auto link : this->model->GetLinks())
          {
            gzerr << "  - " << link->GetScopedName() << std::endl;
          }
          return;
        }

        std::cout << "[BetaflightPlugin] Using base_link: " << this->base_link->GetScopedName() << std::endl;

        // Parse SDF parameters
        if (_sdf->HasElement("fdm_addr"))
          fdm_addr = _sdf->Get<std::string>("fdm_addr");
        if (_sdf->HasElement("fdm_port_out"))
          fdm_port_out = _sdf->Get<int>("fdm_port_out");
        if (_sdf->HasElement("fdm_port_in"))
          fdm_port_in = _sdf->Get<int>("fdm_port_in");

        std::cout << "[BetaflightPlugin] FDM will be sent to " << fdm_addr << ":" << fdm_port_out << std::endl;
        std::cout << "[BetaflightPlugin] PWM will be received from " << fdm_addr << ":" << fdm_port_in << std::endl;

        // Setup motors
        SetupMotors(_sdf);

        // Initialize UDP sockets
        if (!InitUDP())
        {
          gzerr << "[BetaflightPlugin] Failed to initialize UDP" << std::endl;
          return;
        }

        // Connect to update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&BetaflightPlugin::OnUpdate, this));

        this->last_time = this->model->GetWorld()->SimTime();

        std::cout << "[BetaflightPlugin] Plugin loaded successfully" << std::endl;
      }

      void SetupMotors(sdf::ElementPtr _sdf)
      {
        // Parse motor control channels
        sdf::ElementPtr control = _sdf->GetElement("control");
        while (control)
        {
          std::string joint_name = control->Get<std::string>("jointName");
          double multiplier = control->Get<double>("multiplier");

          // Try to find joint with various naming patterns
          physics::JointPtr joint = this->model->GetJoint(joint_name);
          
          // If not found, try searching all joints for a match
          if (!joint)
          {
            // Extract the base joint name (e.g., "rotor_0_joint" from "iris::rotor_0_joint")
            std::string base_name = joint_name;
            size_t last_colon = joint_name.rfind("::");
            if (last_colon != std::string::npos)
            {
              base_name = joint_name.substr(last_colon + 2);
            }
            
            // Search through all joints
            for (auto j : this->model->GetJoints())
            {
              std::string j_name = j->GetScopedName();
              if (j_name.find(base_name) != std::string::npos)
              {
                joint = j;
                std::cout << "[BetaflightPlugin] Found joint by search: " << j_name 
                          << " (looking for: " << joint_name << ")" << std::endl;
                break;
              }
            }
          }

          if (joint)
          {
            joints.push_back(joint);
            joint_names.push_back(joint_name);
            motor_multipliers.push_back(multiplier);
            std::cout << "[BetaflightPlugin] Motor " << joints.size()-1 << ": "
                      << joint->GetScopedName() << " (multiplier: " << multiplier << ")" << std::endl;
          }
          else
          {
            gzerr << "[BetaflightPlugin] Joint not found: " << joint_name << std::endl;
            gzerr << "[BetaflightPlugin] Available joints:" << std::endl;
            for (auto j : this->model->GetJoints())
            {
              gzerr << "  - " << j->GetScopedName() << std::endl;
            }
          }

          control = control->GetNextElement("control");
        }

        std::cout << "[BetaflightPlugin] Configured " << joints.size() << " motors" << std::endl;
      }

      bool InitUDP()
      {
        // Create socket for sending FDM data
        sock_fdm = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fdm < 0)
        {
          gzerr << "[BetaflightPlugin] Failed to create FDM socket: " << strerror(errno) << std::endl;
          return false;
        }

        memset(&addr_fdm, 0, sizeof(addr_fdm));
        addr_fdm.sin_family = AF_INET;
        addr_fdm.sin_port = htons(fdm_port_out);
        inet_pton(AF_INET, fdm_addr.c_str(), &addr_fdm.sin_addr);

        // Create socket for receiving PWM data
        sock_pwm = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_pwm < 0)
        {
          gzerr << "[BetaflightPlugin] Failed to create PWM socket: " << strerror(errno) << std::endl;
          return false;
        }

        // Set non-blocking
        int flags = fcntl(sock_pwm, F_GETFL, 0);
        fcntl(sock_pwm, F_SETFL, flags | O_NONBLOCK);

        memset(&addr_pwm, 0, sizeof(addr_pwm));
        addr_pwm.sin_family = AF_INET;
        addr_pwm.sin_port = htons(fdm_port_in);
        addr_pwm.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock_pwm, (struct sockaddr*)&addr_pwm, sizeof(addr_pwm)) < 0)
        {
          gzerr << "[BetaflightPlugin] Failed to bind PWM socket to port " << fdm_port_in
                << ": " << strerror(errno) << std::endl;
          return false;
        }

        std::cout << "[BetaflightPlugin] UDP sockets initialized successfully" << std::endl;
        std::cout << "[BetaflightPlugin] - FDM send socket ready (to " << fdm_addr << ":" << fdm_port_out << ")" << std::endl;
        std::cout << "[BetaflightPlugin] - PWM recv socket bound to port " << fdm_port_in << " (listening...)" << std::endl;
        std::cout << "[BetaflightPlugin] - PWM socket set to NON-BLOCKING mode" << std::endl;
        std::cout << "[BetaflightPlugin] - Expected PWM packet size: " << sizeof(servo_packet) << " bytes" << std::endl;
        return true;
      }

      void OnUpdate()
      {
        if (!this->base_link) return;

        common::Time current_time = this->model->GetWorld()->SimTime();
        double dt = (current_time - this->last_time).Double();

        // Update at ~250Hz
        if (dt < 0.004) return;

        this->last_time = current_time;

        // Receive PWM data from Betaflight
        ReceivePWM();

        // Get state from Gazebo and fill FDM packet
        UpdateFDMPacket();

        // Send FDM packet to Betaflight
        SendFDM();

        // Apply motor commands
        ApplyMotorCommands();
      }

      void ReceivePWM()
      {
        servo_packet temp_servo;
        static int recv_count = 0;
        static int error_count = 0;
        static int invalid_count = 0;
        static int extreme_change_count = 0;

        // Read all available packets to get the most recent one
        // This prevents using stale data when multiple packets are queued
        int packets_read = 0;
        bool got_valid_packet = false;
        servo_packet newest_servo;

        while (packets_read < 10)  // Safety limit to prevent infinite loop
        {
          ssize_t n = recv(sock_pwm, &temp_servo, sizeof(temp_servo), 0);

          if (n == sizeof(temp_servo))
          {
            recv_count++;
            packets_read++;

            // Validate motor values are in reasonable range
            bool valid = true;
            for (int i = 0; i < 4; i++)
            {
              if (std::isnan(temp_servo.motor_speed[i]) || std::isinf(temp_servo.motor_speed[i]))
              {
                valid = false;
                break;
              }
              // Allow slightly outside [0,1] range for safety, but reject extreme values
              if (temp_servo.motor_speed[i] < -0.1 || temp_servo.motor_speed[i] > 1.1)
              {
                valid = false;
                break;
              }
            }

            // Check for extreme changes that might indicate a crash/flip
            if (valid && has_valid_servo)
            {
              bool extreme_change = false;
              for (int i = 0; i < 4; i++)
              {
                float change = std::abs(temp_servo.motor_speed[i] - latest_servo.motor_speed[i]);
                // If any motor changes by more than 40% in one step, it's suspicious
                if (change > 0.4)
                {
                  extreme_change = true;
                  break;
                }
              }

              if (extreme_change)
              {
                extreme_change_count++;
                if (extreme_change_count <= 10) {
                  std::cerr << "[BetaflightPlugin] WARNING: Extreme motor change detected! "
                            << "Old: [" << latest_servo.motor_speed[0] << ", "
                            << latest_servo.motor_speed[1] << ", "
                            << latest_servo.motor_speed[2] << ", "
                            << latest_servo.motor_speed[3] << "] -> New: ["
                            << temp_servo.motor_speed[0] << ", "
                            << temp_servo.motor_speed[1] << ", "
                            << temp_servo.motor_speed[2] << ", "
                            << temp_servo.motor_speed[3] << "]" << std::endl;
                }
                // Still accept it, but log the warning
              }
            }

            if (valid)
            {
              // Keep the newest valid packet
              newest_servo = temp_servo;
              got_valid_packet = true;
            }
            else
            {
              invalid_count++;
              if (invalid_count < 10) {
                std::cerr << "[BetaflightPlugin] Invalid PWM values received: "
                          << temp_servo.motor_speed[0] << ", "
                          << temp_servo.motor_speed[1] << ", "
                          << temp_servo.motor_speed[2] << ", "
                          << temp_servo.motor_speed[3] << std::endl;
              }
            }
          }
          else if (n < 0)
          {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
              // No more data available - this is normal for non-blocking socket
              break;
            }
            else
            {
              error_count++;
              if (error_count < 5) {
                std::cerr << "[BetaflightPlugin] recv() error: " << strerror(errno) << std::endl;
              }
              break;
            }
          }
          else if (n > 0)
          {
            std::cerr << "[BetaflightPlugin] Received partial packet: " << n
                      << " bytes (expected " << sizeof(temp_servo) << ")" << std::endl;
            break;
          }
          else
          {
            // n == 0: connection closed
            break;
          }
        }

        // Update latest_servo with the newest valid packet received in this cycle
        if (got_valid_packet)
        {
          // Log raw data received from SITL
          static int raw_log_count = 0;
          if (++raw_log_count % 250 == 0 || recv_count < 10) {
            std::cout << "[BetaflightPlugin] RAW PWM from SITL: ["
                      << newest_servo.motor_speed[0] << ", "
                      << newest_servo.motor_speed[1] << ", "
                      << newest_servo.motor_speed[2] << ", "
                      << newest_servo.motor_speed[3] << "]" << std::endl;
          }
          
          // Apply smoothing/filtering to prevent extreme changes
          if (!has_valid_servo)
          {
            // First time receiving data - accept immediately
            latest_servo = newest_servo;
            stable_servo = newest_servo;
            has_valid_servo = true;
            extreme_change_streak = 0;
            
            std::cout << "[BetaflightPlugin] First PWM accepted: ["
                      << stable_servo.motor_speed[0] << ", "
                      << stable_servo.motor_speed[1] << ", "
                      << stable_servo.motor_speed[2] << ", "
                      << stable_servo.motor_speed[3] << "]" << std::endl;
          }
          else
          {
            // DISABLED EXTREME CHANGE DETECTION - accept all valid packets immediately
            // Previous filtering was causing 20ms lag → FC overcompensation → oscillation
            latest_servo = newest_servo;
            stable_servo = newest_servo;
            
            // For debugging, still detect extreme changes but don't block them
            servo_packet& baseline = stable_servo;
            bool is_extreme_change = false;
            float max_change = 0.0;
            for (int i = 0; i < 4; i++)
            {
              float change = std::abs(newest_servo.motor_speed[i] - baseline.motor_speed[i]);
              if (change > max_change) max_change = change;
              if (change > 0.5)  // Higher threshold just for logging
              {
                is_extreme_change = true;
              }
            }

            if (is_extreme_change)
            {
              // Just log it, don't block
              std::cout << "[BetaflightPlugin] Large change detected (max Δ=" << max_change 
                        << ") - ACCEPTING immediately" << std::endl;
            }
          }

          if (recv_count % 250 == 0) {
            std::cout << "[BetaflightPlugin] PWM packets received: " << recv_count
                      << " | motors: " << newest_servo.motor_speed[0] << ", "
                      << newest_servo.motor_speed[1] << ", "
                      << newest_servo.motor_speed[2] << ", "
                      << newest_servo.motor_speed[3];
            if (packets_read > 1) {
              std::cout << " | (" << packets_read << " packets in queue)";
            }
            std::cout << std::endl;
          }
        }
      }

      void UpdateFDMPacket()
      {
        // Timestamp
        fdm.timestamp = this->model->GetWorld()->SimTime().Double();

        // Get pose and velocity in world frame
        ignition::math::Pose3d pose = this->base_link->WorldPose();
        ignition::math::Vector3d linear_vel = this->base_link->WorldLinearVel();
        ignition::math::Vector3d angular_vel = this->base_link->RelativeAngularVel();
        ignition::math::Vector3d linear_accel = this->base_link->RelativeLinearAccel();

        // DIAGNOSTIC: Detect sudden changes in sensor data (possible physics instability)
        if (has_prev_sensor_data)
        {
          // Calculate changes in sensor readings
          ignition::math::Vector3d gyro_change = angular_vel - prev_angular_vel;
          ignition::math::Vector3d accel_change = linear_accel - prev_linear_accel;
          
          double gyro_change_mag = gyro_change.Length();
          double accel_change_mag = accel_change.Length();
          
          // Thresholds for "sudden" changes that might trigger FC overreaction
          // At 250Hz update rate (4ms), sudden change > 100 rad/s² in gyro is suspicious
          // or > 50 m/s² in accel
          const double GYRO_SPIKE_THRESHOLD = 10.0;    // rad/s change per cycle (4ms)
          const double ACCEL_SPIKE_THRESHOLD = 20.0;   // m/s² change per cycle (4ms)
          
          if (gyro_change_mag > GYRO_SPIKE_THRESHOLD || accel_change_mag > ACCEL_SPIKE_THRESHOLD)
          {
            std::cout << "[BetaflightPlugin] ⚠️  SENSOR SPIKE DETECTED!" << std::endl;
            std::cout << "  Gyro change: " << gyro_change_mag << " rad/s (threshold: " 
                      << GYRO_SPIKE_THRESHOLD << ")" << std::endl;
            std::cout << "    Prev: [" << prev_angular_vel.X() << ", " << prev_angular_vel.Y() 
                      << ", " << prev_angular_vel.Z() << "]" << std::endl;
            std::cout << "    Now:  [" << angular_vel.X() << ", " << angular_vel.Y() 
                      << ", " << angular_vel.Z() << "]" << std::endl;
            std::cout << "  Accel change: " << accel_change_mag << " m/s² (threshold: " 
                      << ACCEL_SPIKE_THRESHOLD << ")" << std::endl;
            std::cout << "    Prev: [" << prev_linear_accel.X() << ", " << prev_linear_accel.Y() 
                      << ", " << prev_linear_accel.Z() << "]" << std::endl;
            std::cout << "    Now:  [" << linear_accel.X() << ", " << linear_accel.Y() 
                      << ", " << linear_accel.Z() << "]" << std::endl;
          }
        }
        
        // Store current values for next cycle comparison
        prev_angular_vel = angular_vel;
        prev_linear_accel = linear_accel;
        has_prev_sensor_data = true;

        // Position (NED frame: North-East-Down)
        // Gazebo uses ENU (East-North-Up), convert to NED
        fdm.position_xyz[0] = pose.Pos().Y();   // North (Gazebo Y)
        fdm.position_xyz[1] = pose.Pos().X();   // East (Gazebo X)
        fdm.position_xyz[2] = -pose.Pos().Z();  // Down (Gazebo -Z)

        // Velocity (NED frame, earth frame)
        fdm.velocity_xyz[0] = linear_vel.Y();   // North
        fdm.velocity_xyz[1] = linear_vel.X();   // East
        fdm.velocity_xyz[2] = -linear_vel.Z();  // Down

        // Orientation quaternion (Gazebo ENU to Betaflight NED)
        // ENU: X=East, Y=North, Z=Up
        // NED: X=North, Y=East, Z=Down
        // Transformation: swap X↔Y, negate Z
        // In quaternion: rotate -90° around Z axis
        ignition::math::Quaterniond q_gazebo = pose.Rot();
        
        // Create rotation quaternion for -90° around Z
        // q = cos(θ/2) + sin(θ/2) * axis
        // -90° = -π/2, so θ/2 = -π/4
        double angle = -M_PI / 2.0;
        ignition::math::Quaterniond q_z_rotation(cos(angle/2), 0, 0, sin(angle/2));
        
        // Apply rotation: NED = Rz(-90°) * ENU * Rz(90°)
        // Simplified: just swap and negate in quaternion representation
        ignition::math::Quaterniond q_ned(
            q_gazebo.W(),   // w stays same
            q_gazebo.Y(),   // x_ned = y_enu (North from North)
            q_gazebo.X(),   // y_ned = x_enu (East from East) 
            -q_gazebo.Z()   // z_ned = -z_enu (Down from Up)
        );

        fdm.imu_orientation_quat[0] = q_ned.W();
        fdm.imu_orientation_quat[1] = q_ned.X();
        fdm.imu_orientation_quat[2] = q_ned.Y();
        fdm.imu_orientation_quat[3] = q_ned.Z();

        // Angular velocity (body frame, raw Gazebo data - let SITL handle conversions)
        fdm.imu_angular_velocity_rpy[0] = angular_vel.X();  // Roll rate
        fdm.imu_angular_velocity_rpy[1] = angular_vel.Y();  // Pitch rate
        fdm.imu_angular_velocity_rpy[2] = angular_vel.Z();  // Yaw rate

        // Linear acceleration (body frame, raw Gazebo data - let SITL handle conversions)
        // Add gravity component
        ignition::math::Vector3d gravity(0, 0, -9.80665);
        ignition::math::Vector3d accel_with_gravity = linear_accel - pose.Rot().RotateVectorReverse(gravity);

        fdm.imu_linear_acceleration_xyz[0] = accel_with_gravity.X();
        fdm.imu_linear_acceleration_xyz[1] = accel_with_gravity.Y();
        fdm.imu_linear_acceleration_xyz[2] = accel_with_gravity.Z();

        // Pressure (from altitude using barometric formula)
        // P = P0 * (1 - L*h/T0)^(g*M/R*L)
        // Simplified: P ≈ 101325 * exp(-h/8000)
        double altitude = -fdm.position_xyz[2]; // Convert NED down to altitude
        fdm.pressure = 101325.0 * exp(-altitude / 8000.0);

        // GPS data (simulated from position)
        // Use home location as reference point (can be configured)
        // Default: somewhere arbitrary, can be set via SDF parameters
        static const double home_latitude = -35.3632607;   // Example: CMAC field
        static const double home_longitude = 149.1652351;
        static const double home_altitude_msl = 584.0;     // meters
        
        // Convert NED position to lat/lon offset
        // Approximate: 1 degree latitude ≈ 111,320 m
        // 1 degree longitude ≈ 111,320 * cos(latitude) m
        double meters_per_deg_lat = 111320.0;
        double meters_per_deg_lon = 111320.0 * cos(home_latitude * M_PI / 180.0);
        
        fdm.latitude = home_latitude + (fdm.position_xyz[0] / meters_per_deg_lat);
        fdm.longitude = home_longitude + (fdm.position_xyz[1] / meters_per_deg_lon);
        fdm.altitude_msl = home_altitude_msl - fdm.position_xyz[2]; // NED Z is down
        
        // GPS velocity (same as earth-frame velocity, already in NED)
        fdm.gps_velocity_ned[0] = fdm.velocity_xyz[0];
        fdm.gps_velocity_ned[1] = fdm.velocity_xyz[1];
        fdm.gps_velocity_ned[2] = fdm.velocity_xyz[2];
        
        // Simulated GPS fix
        fdm.num_satellites = 12;  // Good GPS fix
        fdm.gps_fix_type = 3;     // 3D fix
      }

      void SendFDM()
      {
        ssize_t n = sendto(sock_fdm, &fdm, sizeof(fdm), 0,
                           (struct sockaddr*)&addr_fdm, sizeof(addr_fdm));

        static bool first_send = true;
        static int send_count = 0;

        if (n != sizeof(fdm))
        {
          if (first_send)
          {
            gzerr << "[BetaflightPlugin] Failed to send FDM packet: " << strerror(errno) << std::endl;
            first_send = false;
          }
        }
        else
        {
          if (first_send)
          {
            std::cout << "[BetaflightPlugin] First FDM packet sent successfully (size: " << n << " bytes)" << std::endl;
            first_send = false;
          }

          send_count++;
          if (send_count % 250 == 0)  // Every second at 250Hz
          {
            std::cout << "[BetaflightPlugin] FDM packets sent: " << send_count
                      << " | timestamp: " << fdm.timestamp << std::endl;
          }
        }
      }

      void ApplyMotorCommands()
      {
        // Don't apply commands if we haven't received valid PWM data yet
        if (!has_valid_servo)
        {
          return;
        }

        // Direct mapping: Betaflight motor index → Gazebo joint index
        // World file sudah di-reorder sesuai Betaflight QUAD_X order
        // Motor 0 = Rear-Right, Motor 1 = Front-Right, Motor 2 = Rear-Left, Motor 3 = Front-Left

        for (size_t i = 0; i < joints.size() && i < 4; i++)
        {
          // Use stable_servo instead of latest_servo for smoother control
          double motor_speed = stable_servo.motor_speed[i];
          
          // Clamp to valid range
          if (motor_speed < 0.0) motor_speed = 0.0;
          if (motor_speed > 1.0) motor_speed = 1.0;

          double target_velocity = motor_speed * motor_multipliers[i];
          joints[i]->SetVelocity(0, target_velocity);
        }

        // Debug output
        static int debug_count = 0;
        if (++debug_count % 250 == 0) {
          // Get current altitude and vertical velocity
          ignition::math::Pose3d pose = this->base_link->WorldPose();
          ignition::math::Vector3d linear_vel = this->base_link->WorldLinearVel();
          
          std::cout << "[BetaflightPlugin] ===== STATUS @ count " << debug_count << " =====" << std::endl;
          std::cout << "  RAW from SITL:  [" 
                    << latest_servo.motor_speed[0] << ", "
                    << latest_servo.motor_speed[1] << ", "
                    << latest_servo.motor_speed[2] << ", "
                    << latest_servo.motor_speed[3] << "]" << std::endl;
          std::cout << "  STABLE (used):  [" 
                    << stable_servo.motor_speed[0] << ", "
                    << stable_servo.motor_speed[1] << ", "
                    << stable_servo.motor_speed[2] << ", "
                    << stable_servo.motor_speed[3] << "]" << std::endl;
          std::cout << "  Motor Vel:      [" 
                    << joints[0]->GetVelocity(0) << ", "
                    << joints[1]->GetVelocity(0) << ", "
                    << joints[2]->GetVelocity(0) << ", "
                    << joints[3]->GetVelocity(0) << "]" << std::endl;
          std::cout << "  Alt: " << pose.Pos().Z() << " m | VelZ: " << linear_vel.Z() << " m/s" << std::endl;
        }
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(BetaflightPlugin)
}
