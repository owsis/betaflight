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
  };

  // Betaflight servo packet (PWM from Betaflight)
  struct servo_packet {
    float motor_speed[4];  // normal: [0.0, 1.0], 3D: [-1.0, 1.0]
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
      servo_packet servo;

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
        memset(&servo, 0, sizeof(servo));
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
        ssize_t n = recv(sock_pwm, &servo, sizeof(servo), 0);

        static int recv_count = 0;
        static int error_count = 0;

        if (n == sizeof(servo))
        {
          recv_count++;
          static bool first_pwm = true;
          if (first_pwm)
          {
            std::cout << "[BetaflightPlugin] First PWM packet received (size " << n << " bytes): "
                      << servo.motor_speed[0] << ", "
                      << servo.motor_speed[1] << ", "
                      << servo.motor_speed[2] << ", "
                      << servo.motor_speed[3] << std::endl;
            first_pwm = false;
          }

          if (recv_count % 250 == 0) {
            std::cout << "[BetaflightPlugin] PWM packets received: " << recv_count
                      << " | motors: " << servo.motor_speed[0] << ", "
                      << servo.motor_speed[1] << ", "
                      << servo.motor_speed[2] << ", "
                      << servo.motor_speed[3] << std::endl;
          }
        }
        else if (n < 0)
        {
          if (errno != EAGAIN && errno != EWOULDBLOCK)
          {
            error_count++;
            if (error_count < 5) {
              std::cerr << "[BetaflightPlugin] recv() error: " << strerror(errno) << std::endl;
            }
          }
        }
        else if (n > 0)
        {
          std::cerr << "[BetaflightPlugin] Received partial packet: " << n
                    << " bytes (expected " << sizeof(servo) << ")" << std::endl;
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

        // Position (NED frame: North-East-Down)
        // Gazebo uses ENU (East-North-Up), convert to NED
        fdm.position_xyz[0] = pose.Pos().Y();   // North (Gazebo Y)
        fdm.position_xyz[1] = pose.Pos().X();   // East (Gazebo X)
        fdm.position_xyz[2] = -pose.Pos().Z();  // Down (Gazebo -Z)

        // Velocity (NED frame, earth frame)
        fdm.velocity_xyz[0] = linear_vel.Y();   // North
        fdm.velocity_xyz[1] = linear_vel.X();   // East
        fdm.velocity_xyz[2] = -linear_vel.Z();  // Down

        // Orientation quaternion (Gazebo to NED)
        ignition::math::Quaterniond q = pose.Rot();
        // Convert ENU to NED: rotate -90 deg around Z, then 180 deg around X
        ignition::math::Quaterniond q_enu_to_ned(0, 1, 0, 0); // 180 deg around X
        ignition::math::Quaterniond q_ned = q_enu_to_ned * q;

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
        // Direct mapping: Betaflight motor index → Gazebo joint index
        // World file sudah di-reorder sesuai Betaflight QUAD_X order
        // Motor 0 = Rear-Right, Motor 1 = Front-Right, Motor 2 = Rear-Left, Motor 3 = Front-Left
        
        for (size_t i = 0; i < joints.size() && i < 4; i++)
        {
          double motor_speed = servo.motor_speed[i];
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
          
          std::cout << "[BetaflightPlugin] Motors[0-3]: "
                    << servo.motor_speed[0] << "," << servo.motor_speed[1] << ","
                    << servo.motor_speed[2] << "," << servo.motor_speed[3]
                    << " | Vel: "
                    << joints[0]->GetVelocity(0) << ","
                    << joints[1]->GetVelocity(0) << ","
                    << joints[2]->GetVelocity(0) << ","
                    << joints[3]->GetVelocity(0)
                    << " | Alt: " << pose.Pos().Z()
                    << " | VelZ: " << linear_vel.Z() << std::endl;
        }
      }
  };

  GZ_REGISTER_MODEL_PLUGIN(BetaflightPlugin)
}
