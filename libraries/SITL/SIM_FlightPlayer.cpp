/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for ardupilot version of Gazebo
*/

#include "SIM_FlightPlayer.h"

#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <errno.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

    FlightPlayer::FlightPlayer(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    dt(0.001),
    sock(false)
{
    // try to bind to a specific port so that if we restart ArduPilot
    // Gazebo keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!sock.connect("127.0.0.1", 9003)) {
        fprintf(stdout, "SITL: socket bind failed - %s\n", strerror(errno));
        exit(1);
    }

    set_rate_hz(1000);
    
    sock.reuseaddress();
    sock.set_blocking(false);
    fprintf(stdout, "bind\n");
}
    
/*
      decode and send servos
*/
void FlightPlayer::send_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    pkt.motor_speed[0] = (input.servos[0]-1000) / 1000.0f;
    pkt.motor_speed[1] = (input.servos[1]-1000) / 1000.0f;
    pkt.motor_speed[2] = (input.servos[2]-1000) / 1000.0f;
    pkt.motor_speed[3] = (input.servos[3]-1000) / 1000.0f;
    sock.sendto(&pkt, sizeof(pkt), "127.0.0.1", 9002);
}
    
/*
      receive an update from the FDM
      This is a blocking function
 */
void FlightPlayer::recv_fdm(const struct sitl_input &input)
{
    fdm_packet pkt;

    /*
      we re-send the servo packet every 0.1 seconds until we get a
      reply. This allows us to cope with some packet loss to the FDM
     */
    while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        send_servos(input);
    }

    // get imu stuff
    accel_body = Vector3f(pkt.imu_linear_acceleration_xyz[0],
                          pkt.imu_linear_acceleration_xyz[1],
                          pkt.imu_linear_acceleration_xyz[2]);

    gyro = Vector3f(pkt.imu_angular_velocity_rpy[0],
                    pkt.imu_angular_velocity_rpy[1],
                    pkt.imu_angular_velocity_rpy[2]);

    // compute dcm from imu orientation
    Quaternion quat(pkt.imu_orientation_quat[0],
                    pkt.imu_orientation_quat[1],
                    pkt.imu_orientation_quat[2],
                    pkt.imu_orientation_quat[3]);
    quat.rotation_matrix(dcm);

    double speedN =  pkt.velocity_xyz[0];
    double speedE =  pkt.velocity_xyz[1];
    double speedD =  pkt.velocity_xyz[2];
    velocity_ef = Vector3f(speedN, speedE, speedD);

    position = Vector3f(pkt.position_xyz[0],
                        pkt.position_xyz[1],
                        pkt.position_xyz[2]);

    // FlightPlayer runs a fixed-step solver
    time_now_us += dt * 1.0e6;
    adjust_frame_time(1.0 / dt);
}

/*
  run one step in FlightPlayer
 */
void FlightPlayer::update(const struct sitl_input &input)
{
    send_servos(input);
    recv_fdm(input);
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

} // namespace SITL
