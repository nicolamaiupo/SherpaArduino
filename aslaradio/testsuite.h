/** @file
 *	@brief MAVLink comm protocol testsuite generated from aslaradio.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef ASLARADIO_TESTSUITE_H
#define ASLARADIO_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_aslaradio(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_aslaradio(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_heartbeat(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_heartbeat_t packet_in = {
		963497464,17,84,151,218,3
    };
	mavlink_heartbeat_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.custom_mode = packet_in.custom_mode;
        	packet1.type = packet_in.type;
        	packet1.autopilot = packet_in.autopilot;
        	packet1.base_mode = packet_in.base_mode;
        	packet1.system_status = packet_in.system_status;
        	packet1.mavlink_version = packet_in.mavlink_version;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack(system_id, component_id, &msg , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_send(MAVLINK_COMM_1 , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	mavlink_msg_heartbeat_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sys_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sys_status_t packet_in = {
		963497464,963497672,963497880,17859,17963,18067,18171,18275,18379,18483,18587,18691,223
    };
	mavlink_sys_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.onboard_control_sensors_present = packet_in.onboard_control_sensors_present;
        	packet1.onboard_control_sensors_enabled = packet_in.onboard_control_sensors_enabled;
        	packet1.onboard_control_sensors_health = packet_in.onboard_control_sensors_health;
        	packet1.load = packet_in.load;
        	packet1.voltage_battery = packet_in.voltage_battery;
        	packet1.current_battery = packet_in.current_battery;
        	packet1.drop_rate_comm = packet_in.drop_rate_comm;
        	packet1.errors_comm = packet_in.errors_comm;
        	packet1.errors_count1 = packet_in.errors_count1;
        	packet1.errors_count2 = packet_in.errors_count2;
        	packet1.errors_count3 = packet_in.errors_count3;
        	packet1.errors_count4 = packet_in.errors_count4;
        	packet1.battery_remaining = packet_in.battery_remaining;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_status_pack(system_id, component_id, &msg , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
	mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
	mavlink_msg_sys_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sys_status_send(MAVLINK_COMM_1 , packet1.onboard_control_sensors_present , packet1.onboard_control_sensors_enabled , packet1.onboard_control_sensors_health , packet1.load , packet1.voltage_battery , packet1.current_battery , packet1.battery_remaining , packet1.drop_rate_comm , packet1.errors_comm , packet1.errors_count1 , packet1.errors_count2 , packet1.errors_count3 , packet1.errors_count4 );
	mavlink_msg_sys_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_system_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_system_time_t packet_in = {
		93372036854775807ULL,963497880
    };
	mavlink_system_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_unix_usec = packet_in.time_unix_usec;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_time_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_time_pack(system_id, component_id, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
	mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_unix_usec , packet1.time_boot_ms );
	mavlink_msg_system_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_system_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_system_time_send(MAVLINK_COMM_1 , packet1.time_unix_usec , packet1.time_boot_ms );
	mavlink_msg_system_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ping(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ping_t packet_in = {
		93372036854775807ULL,963497880,41,108
    };
	mavlink_ping_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.seq = packet_in.seq;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ping_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ping_pack(system_id, component_id, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
	mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ping_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
	mavlink_msg_ping_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ping_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ping_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.seq , packet1.target_system , packet1.target_component );
	mavlink_msg_ping_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_commands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_commands_t packet_in = {
		963497464,17443,17547,17651,17755,17859,17963,18067
    };
	mavlink_commands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.TStamp = packet_in.TStamp;
        	packet1.LSX = packet_in.LSX;
        	packet1.LSY = packet_in.LSY;
        	packet1.RSX = packet_in.RSX;
        	packet1.RSY = packet_in.RSY;
        	packet1.BSX = packet_in.BSX;
        	packet1.BSY = packet_in.BSY;
        	packet1.Buttons = packet_in.Buttons;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_commands_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_commands_pack(system_id, component_id, &msg , packet1.LSX , packet1.LSY , packet1.RSX , packet1.RSY , packet1.BSX , packet1.BSY , packet1.Buttons , packet1.TStamp );
	mavlink_msg_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_commands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.LSX , packet1.LSY , packet1.RSX , packet1.RSY , packet1.BSX , packet1.BSY , packet1.Buttons , packet1.TStamp );
	mavlink_msg_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_commands_send(MAVLINK_COMM_1 , packet1.LSX , packet1.LSY , packet1.RSX , packet1.RSY , packet1.BSX , packet1.BSY , packet1.Buttons , packet1.TStamp );
	mavlink_msg_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_raw_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_raw_int_t packet_in = {
		93372036854775807ULL,963497880,963498088,963498296,18275,18379,18483,18587,89,156
    };
	mavlink_gps_raw_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.eph = packet_in.eph;
        	packet1.epv = packet_in.epv;
        	packet1.vel = packet_in.vel;
        	packet1.cog = packet_in.cog;
        	packet1.fix_type = packet_in.fix_type;
        	packet1.satellites_visible = packet_in.satellites_visible;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_pack(system_id, component_id, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
	mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
	mavlink_msg_gps_raw_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_raw_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.fix_type , packet1.lat , packet1.lon , packet1.alt , packet1.eph , packet1.epv , packet1.vel , packet1.cog , packet1.satellites_visible );
	mavlink_msg_gps_raw_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_status_t packet_in = {
		5,{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91 },{ 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151 },{ 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211 },{ 252, 253, 254, 255, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },{ 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75 }
    };
	mavlink_gps_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.satellites_visible = packet_in.satellites_visible;
        
        	mav_array_memcpy(packet1.satellite_prn, packet_in.satellite_prn, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_used, packet_in.satellite_used, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_elevation, packet_in.satellite_elevation, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_azimuth, packet_in.satellite_azimuth, sizeof(uint8_t)*20);
        	mav_array_memcpy(packet1.satellite_snr, packet_in.satellite_snr, sizeof(uint8_t)*20);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_status_pack(system_id, component_id, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
	mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
	mavlink_msg_gps_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_status_send(MAVLINK_COMM_1 , packet1.satellites_visible , packet1.satellite_prn , packet1.satellite_used , packet1.satellite_elevation , packet1.satellite_azimuth , packet1.satellite_snr );
	mavlink_msg_gps_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_t packet_in = {
		963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
	mavlink_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.roll = packet_in.roll;
        	packet1.pitch = packet_in.pitch;
        	packet1.yaw = packet_in.yaw;
        	packet1.rollspeed = packet_in.rollspeed;
        	packet1.pitchspeed = packet_in.pitchspeed;
        	packet1.yawspeed = packet_in.yawspeed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.roll , packet1.pitch , packet1.yaw , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude_quaternion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_quaternion_t packet_in = {
		963497464,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
	mavlink_attitude_quaternion_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.q1 = packet_in.q1;
        	packet1.q2 = packet_in.q2;
        	packet1.q3 = packet_in.q3;
        	packet1.q4 = packet_in.q4;
        	packet1.rollspeed = packet_in.rollspeed;
        	packet1.pitchspeed = packet_in.pitchspeed;
        	packet1.yawspeed = packet_in.yawspeed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_quaternion_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_attitude_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.q1 , packet1.q2 , packet1.q3 , packet1.q4 , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed );
	mavlink_msg_attitude_quaternion_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_ned(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_ned_t packet_in = {
		963497464,45.0,73.0,101.0,129.0,157.0,185.0
    };
	mavlink_local_position_ned_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_local_position_ned_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_local_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz );
	mavlink_msg_local_position_ned_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_position_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_position_int_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587
    };
	mavlink_global_position_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.relative_alt = packet_in.relative_alt;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        	packet1.hdg = packet_in.hdg;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
	mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
	mavlink_msg_global_position_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.hdg );
	mavlink_msg_global_position_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_channels_scaled(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_scaled_t packet_in = {
		963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,132
    };
	mavlink_rc_channels_scaled_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.chan1_scaled = packet_in.chan1_scaled;
        	packet1.chan2_scaled = packet_in.chan2_scaled;
        	packet1.chan3_scaled = packet_in.chan3_scaled;
        	packet1.chan4_scaled = packet_in.chan4_scaled;
        	packet1.chan5_scaled = packet_in.chan5_scaled;
        	packet1.chan6_scaled = packet_in.chan6_scaled;
        	packet1.chan7_scaled = packet_in.chan7_scaled;
        	packet1.chan8_scaled = packet_in.chan8_scaled;
        	packet1.port = packet_in.port;
        	packet1.rssi = packet_in.rssi;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_scaled_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_scaled_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
	mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_scaled_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
	mavlink_msg_rc_channels_scaled_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_channels_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.port , packet1.chan1_scaled , packet1.chan2_scaled , packet1.chan3_scaled , packet1.chan4_scaled , packet1.chan5_scaled , packet1.chan6_scaled , packet1.chan7_scaled , packet1.chan8_scaled , packet1.rssi );
	mavlink_msg_rc_channels_scaled_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_channels_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_raw_t packet_in = {
		963497464,17443,17547,17651,17755,17859,17963,18067,18171,65,132
    };
	mavlink_rc_channels_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.chan1_raw = packet_in.chan1_raw;
        	packet1.chan2_raw = packet_in.chan2_raw;
        	packet1.chan3_raw = packet_in.chan3_raw;
        	packet1.chan4_raw = packet_in.chan4_raw;
        	packet1.chan5_raw = packet_in.chan5_raw;
        	packet1.chan6_raw = packet_in.chan6_raw;
        	packet1.chan7_raw = packet_in.chan7_raw;
        	packet1.chan8_raw = packet_in.chan8_raw;
        	packet1.port = packet_in.port;
        	packet1.rssi = packet_in.rssi;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_raw_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
	mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
	mavlink_msg_rc_channels_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_channels_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.port , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.rssi );
	mavlink_msg_rc_channels_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_set_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_set_gps_global_origin_t packet_in = {
		963497464,963497672,963497880,41
    };
	mavlink_set_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        	packet1.target_system = packet_in.target_system;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_pack(system_id, component_id, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_set_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_set_gps_global_origin_send(MAVLINK_COMM_1 , packet1.target_system , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_set_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_global_origin(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_global_origin_t packet_in = {
		963497464,963497672,963497880
    };
	mavlink_gps_global_origin_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.altitude = packet_in.altitude;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_pack(system_id, component_id, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_gps_global_origin_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_global_origin_send(MAVLINK_COMM_1 , packet1.latitude , packet1.longitude , packet1.altitude );
	mavlink_msg_gps_global_origin_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude_quaternion_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_attitude_quaternion_cov_t packet_in = {
		963497464,{ 45.0, 46.0, 47.0, 48.0 },157.0,185.0,213.0,{ 241.0, 242.0, 243.0, 244.0, 245.0, 246.0, 247.0, 248.0, 249.0 }
    };
	mavlink_attitude_quaternion_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.rollspeed = packet_in.rollspeed;
        	packet1.pitchspeed = packet_in.pitchspeed;
        	packet1.yawspeed = packet_in.yawspeed;
        
        	mav_array_memcpy(packet1.q, packet_in.q, sizeof(float)*4);
        	mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*9);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_cov_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_cov_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
	mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
	mavlink_msg_attitude_quaternion_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_attitude_quaternion_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_attitude_quaternion_cov_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.q , packet1.rollspeed , packet1.pitchspeed , packet1.yawspeed , packet1.covariance );
	mavlink_msg_attitude_quaternion_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_nav_controller_output(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_nav_controller_output_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,18275,18379,18483
    };
	mavlink_nav_controller_output_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.nav_roll = packet_in.nav_roll;
        	packet1.nav_pitch = packet_in.nav_pitch;
        	packet1.alt_error = packet_in.alt_error;
        	packet1.aspd_error = packet_in.aspd_error;
        	packet1.xtrack_error = packet_in.xtrack_error;
        	packet1.nav_bearing = packet_in.nav_bearing;
        	packet1.target_bearing = packet_in.target_bearing;
        	packet1.wp_dist = packet_in.wp_dist;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_controller_output_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_controller_output_pack(system_id, component_id, &msg , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
	mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_controller_output_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
	mavlink_msg_nav_controller_output_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_nav_controller_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_nav_controller_output_send(MAVLINK_COMM_1 , packet1.nav_roll , packet1.nav_pitch , packet1.nav_bearing , packet1.target_bearing , packet1.wp_dist , packet1.alt_error , packet1.aspd_error , packet1.xtrack_error );
	mavlink_msg_nav_controller_output_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_global_position_int_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_global_position_int_cov_t packet_in = {
		93372036854775807ULL,963497880,963498088,963498296,963498504,963498712,213.0,241.0,269.0,{ 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0, 305.0, 306.0, 307.0, 308.0, 309.0, 310.0, 311.0, 312.0, 313.0, 314.0, 315.0, 316.0, 317.0, 318.0, 319.0, 320.0, 321.0, 322.0, 323.0, 324.0, 325.0, 326.0, 327.0, 328.0, 329.0, 330.0, 331.0, 332.0 },45
    };
	mavlink_global_position_int_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_utc = packet_in.time_utc;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.alt = packet_in.alt;
        	packet1.relative_alt = packet_in.relative_alt;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        	packet1.estimator_type = packet_in.estimator_type;
        
        	mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*36);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_cov_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_cov_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_global_position_int_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_global_position_int_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_global_position_int_cov_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.lat , packet1.lon , packet1.alt , packet1.relative_alt , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_global_position_int_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_local_position_ned_cov(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_local_position_ned_cov_t packet_in = {
		93372036854775807ULL,963497880,101.0,129.0,157.0,185.0,213.0,241.0,{ 269.0, 270.0, 271.0, 272.0, 273.0, 274.0, 275.0, 276.0, 277.0, 278.0, 279.0, 280.0, 281.0, 282.0, 283.0, 284.0, 285.0, 286.0, 287.0, 288.0, 289.0, 290.0, 291.0, 292.0, 293.0, 294.0, 295.0, 296.0, 297.0, 298.0, 299.0, 300.0, 301.0, 302.0, 303.0, 304.0 },33
    };
	mavlink_local_position_ned_cov_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_utc = packet_in.time_utc;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        	packet1.vx = packet_in.vx;
        	packet1.vy = packet_in.vy;
        	packet1.vz = packet_in.vz;
        	packet1.estimator_type = packet_in.estimator_type;
        
        	mav_array_memcpy(packet1.covariance, packet_in.covariance, sizeof(float)*36);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_cov_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_cov_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_cov_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_local_position_ned_cov_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_local_position_ned_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_local_position_ned_cov_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.time_utc , packet1.estimator_type , packet1.x , packet1.y , packet1.z , packet1.vx , packet1.vy , packet1.vz , packet1.covariance );
	mavlink_msg_local_position_ned_cov_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_channels(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_t packet_in = {
		963497464,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,125,192
    };
	mavlink_rc_channels_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.chan1_raw = packet_in.chan1_raw;
        	packet1.chan2_raw = packet_in.chan2_raw;
        	packet1.chan3_raw = packet_in.chan3_raw;
        	packet1.chan4_raw = packet_in.chan4_raw;
        	packet1.chan5_raw = packet_in.chan5_raw;
        	packet1.chan6_raw = packet_in.chan6_raw;
        	packet1.chan7_raw = packet_in.chan7_raw;
        	packet1.chan8_raw = packet_in.chan8_raw;
        	packet1.chan9_raw = packet_in.chan9_raw;
        	packet1.chan10_raw = packet_in.chan10_raw;
        	packet1.chan11_raw = packet_in.chan11_raw;
        	packet1.chan12_raw = packet_in.chan12_raw;
        	packet1.chan13_raw = packet_in.chan13_raw;
        	packet1.chan14_raw = packet_in.chan14_raw;
        	packet1.chan15_raw = packet_in.chan15_raw;
        	packet1.chan16_raw = packet_in.chan16_raw;
        	packet1.chan17_raw = packet_in.chan17_raw;
        	packet1.chan18_raw = packet_in.chan18_raw;
        	packet1.chancount = packet_in.chancount;
        	packet1.rssi = packet_in.rssi;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
	mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
	mavlink_msg_rc_channels_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.chancount , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw , packet1.chan9_raw , packet1.chan10_raw , packet1.chan11_raw , packet1.chan12_raw , packet1.chan13_raw , packet1.chan14_raw , packet1.chan15_raw , packet1.chan16_raw , packet1.chan17_raw , packet1.chan18_raw , packet1.rssi );
	mavlink_msg_rc_channels_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rc_channels_override(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rc_channels_override_t packet_in = {
		17235,17339,17443,17547,17651,17755,17859,17963,53,120
    };
	mavlink_rc_channels_override_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.chan1_raw = packet_in.chan1_raw;
        	packet1.chan2_raw = packet_in.chan2_raw;
        	packet1.chan3_raw = packet_in.chan3_raw;
        	packet1.chan4_raw = packet_in.chan4_raw;
        	packet1.chan5_raw = packet_in.chan5_raw;
        	packet1.chan6_raw = packet_in.chan6_raw;
        	packet1.chan7_raw = packet_in.chan7_raw;
        	packet1.chan8_raw = packet_in.chan8_raw;
        	packet1.target_system = packet_in.target_system;
        	packet1.target_component = packet_in.target_component;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
	mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
	mavlink_msg_rc_channels_override_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rc_channels_override_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.chan1_raw , packet1.chan2_raw , packet1.chan3_raw , packet1.chan4_raw , packet1.chan5_raw , packet1.chan6_raw , packet1.chan7_raw , packet1.chan8_raw );
	mavlink_msg_rc_channels_override_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_vfr_hud(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_vfr_hud_t packet_in = {
		17.0,45.0,73.0,101.0,18067,18171
    };
	mavlink_vfr_hud_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.airspeed = packet_in.airspeed;
        	packet1.groundspeed = packet_in.groundspeed;
        	packet1.alt = packet_in.alt;
        	packet1.climb = packet_in.climb;
        	packet1.heading = packet_in.heading;
        	packet1.throttle = packet_in.throttle;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vfr_hud_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vfr_hud_pack(system_id, component_id, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
	mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vfr_hud_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
	mavlink_msg_vfr_hud_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_vfr_hud_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_vfr_hud_send(MAVLINK_COMM_1 , packet1.airspeed , packet1.groundspeed , packet1.heading , packet1.throttle , packet1.alt , packet1.climb );
	mavlink_msg_vfr_hud_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radio_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radio_status_t packet_in = {
		17235,17339,17,84,151,218,29
    };
	mavlink_radio_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rxerrors = packet_in.rxerrors;
        	packet1.fixed = packet_in.fixed;
        	packet1.rssi = packet_in.rssi;
        	packet1.remrssi = packet_in.remrssi;
        	packet1.txbuf = packet_in.txbuf;
        	packet1.noise = packet_in.noise;
        	packet1.remnoise = packet_in.remnoise;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_status_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radio_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_status_send(MAVLINK_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_power_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_power_status_t packet_in = {
		17235,17339,17443
    };
	mavlink_power_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.Vcc = packet_in.Vcc;
        	packet1.Vservo = packet_in.Vservo;
        	packet1.flags = packet_in.flags;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_power_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_power_status_pack(system_id, component_id, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
	mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_power_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.Vcc , packet1.Vservo , packet1.flags );
	mavlink_msg_power_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_power_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_power_status_send(MAVLINK_COMM_1 , packet1.Vcc , packet1.Vservo , packet1.flags );
	mavlink_msg_power_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_battery_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_battery_status_t packet_in = {
		963497464,963497672,17651,{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764 },18795,101,168,235,46
    };
	mavlink_battery_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.current_consumed = packet_in.current_consumed;
        	packet1.energy_consumed = packet_in.energy_consumed;
        	packet1.temperature = packet_in.temperature;
        	packet1.current_battery = packet_in.current_battery;
        	packet1.id = packet_in.id;
        	packet1.function = packet_in.function;
        	packet1.type = packet_in.type;
        	packet1.battery_remaining = packet_in.battery_remaining;
        
        	mav_array_memcpy(packet1.voltages, packet_in.voltages, sizeof(uint16_t)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_battery_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_battery_status_pack(system_id, component_id, &msg , packet1.id , packet1.function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining );
	mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_battery_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining );
	mavlink_msg_battery_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_battery_status_send(MAVLINK_COMM_1 , packet1.id , packet1.function , packet1.type , packet1.temperature , packet1.voltages , packet1.current_battery , packet1.current_consumed , packet1.energy_consumed , packet1.battery_remaining );
	mavlink_msg_battery_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_statustext(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_statustext_t packet_in = {
		5,"BCDEFGHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWX"
    };
	mavlink_statustext_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.severity = packet_in.severity;
        
        	mav_array_memcpy(packet1.text, packet_in.text, sizeof(char)*50);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_pack(system_id, component_id, &msg , packet1.severity , packet1.text );
	mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.severity , packet1.text );
	mavlink_msg_statustext_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_statustext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_statustext_send(MAVLINK_COMM_1 , packet1.severity , packet1.text );
	mavlink_msg_statustext_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radio(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radio_t packet_in = {
		17235,17339,17,84,151,218,29
    };
	mavlink_radio_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rxerrors = packet_in.rxerrors;
        	packet1.fixed = packet_in.fixed;
        	packet1.rssi = packet_in.rssi;
        	packet1.remrssi = packet_in.remrssi;
        	packet1.txbuf = packet_in.txbuf;
        	packet1.noise = packet_in.noise;
        	packet1.remnoise = packet_in.remnoise;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_send(MAVLINK_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps_rtk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps_rtk_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,963498504,963498712,18691,223,34,101,168,235
    };
	mavlink_gps_rtk_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_last_baseline_ms = packet_in.time_last_baseline_ms;
        	packet1.tow = packet_in.tow;
        	packet1.baseline_a_mm = packet_in.baseline_a_mm;
        	packet1.baseline_b_mm = packet_in.baseline_b_mm;
        	packet1.baseline_c_mm = packet_in.baseline_c_mm;
        	packet1.accuracy = packet_in.accuracy;
        	packet1.iar_num_hypotheses = packet_in.iar_num_hypotheses;
        	packet1.wn = packet_in.wn;
        	packet1.rtk_receiver_id = packet_in.rtk_receiver_id;
        	packet1.rtk_health = packet_in.rtk_health;
        	packet1.rtk_rate = packet_in.rtk_rate;
        	packet1.nsats = packet_in.nsats;
        	packet1.baseline_coords_type = packet_in.baseline_coords_type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_rtk_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_rtk_pack(system_id, component_id, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_rtk_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps_rtk_send(MAVLINK_COMM_1 , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps2_rtk(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps2_rtk_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,963498504,963498712,18691,223,34,101,168,235
    };
	mavlink_gps2_rtk_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_last_baseline_ms = packet_in.time_last_baseline_ms;
        	packet1.tow = packet_in.tow;
        	packet1.baseline_a_mm = packet_in.baseline_a_mm;
        	packet1.baseline_b_mm = packet_in.baseline_b_mm;
        	packet1.baseline_c_mm = packet_in.baseline_c_mm;
        	packet1.accuracy = packet_in.accuracy;
        	packet1.iar_num_hypotheses = packet_in.iar_num_hypotheses;
        	packet1.wn = packet_in.wn;
        	packet1.rtk_receiver_id = packet_in.rtk_receiver_id;
        	packet1.rtk_health = packet_in.rtk_health;
        	packet1.rtk_rate = packet_in.rtk_rate;
        	packet1.nsats = packet_in.nsats;
        	packet1.baseline_coords_type = packet_in.baseline_coords_type;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps2_rtk_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps2_rtk_pack(system_id, component_id, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps2_rtk_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps2_rtk_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps2_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps2_rtk_send(MAVLINK_COMM_1 , packet1.time_last_baseline_ms , packet1.rtk_receiver_id , packet1.wn , packet1.tow , packet1.rtk_health , packet1.rtk_rate , packet1.nsats , packet1.baseline_coords_type , packet1.baseline_a_mm , packet1.baseline_b_mm , packet1.baseline_c_mm , packet1.accuracy , packet1.iar_num_hypotheses );
	mavlink_msg_gps2_rtk_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_distance_sensor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_distance_sensor_t packet_in = {
		963497464,17443,17547,17651,163,230,41,108
    };
	mavlink_distance_sensor_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.min_distance = packet_in.min_distance;
        	packet1.max_distance = packet_in.max_distance;
        	packet1.current_distance = packet_in.current_distance;
        	packet1.type = packet_in.type;
        	packet1.id = packet_in.id;
        	packet1.orientation = packet_in.orientation;
        	packet1.covariance = packet_in.covariance;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_distance_sensor_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_distance_sensor_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
	mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_distance_sensor_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
	mavlink_msg_distance_sensor_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_distance_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_distance_sensor_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.min_distance , packet1.max_distance , packet1.current_distance , packet1.type , packet1.id , packet1.orientation , packet1.covariance );
	mavlink_msg_distance_sensor_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_terrain_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_terrain_request_t packet_in = {
		93372036854775807ULL,963497880,963498088,18067
    };
	mavlink_terrain_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.mask = packet_in.mask;
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.grid_spacing = packet_in.grid_spacing;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_request_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
	mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
	mavlink_msg_terrain_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_terrain_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_request_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.mask );
	mavlink_msg_terrain_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_terrain_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_terrain_data_t packet_in = {
		963497464,963497672,17651,{ 17755, 17756, 17757, 17758, 17759, 17760, 17761, 17762, 17763, 17764, 17765, 17766, 17767, 17768, 17769, 17770 },3
    };
	mavlink_terrain_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.grid_spacing = packet_in.grid_spacing;
        	packet1.gridbit = packet_in.gridbit;
        
        	mav_array_memcpy(packet1.data, packet_in.data, sizeof(int16_t)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_data_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
	mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
	mavlink_msg_terrain_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_terrain_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_data_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.grid_spacing , packet1.gridbit , packet1.data );
	mavlink_msg_terrain_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_terrain_check(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_terrain_check_t packet_in = {
		963497464,963497672
    };
	mavlink_terrain_check_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_check_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_check_pack(system_id, component_id, &msg , packet1.lat , packet1.lon );
	mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_check_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon );
	mavlink_msg_terrain_check_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_terrain_check_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_check_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon );
	mavlink_msg_terrain_check_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_terrain_report(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_terrain_report_t packet_in = {
		963497464,963497672,73.0,101.0,18067,18171,18275
    };
	mavlink_terrain_report_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.terrain_height = packet_in.terrain_height;
        	packet1.current_height = packet_in.current_height;
        	packet1.spacing = packet_in.spacing;
        	packet1.pending = packet_in.pending;
        	packet1.loaded = packet_in.loaded;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_report_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_report_pack(system_id, component_id, &msg , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
	mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_report_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
	mavlink_msg_terrain_report_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_terrain_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_terrain_report_send(MAVLINK_COMM_1 , packet1.lat , packet1.lon , packet1.spacing , packet1.terrain_height , packet1.current_height , packet1.pending , packet1.loaded );
	mavlink_msg_terrain_report_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_landing_target(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_landing_target_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,157.0,185.0,89,156
    };
	mavlink_landing_target_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.angle_x = packet_in.angle_x;
        	packet1.angle_y = packet_in.angle_y;
        	packet1.distance = packet_in.distance;
        	packet1.size_x = packet_in.size_x;
        	packet1.size_y = packet_in.size_y;
        	packet1.target_num = packet_in.target_num;
        	packet1.frame = packet_in.frame;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_landing_target_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_landing_target_pack(system_id, component_id, &msg , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y );
	mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_landing_target_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y );
	mavlink_msg_landing_target_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_landing_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_landing_target_send(MAVLINK_COMM_1 , packet1.time_usec , packet1.target_num , packet1.frame , packet1.angle_x , packet1.angle_y , packet1.distance , packet1.size_x , packet1.size_y );
	mavlink_msg_landing_target_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_debug_vect(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_debug_vect_t packet_in = {
		93372036854775807ULL,73.0,101.0,129.0,"UVWXYZABC"
    };
	mavlink_debug_vect_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_usec = packet_in.time_usec;
        	packet1.x = packet_in.x;
        	packet1.y = packet_in.y;
        	packet1.z = packet_in.z;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_vect_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_vect_pack(system_id, component_id, &msg , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
	mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_vect_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
	mavlink_msg_debug_vect_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_debug_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_debug_vect_send(MAVLINK_COMM_1 , packet1.name , packet1.time_usec , packet1.x , packet1.y , packet1.z );
	mavlink_msg_debug_vect_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_named_value_float(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_value_float_t packet_in = {
		963497464,45.0,"IJKLMNOPQ"
    };
	mavlink_named_value_float_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.value = packet_in.value;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_float_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_float_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_float_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_float_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_named_value_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_float_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_float_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_named_value_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_named_value_int_t packet_in = {
		963497464,963497672,"IJKLMNOPQ"
    };
	mavlink_named_value_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.value = packet_in.value;
        
        	mav_array_memcpy(packet1.name, packet_in.name, sizeof(char)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_int_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_named_value_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_named_value_int_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.name , packet1.value );
	mavlink_msg_named_value_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_aslaradio(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_heartbeat(system_id, component_id, last_msg);
	mavlink_test_sys_status(system_id, component_id, last_msg);
	mavlink_test_system_time(system_id, component_id, last_msg);
	mavlink_test_ping(system_id, component_id, last_msg);
	mavlink_test_commands(system_id, component_id, last_msg);
	mavlink_test_gps_raw_int(system_id, component_id, last_msg);
	mavlink_test_gps_status(system_id, component_id, last_msg);
	mavlink_test_attitude(system_id, component_id, last_msg);
	mavlink_test_attitude_quaternion(system_id, component_id, last_msg);
	mavlink_test_local_position_ned(system_id, component_id, last_msg);
	mavlink_test_global_position_int(system_id, component_id, last_msg);
	mavlink_test_rc_channels_scaled(system_id, component_id, last_msg);
	mavlink_test_rc_channels_raw(system_id, component_id, last_msg);
	mavlink_test_set_gps_global_origin(system_id, component_id, last_msg);
	mavlink_test_gps_global_origin(system_id, component_id, last_msg);
	mavlink_test_attitude_quaternion_cov(system_id, component_id, last_msg);
	mavlink_test_nav_controller_output(system_id, component_id, last_msg);
	mavlink_test_global_position_int_cov(system_id, component_id, last_msg);
	mavlink_test_local_position_ned_cov(system_id, component_id, last_msg);
	mavlink_test_rc_channels(system_id, component_id, last_msg);
	mavlink_test_rc_channels_override(system_id, component_id, last_msg);
	mavlink_test_vfr_hud(system_id, component_id, last_msg);
	mavlink_test_radio_status(system_id, component_id, last_msg);
	mavlink_test_power_status(system_id, component_id, last_msg);
	mavlink_test_battery_status(system_id, component_id, last_msg);
	mavlink_test_statustext(system_id, component_id, last_msg);
	mavlink_test_radio(system_id, component_id, last_msg);
	mavlink_test_gps_rtk(system_id, component_id, last_msg);
	mavlink_test_gps2_rtk(system_id, component_id, last_msg);
	mavlink_test_distance_sensor(system_id, component_id, last_msg);
	mavlink_test_terrain_request(system_id, component_id, last_msg);
	mavlink_test_terrain_data(system_id, component_id, last_msg);
	mavlink_test_terrain_check(system_id, component_id, last_msg);
	mavlink_test_terrain_report(system_id, component_id, last_msg);
	mavlink_test_landing_target(system_id, component_id, last_msg);
	mavlink_test_debug_vect(system_id, component_id, last_msg);
	mavlink_test_named_value_float(system_id, component_id, last_msg);
	mavlink_test_named_value_int(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ASLARADIO_TESTSUITE_H
