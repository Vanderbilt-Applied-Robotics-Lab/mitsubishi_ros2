// UDP command packet specification from 'CRnQ_CRnD_Detailed-Functions-Operations.pdf'
// page 454-457. A modern and simplified version of the original 'strdef.h'
//
// Author: Joshua Holden Turner

#ifndef COMMAND_PACKET_H
#define COMMAND_PACKET_H

// Optional
#define VERSION_H7

// Includes
#include <stdint.h>
#include <array>

// Constant
static const constexpr uint8_t MAX_JOINTS = 8;

struct Packet
{
	/* -------------------------------------------------------------------------- */
	/*                                    Types                                   */
	/* -------------------------------------------------------------------------- */

	enum class Command : uint16_t
	{
		// Real time external command invalid
		INVALID = 0x0000,
		// Real time external command valid
		VALID = 0x0001,
		// Real time external command end
		END = 0x00FF
	};

	enum class Type : uint16_t
	{
		// No data
		NONE = 0x0000,
		// XYZ data
		XYZ = 0x0001,
		// Joint data
		JOINT = 0x0002,
		// Motor pulse data
		MOTOR_PULSE = 0x0003,
		// Filtered XYZ data
		FILTERED_XYZ = 0x0004,
		// Filtered joint data
		FILTERED_JOINT = 0x0005,
		// Filtered motor pulse data
		FILTERED_MOTOR_PULSE = 0x0006,
		// Encoder XYZ data
		ENCODER_XYZ = 0x0007,
		// Encoder joint data
		ENCODER_JOINT = 0x0008,
		// Encoder motor pulse data
		ENCODER_MOTOR_PULSE = 0x0009,
		// Electric current command percentage
		PERCENT_CURRENT_COMMAND = 0x000A,
		// Electric current feedback percentage
		PERCENT_CURRENT_FEEDBACK = 0x000B,
	};

	struct World
	{
		// Cartesian coordinates [mm]
		float x, y, z;
		// Polar coordinates[rad]
		float a, b, c;
		// Additional axis 1 & 2 [mm or rad]
		float l1, l2;
	};

	struct Pose
	{
		// Pose in world frame [mm or rad]
		World world;
		// Structural flags
		uint32_t struct_flag_1, struct_flag_2;
	};

	struct Joint
	{
		// Joint values [rad]
		float j[MAX_JOINTS];
	};

	struct Pulse
	{
		// Motor pulse values [pls]
		int32_t p[MAX_JOINTS];
	};

	enum class IO : uint16_t
	{
		// No data
		None = 0x00,
		// Output signal
		Output = 0x01,
		// Input signal
		Input = 0x02
	};

	union Data
	{
		// Pose in XYZ and Polar coordinates [mm or rad]
		Pose pose;
		// Joint values [rad]
		Joint joint;
		// Motor pulse values [pls]
		Pulse pulse;
		// Integer type [1]
		int32_t lng[MAX_JOINTS];
	};

	struct Reply
	{
		// Reply data type specification
		Type type;
		// Reserved
		const uint16_t reserved = 0x0000;
		// Reply data
		Data data;
	};

	/* -------------------------------------------------------------------------- */
	/*                                    Data                                    */
	/* -------------------------------------------------------------------------- */

	// Command specification
	Command command;
	Type send_type;
	// Reply data #0
	Reply reply_0;
	// IO signal designation
	IO send_io_type;
	IO recv_io_type;
	// Bit masks
	uint16_t bit_top;
	uint16_t bit_mask;
	uint16_t io_data;
	// Counters
	uint16_t timeout_counter;
	uint32_t transmission_counter;
#ifdef VERSION_H7
	// Reply data #1
	Reply reply_1;
	// Reply data #2
	Reply reply_2;
	// Reply data #3
	Reply reply_3;
#endif // VERSION_H7
};

#endif // COMMAND_PACKET_H