#ifndef STATE_DEFINES_HPP
#define STATE_DEFINES_HPP
#include "Arduino.h"
#include "Logger.hpp"
#include "stdint.h"

namespace TrafficLight
{
enum class Mode : uint8_t
{
	AUTO,
	MANUAL
};
enum class State : uint8_t
{
	INIT = 0,
	RED,
	GREEN,
	YELLOW,
	FAULT
};
enum class Event : uint8_t
{
	OK = 0,
	TIMER_FAULT,
	LED_FAULT,
};
enum class CmdType : uint8_t
{
	NONE = 0,
	RESET,
	TURN_ON_RED,
	TURN_ON_YELLOW,
	TURN_ON_GREEN,
	TURN_OFF_RED,
	TURN_OFF_YELLOW,
	TURN_OFF_GREEN,
};
struct CommandData
{
	int timeout1 = 0;
	int timeout2 = 0;
	bool immediate_transition = false;
};
struct Command
{
	bool check_exit = false;
	bool check_entry = false;
	CmdType exit_command;
	CmdType entry_command;
	CommandData exit_data;
	CommandData entry_data;
};
struct Config
{
	int red_to = 8000;
	int yellow_to = 2000;
	int green_to = 6000;
	bool transit_now = false;
	uint16_t error_blink = 500;
	uint8_t max_errors = 3;
};
struct Inputs
{
	bool new_data = false;
	bool mode_changed = false;
	bool faultyinput = false;
	struct Timer
	{
		int ext_delta = 0;
		uint32_t now = 0;
		bool t1_expired = false;
		bool t2_expired = false;
	} timer;
	struct UI
	{
		bool turn_on_red = false;
		bool turn_on_yellow = false;
		bool turn_on_green = false;
		bool button_pressed = false;
		bool manual_mode = false;
	} ui;
};
struct Data
{
	uint32_t now = 0;
	bool timer_expired = false;
};

struct Context
{
	using State = TrafficLight::State;
	using Config = TrafficLight::Config;
	using Data = TrafficLight::Data;
	using Inputs = TrafficLight::Inputs;
	using Event = TrafficLight::Event;
	using Mode = TrafficLight::Mode;
	using Command = TrafficLight::Command;
	State curr, prev;
	Config config;
	Data data;
	Inputs inputs;
	Event event;
	Mode mode = Mode::AUTO;
	Mode prev_mode = Mode::AUTO;
	Command new_cmd;

	// Assignment operator
	Context& operator=(const Context& other)
	{
		if(this != &other)
		{
			curr = other.curr;
			prev = other.prev;
			config = other.config;
			data = other.data;
			inputs = other.inputs;
			event = other.event;
			mode = other.mode;
			prev_mode = other.prev_mode;
			new_cmd = other.new_cmd;
		}
		return *this;
	}
};

} // namespace TrafficLight
namespace WaterLevel
{
enum class Mode : uint8_t
{
	AUTO,
	MANUAL
};
enum class State : uint8_t
{
	EMPTY = 0,
	START_FILLING,
	FILLING,
	DRAINING,
	PARTIAL_FILLED,
	FULL,
	OVERFLOW,
	FAULT
};
enum class CmdType : uint8_t
{
	NONE = 0,
	LEVEL_UPDATE,
	START_FILL,
	FILL_TO_LEVEL,
	STOP,
	START_DRAIN,
	TRIGGER_ALARM,
	CLEAR_ALARM,

};
struct CommandData
{
	int timeout1 = 0;
	int timeout2 = 0;
	int pump_speed = 0;
	int32_t target_level = 0;
	bool open_drain_valve = false;
	bool open_fill_valve = false;
	bool alarm = false;
};
struct Command
{
	bool check_exit = false;
	bool check_entry = false;
	CmdType exit_command;
	CmdType entry_command;
	CommandData exit_data;
	CommandData entry_data;
};
struct Config
{
	uint16_t alarm_level = 2100;
	uint16_t max_capacity = 2000;
	uint16_t partial_mark = 500;
	uint16_t draining_mark = 1800;
	int fill_rate = 100;
	int drain_rate = -50;
	int32_t sensor_min_sensitivity = 10;
	int32_t hysteresis = 15;
};

struct Inputs
{
	bool new_data = false;
	struct Timer
	{
		int ext_delta = 0;
		bool t1_expired = false;
		bool t2_expired = false;
		int now = 0;
	} timer;
	struct Sensors
	{
		int32_t raw_adc_value = 0;
		int32_t measured_level = 0;
		bool drain_valve_state = false;
		bool fill_valve_state = false;
	} sensors;
	struct UI
	{
		int32_t new_target_level = 0;
		bool fill_request = false;
		bool drain_request = false;
		bool stop = false;
	} ui;
};
enum class Event : uint8_t
{
	OK = 0,
	SENSOR_FAULT,
	PUMP_FAULT,
	DRAIN_VALVE_FAULT,
	FILL_VALVE_FAULT,
};
struct Data
{
	unsigned long filling_start_time = 0;
	int32_t current_level = 0;
	int32_t current_target_level = 0;
};

struct Context
{
	using State = WaterLevel::State;
	using Config = WaterLevel::Config;
	using Data = WaterLevel::Data;
	using Inputs = WaterLevel::Inputs;
	using Event = WaterLevel::Event;
	using Mode = WaterLevel::Mode;
	using Command = WaterLevel::Command;
	State curr, prev;
	Config config;
	Data data;
	Inputs inputs;
	Event event;
	Mode mode = Mode::AUTO;
	Command new_cmd;

	// Assignment operator
	Context& operator=(const Context& other)
	{
		if(this != &other)
		{
			curr = other.curr;
			prev = other.prev;
			config = other.config;
			data = other.data;
			inputs = other.inputs;
			event = other.event;
			mode = other.mode;
			new_cmd = other.new_cmd;
		}
		return *this;
	}
};
} // namespace WaterLevel
namespace StepperMotor
{
enum class Mode : uint8_t
{
	AUTO,
	MANUAL
};
enum class State : uint8_t
{
	IDLE = 0,
	MOVING_CW,
	MOVING_CCW,
	HOMING,
	ERROR
};

enum class CmdType : uint8_t
{
	NONE = 0,
	START_MOVE,
	STOP,
	SET_TARGET,
	HOME,
	CLEAR_ERROR,
	EMERGENCY_STOP
};

struct CommandData
{
	int timeout1 = 0;
	int timeout2 = 0;
	int32_t target_position = 0;
	uint16_t speed_rpm = 60;
	bool direction = true; // CW=true, CCW=false
};
struct Command
{
	bool check_exit = false;
	bool check_entry = false;
	CmdType exit_command;
	CmdType entry_command;
	CommandData exit_data;
	CommandData entry_data;
};
struct Config
{
	// Physical properties
	float degrees_per_step = 1.8;
	uint16_t steps_per_revolution = 200;
	uint16_t max_steps = 2000; // Mechanical limit

	// Electrical properties
	uint16_t max_rpm = 300;
	uint16_t hold_current = 30; // Percentage
	uint16_t move_current = 100; // Percentage

	// Safety
	uint32_t stall_timeout = 2000; // ms
	int32_t position_tolerance = 5; // Steps
};

struct Inputs
{
	bool new_data = false;
	struct Timer
	{
		int ext_delta = 0;
		bool t1_expired = false;
		bool t2_expired = false;
		int now = 0;
	} timer;
	struct Sensors
	{
		// PCNT inputs
		int16_t pulse_count = 0;
		bool direction_state = true;

		// Limit switches
		bool home_switch = false;
		bool cw_limit = false;
		bool ccw_limit = false;

		// Driver feedback
		bool driver_fault = false;
		bool over_temp = false;
	} sensors;

	struct UI
	{
		int32_t target_position = 0;
		bool start_move = false;
		bool stop = false;
		bool home_request = false;
		bool clear_error = false;
	} ui;
};

enum class Event : uint8_t
{
	OK = 0,
	MOVEMENT_COMPLETE,
	HOMING_COMPLETE,
	STALL_DETECTED,
	OVERCURRENT,
	OVERTEMP,
	LIMIT_SW_ACTIVATED
};

struct Data
{
	// Position tracking
	int32_t current_position = 0;
	int32_t target_position = 0;
	// Movement parameters
	uint16_t current_rpm = 0;
	bool current_direction = true;
	// Timing
	unsigned long move_start_time = 0;
	// Error tracking
	uint8_t error_code = 0;
};

// Pulse generation control structure
struct PulseControl
{
	uint8_t step_pin;
	uint8_t dir_pin;
	uint16_t pulse_delay; // µs between pulses
	bool pulse_state;
};
struct Context
{
	using State = StepperMotor::State;
	using Config = StepperMotor::Config;
	using Data = StepperMotor::Data;
	using Inputs = StepperMotor::Inputs;
	using Event = StepperMotor::Event;
	using Pulse = StepperMotor::PulseControl;
	using Mode = StepperMotor::Mode;
	using Command = StepperMotor::Command;

	State curr, prev;
	Config config;
	Data data;
	Inputs inputs;
	Event event;
	Pulse pulse;
	Mode mode = Mode::AUTO;
	Command new_cmd;

	// Assignment operator
	Context& operator=(const Context& other)
	{
		if(this != &other)
		{
			curr = other.curr;
			prev = other.prev;
			config = other.config;
			data = other.data;
			inputs = other.inputs;
			event = other.event;
			pulse = other.pulse;
			mode = other.mode;
			new_cmd = other.new_cmd;
		}
		return *this;
	}
};
} // namespace StepperMotor
namespace Items
{

enum class State : uint8_t
{
	INIT = 0,
	PLACED,
	SENSED,
	ARRIVAL,
	PICKED,
	FAILED
};

struct Data
{
	uint32_t place_time; // Placement timestamp
	uint32_t sense_time; // Expected sensor trigger
	uint32_t pick_time; // Expected picker arrival
	uint32_t pick_attempt; // Pick initiation time
	uint32_t deadline; // Latest allowed pick time
};

} // namespace Items
namespace ObjectCounter
{
enum class Mode : uint8_t
{
	AUTO,
	MANUAL
};
enum class State : uint8_t
{
	INIT = 0,
	READY,
	RUNNING,
	FAULT,
	E_STOP,
	RESET
};

struct Item
{
	int id = 0;
	Items::State state;
	Items::Data data;
	bool on_conv = false;
	bool sensed = false;
	bool at_pick = false;
	uint16_t x_pos = 0;
	uint16_t y_pos = 0;
};
enum class Event : uint8_t
{
	OK = 0,
	NONE,
	SENSED,
	ARRIVAL,
	PICK_OK,
	ESTOP,
	SAFETY_TO,
	RUNWAY
};
enum class CmdType : uint8_t
{
	NONE = 0,
	RESET,
	START,
	STOP,
	NEW_OBJ,
	SENSE,
	PICK,
	ALARM,
	CLEAR_ALARM
};
struct CommandData
{
	int timeout1 = 0;
	int timeout2 = 0;
	uint32_t duration = 0;
	uint8_t retries = 0;
	bool needs_ack = false;
	bool alarm = false;
};
struct Command
{
	bool check_exit = false;
	bool check_entry = false;
	CmdType exit_command;
	CmdType entry_command;
	CommandData exit_data;
	CommandData entry_data;
};
struct Config
{
	static constexpr uint16_t conv_length = 1000;
	static constexpr uint16_t obj_length = 50;
	static constexpr uint16_t conv_mmps = 50;
	static constexpr uint16_t sen_pos = 200;
	static constexpr uint16_t pick_pos = 800;
	static constexpr int32_t placement_rate = 10;
	static constexpr uint32_t sim_pick_delay = 20;
	static constexpr uint8_t max_objs = 10;
	static constexpr uint32_t sensor_latch = 2000;
	static constexpr uint32_t pick_latch = 1000;

	// Derived timings (calculated once during initialization)
	uint32_t sense_delay; //
	uint32_t sense_timeout;
	uint32_t pick_timeout;
	uint32_t pick_delay; //
	uint32_t place_interval; //
	uint32_t auto_timeout; //
	uint32_t manual_timeout; //

	Config()
	{
		sense_delay = (sen_pos * 1000) / conv_mmps;
		sense_timeout = sense_delay + sensor_latch;

		pick_delay = (pick_pos * 1000) / conv_mmps;
		pick_timeout = pick_delay + pick_latch;
		place_interval = placement_rate * 1000;
		auto_timeout = pick_delay + sim_pick_delay;
		manual_timeout = 2000;
	}
};
struct Inputs
{
	bool new_data = false;
	bool mode_changed = false;
	struct Timer
	{
		bool t1_expired = false; // placement timer, resets after placement
		bool t2_expired = false; // timer item
		uint32_t place_time = 0;
		uint32_t obj_time = 0;
	} timer;
	struct Sensors
	{
		bool photoeye = false; // Main item detection sensor
		bool pick_pos = false; // Picker mechanism position
		bool estop = false; // E-stop button state
		bool guard = true; // Machine safety guard
	} sensors;
	struct UI
	{
		bool start = false;
		bool stop = false;
		bool pick = false; // Manual pick trigger
		bool manual_mode = false; // Auto/Manual toggle
		bool reset = false;
	} ui;
};
struct Data
{
	bool picking = false;
	struct Timing
	{
		uint32_t now = 0;
		uint32_t last_placed = 0;
		uint32_t uptime = 0;
		uint32_t runtime = 0;
	} timing;
	struct Stats
	{
		uint16_t total = 0;
		uint16_t ok = 0;
		uint16_t failed = 0;
		uint32_t start_time = 0;
		float avg_cycle = 0.0f;
	} stat;
};

struct Context
{
	using State = ObjectCounter::State;
	using Config = ObjectCounter::Config;
	using Data = ObjectCounter::Data;
	using Inputs = ObjectCounter::Inputs;
	using Event = ObjectCounter::Event;
	using Mode = ObjectCounter::Mode;
	using Command = ObjectCounter::Command;

	State curr, prev;
	Config config;
	Data data;
	Inputs inputs;
	Event event;
	Mode mode = Mode::AUTO;
	Mode prev_mode = Mode::AUTO;
	Command new_cmd;
	Item items[Config::max_objs];
	uint8_t obj_cnt = 0; // Number of active items
	uint8_t id_cnt = 0;
	// Assignment operator
	Context& operator=(const Context& other)
	{
		if(this != &other)
		{
			curr = other.curr;
			prev = other.prev;
			config = other.config;
			data = other.data;
			inputs = other.inputs;
			event = other.event;
			mode = other.mode;
			prev_mode = other.prev_mode;
			new_cmd = other.new_cmd;
			for(uint8_t i = 0; i < Config::max_objs; i++)
				items[i] = other.items[i];
			obj_cnt = other.obj_cnt;
		}
		return *this;
	}
};

} // namespace ObjectCounter
#endif