// TrafficLightSM.hpp
#pragma once
#include "Arduino.h"
#include "StateMachines/StateDefines.hpp"
#include "StateMachines/StateMachine.hpp"
#include "Utility/gpsuUtility.hpp"

namespace TrafficLight
{

struct Traits
{
	using Context = TrafficLight::Context;
	using Config = TrafficLight::Config;
	using Data = TrafficLight::Data;
	using Inputs = TrafficLight::Inputs;
	using Event = TrafficLight::Event;
	using State = TrafficLight::State;
	using Command = TrafficLight::Command;
	using Mode = TrafficLight::Mode;
	using Guard = bool (*)(const Context&);
	using ExitAction = Command (*)(const Context&);
	using EntryAction = Command (*)(const Context&);

	static constexpr bool has_exit_actions = true;
	static constexpr bool has_entry_actions = true;
	static constexpr uint16_t state_count = 5;
	static constexpr uint16_t transition_count = 18;

	struct Transition
	{
		State from;
		State to;
		Guard condition;
		EntryAction entry_action;
		ExitAction exit_action;
	};
	static inline bool faultyInput(const Context& ctx)
	{

		return !auto_mode(ctx) && ctx.inputs.faultyinput;
	}
	static inline bool systemFault(const Context& ctx)
	{
		return ctx.event == Event::LED_FAULT || ctx.event == Event::TIMER_FAULT || faultyInput(ctx);
	}
	static inline bool systemFaultCleared(const Context& ctx)
	{
		return ctx.event == Event::OK && !faultyInput(ctx);
	}
	static inline bool auto_mode(const Context& ctx) { return ctx.mode == Mode::AUTO; }
	static inline bool modeChanged(const Context& ctx)
	{
		if(ctx.inputs.mode_changed)
		{
			LOG::DEBUG("TL_SM", "triggered mode changed to  %s",
					   ctx.mode == TrafficLight::Mode::AUTO ? "Auto" : "Manual");
		}
		else
		{
			LOG::DEBUG("no mode changed");
		}

		return ctx.inputs.mode_changed;
	}

	static inline bool manual_red(const Context& ctx)
	{
		return !auto_mode(ctx) && !faultyInput(ctx) && ctx.inputs.ui.turn_on_red;
	}
	static inline bool manual_yellow(const Context& ctx)
	{
		return !auto_mode(ctx) && !faultyInput(ctx) && ctx.inputs.ui.turn_on_yellow;
	}

	static inline bool manual_green(const Context& ctx)
	{
		return !auto_mode(ctx) && !faultyInput(ctx) && ctx.inputs.ui.turn_on_green;
	}

	static inline bool timeoutRed(const Context& ctx)
	{
		return auto_mode(ctx) && (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.red_to);
	}
	static inline bool timeoutGreen(const Context& ctx)
	{
		return auto_mode(ctx) &&
			   (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.green_to);
	}

	static inline bool manualYellowToRed(const Context& ctx)
	{
		return !auto_mode(ctx) && (ctx.inputs.ui.turn_on_red) && (ctx.prev == State::GREEN);
	}
	static inline bool manualYellowToGreen(const Context& ctx)
	{
		return !auto_mode(ctx) && (ctx.inputs.ui.turn_on_green) && (ctx.prev == State::RED);
	}
	static inline bool timeoutYellowToRed(const Context& ctx)
	{
		return auto_mode(ctx) &&
			   (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.yellow_to) &&
			   (ctx.prev == State::GREEN);
	}
	static inline bool timeoutYellowToGreen(const Context& ctx)
	{
		return auto_mode(ctx) &&
			   (ctx.inputs.timer.t1_expired || ctx.data.now >= ctx.config.yellow_to) &&
			   (ctx.prev == State::RED);
	}
	static inline bool buttonPress(const Context& ctx) { return ctx.inputs.ui.button_pressed; }
	struct entryActions
	{
		static Command initToRed(const Context& ctx)
		{

			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_RED;
			cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.red_to : 0;
			cmd.entry_data.immediate_transition = false;
			return cmd;
		}
		static Command redToYellow(const Context& ctx)
		{
			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_YELLOW;
			cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.yellow_to : 0;
			return cmd;
		}
		static Command redToGreen(const Context& ctx)
		{
			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_GREEN;
			cmd.entry_data.immediate_transition = true;
			return cmd;
		}
		static Command yellowToGreen(const Context& ctx)
		{
			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_GREEN;
			cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.green_to : 0;

			return cmd;
		}
		static Command greenToYellow(const Context& ctx)
		{
			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_YELLOW;
			cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.yellow_to : 0;

			return cmd;
		}
		static Command yellowToRed(const Context& ctx)
		{
			Command cmd;
			cmd.check_entry = true;
			cmd.entry_command = CmdType::TURN_ON_RED;
			cmd.entry_data.timeout1 = auto_mode(ctx) ? ctx.config.red_to : 0;
			return cmd;
		}
	};
	struct exitActions
	{

		static Command redToYellow(const Context& ctx)
		{

			Command cmd;
			cmd.check_exit = true;
			cmd.exit_command = CmdType::TURN_OFF_RED;
			return cmd;
		}
		static Command redToGreen(const Context& ctx)
		{

			Command cmd;
			cmd.check_exit = true;
			cmd.exit_command = CmdType::TURN_OFF_RED;
			cmd.exit_data.immediate_transition = false;
			return cmd;
		}
		static Command yellowToGreen(const Context& ctx)
		{
			Command cmd;
			cmd.check_exit = true;
			cmd.exit_command = CmdType::TURN_OFF_YELLOW;
			return cmd;
		}
		static Command greenToYellow(const Context& ctx)
		{
			Command cmd;
			cmd.check_exit = true;
			cmd.exit_command = CmdType::TURN_OFF_GREEN;
			return cmd;
		}
		static Command yellowToRed(const Context& ctx)
		{
			Command cmd;
			cmd.check_exit = true;
			cmd.exit_command = CmdType::TURN_OFF_YELLOW;
			return cmd;
		}
	};

	static constexpr std::array<Transition, transition_count> transitions = {{
		{State::INIT, State::RED, auto_mode, entryActions::initToRed, nullptr},
		{State::INIT, State::RED, manual_red, entryActions::initToRed, nullptr},
		{State::RED, State::YELLOW, timeoutRed, entryActions::redToYellow,
		 exitActions::redToYellow},
		{State::RED, State::YELLOW, manual_yellow, entryActions::redToYellow,
		 exitActions::redToYellow},

		{State::YELLOW, State::GREEN, timeoutYellowToGreen, entryActions::yellowToGreen,
		 exitActions::yellowToGreen},
		{State::YELLOW, State::GREEN, manualYellowToGreen, entryActions::yellowToGreen,
		 exitActions::yellowToGreen},

		{State::GREEN, State::YELLOW, timeoutGreen, entryActions::greenToYellow,
		 exitActions::greenToYellow},
		{State::GREEN, State::YELLOW, manual_yellow, entryActions::greenToYellow,
		 exitActions::greenToYellow},

		{State::YELLOW, State::RED, timeoutYellowToRed, entryActions::yellowToRed,
		 exitActions::yellowToRed},
		{State::YELLOW, State::RED, manualYellowToRed, entryActions::yellowToRed,
		 exitActions::yellowToRed},

		{State::RED, State::GREEN, buttonPress, entryActions::redToGreen, exitActions::redToGreen},

		{State::RED, State::FAULT, systemFault, nullptr, nullptr},
		{State::YELLOW, State::FAULT, systemFault, nullptr, nullptr},
		{State::GREEN, State::FAULT, systemFault, nullptr, nullptr},
		{State::FAULT, State::INIT, systemFaultCleared, nullptr, nullptr},
		{State::RED, State::INIT, modeChanged, nullptr, exitActions::redToYellow},
		{State::GREEN, State::INIT, modeChanged, nullptr, exitActions::greenToYellow},
		{State::YELLOW, State::INIT, modeChanged, nullptr, exitActions::yellowToRed},

	}

	};

	static constexpr auto transitions_by_state =
		SM::group_transitions_by_state<Transition, transition_count, state_count>(transitions);
};

} // namespace TrafficLight

namespace SM
{

class TrafficLightSM : public StateMachine<TrafficLight::Traits, TrafficLightSM>
{
  public:
	using Traits = TrafficLight::Traits;
	using Context = Traits::Context;
	using Config = Traits::Context::Config;
	using State = Traits::State;
	using Inputs = TrafficLight::Inputs;
	using Event = TrafficLight::Event;
	using Mode = TrafficLight::Mode;
	explicit TrafficLightSM(const Context& context, State state, bool internalTimer = true) :
		StateMachine(context, state, internalTimer), use_internal_timer(internalTimer)
	{
		register_callback(transitionCb);
	}
	TrafficLightSM(const Context& context, bool internalTimer = true) :
		TrafficLightSM(context, State::INIT, internalTimer)
	{
	}

	TrafficLightSM() : TrafficLightSM(Context{}, State::INIT, true) {}

	void updateInternalState(const Inputs& input)
	{
		check_ui_inputs(input);
		updateMode(input.ui.manual_mode);

		if(use_internal_timer && ctx_.mode == Mode::AUTO)
		{

			if(input.timer.t1_expired)
			{
				LOG::TEST("TL_SM", "Timer1 expired in %d ms for State %s", ctx_.data.now,
						  GPSU::Util::ToString::TLState(current()));
				ctx_.data.now = 0;
				update();
			}
		}
		else if(!use_internal_timer && ctx_.mode == Mode::AUTO)
		{
			ctx_.data.now += input.timer.ext_delta;
			update();
		}
		else if(ctx_.mode == Mode::MANUAL)
		{
			update();
		}
	}
	void updateInternalState(const Event ev) { ctx_.event = ev; }
	static void transitionCb(State from, State to, Context& ctx) { ctx.data.now = 0; }

	const char* getStateString() const { return GPSU::Util::ToString::TLState(current()); }
	void reset_ui() { ctx_.inputs.ui = Inputs::UI{}; }
	void printConfig() const
	{
		LOG::TEST("TL_SM", "Current Config.....");
		LOG::TEST("TL_SM", "Red Timeout =%d ms", ctx_.config.red_to);
		LOG::TEST("TL_SM", "Ylw Timeout =%d ms", ctx_.config.yellow_to);
		LOG::TEST("TL_SM", "Grn Timeout =%d ms", ctx_.config.green_to);
		LOG::TEST("TL_SM", "current time =%d ms", ctx_.data.now);
		LOG::TEST("TL_SM", "timer1 is %s", ctx_.inputs.timer.t1_expired ? "Expired" : "Running");
		LOG::TEST("TL_SM", "......");
	}

  private:
	bool use_internal_timer;

	void check_ui_inputs(const Inputs& input)
	{
		if(checkFaultyInput())
		{
			ctx_.inputs.faultyinput = true;
		}
		else
		{
			ctx_.inputs.faultyinput = false;
		}
		if(input.ui.turn_on_red)
		{
			LOG::DEBUG("TL_SM", "TURN RED BTN");
		}
		else if(input.ui.turn_on_yellow)
		{
			LOG::DEBUG("TL_SM", "TURN YELLOW BTN");
		}
		else if(input.ui.turn_on_green)
		{
			LOG::DEBUG("TL_SM", "TURN GREEN BTN");
		}
		else if(input.ui.manual_mode)
		{
			LOG::DEBUG("TL_SM", "MANUAL MODE");
		}
	}
	void updateMode(bool manual_requested)
	{
		const Mode new_mode = manual_requested ? Mode::MANUAL : Mode::AUTO;
		ctx_.prev_mode = ctx_.mode;
		if(new_mode != ctx_.mode)
		{

			ctx_.mode = new_mode;
			ctx_.inputs.mode_changed = true;
			LOG::DEBUG("SM", "Mode updated to %s", ctx_.mode == Mode::MANUAL ? "MANUAL" : "AUTO");
			if(ctx_.mode == Mode::AUTO)
			{

				if(timer1_was_deleted)
				{
					timer1_was_deleted = false;
					enableTimer(0);
					createTimers();
				}
				else if(timer2_was_deleted)
				{
					timer2_was_deleted = false;
					enableTimer(1);
					createTimers();
				}
			}
			else
			{
				disableTimers();
			}
			setAutoUpdate();
			update();
			ctx_.inputs.mode_changed = false;
		}
	}
	bool checkFaultyInput()
	{
		bool faulty_input = false;
		faulty_input = (ctx_.inputs.ui.turn_on_red + ctx_.inputs.ui.turn_on_yellow +
						ctx_.inputs.ui.turn_on_green) >= 2;
		return faulty_input;
	}
};

} // namespace SM
