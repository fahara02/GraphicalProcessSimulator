#pragma once
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>
#include "Logger.hpp"
#include <algorithm>
#include <unordered_set>
#include <atomic>
#include <functional>
#include "freertos/FreeRTOS.h"

namespace SM
{
/*
 DESIGN GOAL RULES
//<---STATE--->
1. THERE IS ALWAYS A TOP STATE/ROOT STATE
1.1 A State is Either IDLE Or ACTIVE
1.2 State is of Three Kind
	1.2a  simple State(isSimple = true)- A simple State has no internal Vertices or Transitions.
	1.2b  composite State(isComposite = true)- A composite State contains at least one Region.A
composite State can be either a simple composite State with exactly one Region or an orthogonal
State with multiple Regions (isOrthogonal = true)
	1.2c submachine State- refers to an entire StateMachine, which is, conceptually, deemed to be
	 “nested” within the State
1.3  Any State enclosed within a Region of a composite State is called a substate of that
composite State.
1.4  Sharing of substate i.e overlapping not allowed !
1.5 A State is said to be active if it is part of the active state configuration.
1.6 A state configuration is said to be stable when:
	1.6a no further Transitions from that state configuration are enabled and
	1.6b all the entry Behaviors of that configuration, if present, have completed (but not
necessarily the doActivity Behaviors of that configuration, which, if defined, may continue
executing)
1.7 State entry, exit, and doActivity Behaviors
  1.7a A State may have an associated entry Behavior. This Behavior, if defined, is executed
whenever the State is entered through an external Transition
  1.7b  A State may also have an associated exit Behavior, which, if defined, is executed whenever
the State is exited.
  1.7c A State may also have an associated doActivity Behavior. This Behavior commences execution
when the State is entered (but only after the State entry Behavior has completed) and executes
concurrently with any other Behaviors that may be associated with the State, until:
 1.7c1 it completes (in which case a completion event is generated)
 1.7c2 the State is exited, in which case execution of the doActivity Behavior is aborted.
 1.7c3 The execution of a doActivity Behavior of a State is not affected by the firing of an
internal Transition of that State.
1.8  State history is a Pseudostates and is of two types  deepHistory and shallowHistory
 1.81 Deep history (deepHistory) represents the full state configuration(including innermost state)
of the most recent visit to the containing Region
1.82 Shallow history (shallowHistory) represents a return to only the topmost substate of the most
recent state configuration, which is entered using the default entry rule.

1.9 Entering A State
 1.9a For A Simple State --- the entry Behavior of the State is executed (if defined) upon entry,
but only after any effect Behavior associated . with the incoming Transition is completed. Also, if
a doActivity Behavior is defined for the State, this Behavior commences execution immediately after
the entry Behavior is executed
1.9b Composite State --
	Composites State has  5 types of Entry
	1.9b1  Default entry:  when the composite State is the direct target of a Transition.After
executing the entry Behavior and forking a possible doActivity Behavior execution, There is an
initial Pseudostate  defined, State entry continues from that Vertex via its outgoing Transition to
that initial Pseudostate then it will enter to its inner state based on psedustate transition logic.
if no pseudostate then  treat the composite State as a simple State, terminating the traversal on
that State despite its internal parts.
   1.9b2 Explicit entry: If the incoming Transition or its continuations terminate on a directly
contained substate of the composite State, then that substate becomes active and its entry Behavior
is executed after the execution of the entry Behavior of the containing composite State. This rule
applies recursively if the Transition terminates on an indirect (deeply nested) substate.
   1.9b3 Shallow history entry: If the incoming Transition terminates on a shallowHistory
Pseudostate of a Region of the composite State, the active substate becomes the substate that was
most recently active prior to this entry, unless:
1.9b31  the most recently active substate is the FinalState, or
1.9b32   this is the first entry into this State.
1.9b33  In the latter two cases, if a default shallow history Transition is defined originating from
the shallowHistory Pseudostate, it will be taken. Otherwise, default State entry is applied

1.9b4 Deep history entry: The rule for this case is the same as for shallow history except that the
target Pseudostate is of type deepHistory and the rule is applied recursively to all levels in the
active state configuration below this one
1.9b5 Entry point entry: If a Transition enters a composite State through an entryPoint
Pseudostate, then the effect Behavior associated with the outgoing Transition originating from the
entry point and penetrating into the State (but after the entry Behavior of the composite State has
been executed).
1.9b6  If the composite State is also an orthogonal State with multiple Regions, each of its Regions
is also entered, either by default or explicitly.
1.9b7  If the Transition terminates on the edge of the composite State (i.e., without entering the
State), then all the Regions are entered using the default entry rule above.
1.9b8 If the Transition explicitly enters one or more Regions (in case of a fork), these Regions are
entered explicitly and the others by default.
1.9c Regardless of how a State is entered, the StateMachine is deemed to be “in” that State even
before any entry Behavior or effect Behavior (if defined) of that State start executing.
1.10  Exiting a State
1.10a When exiting a State, regardless of whether it is simple or composite, the final step involved
in the exit, after all other Behaviors associated with the exit are completed, is the execution of
the exit Behavior of that State
1.10b If the State has a doActivity Behavior that is still executing when the State is exited, that
Behavior is aborted before the exit Behavior commences execution.
1.10c When exiting from a composite State, exit commences with the innermost State in the active
state configuration. This means that exit Behaviors are executed in sequence starting with the
innermost active State.
1.10d  If the exit occurs through an exitPoint Pseudostate, then the exit Behavior of the State is
executed after the effect Behavior of the Transition terminating on the exit point.
1.10e  When exiting from an orthogonal State, each of its Regions is exited. After that, the exit
Behavior of the State is executed
1.10f Regardless of how a State is exited, the StateMachine is deemed to have “left” that State only
after the exit Behavior (if defined) of that State has completed execution.


  //<----GROUPS i.e Wrapper of ORTHOGONAL STATES/Region------->
2.HAREL'S AND STATE /ORTHOGONAL STATE IS ACTUALLY PARALLEL CONCURRENT GROUP ,WE WILL CALL IT GROUP
WHERE HE SAYS'S  " being in a state, the system must be in all of its AND components".
2.1 GROUP/GROUPS AND ITS MEMBERS ALWAYS HAVE COMMON PARENT as THERE IS ALWAYS A TOP STATE
2.2 Group will be activated when any of its inner state enters or if it is directly owned by a
StateMachine
2.3 Groups will be deactivated when parent state exit 2.4 IF THERE IS GROUPS DIRECTLY
BELOW TOP_STATE/ANY COMPOSITE STATE  ACTIVE STATE WILL BE EACH ACTIVE STATE OF THOSE GROUPS AND
GROUPS ACTIVATION WILL  BE CONCURRENT .IF TRUE PARALLEL BEHAVIOUR CANT BE POSSIBLE(i.e FOR SINGLE
CORE) ACTIVATION WILL BE ATLEAST BE BASED ON GROUP PRIORITY AND IMMEDIATE SEQUENCE OF HIGHEST
PRIORITY TO LOWER
<----------TRANSITIONS--------->
3 TRANSITIONS IS OF THREE TYPES ---INTERNAL , EXTERNAL ,LOCAL
3.1 INTERNAL -Implies that the Transition, if triggered, occurs without exiting or entering the
source State. Also means Source==Traget
3.2 EXTERNAL- Implies that the Transition, if triggered, will exit the source State.
3.3 LOCAL- Implies that the Transition, if triggered, will not exit the composite (source) State,
but it will exit and re-enter any state within the composite State that is in the current state
configuration.
3.4 Simple state with no orthogonal region inside will only have internal and external transition
3.5 State can only Transit to a State not other Entity. All Transition Must have a Source State and
Target State
3.6 STATE TO GROUP OR GROUP TO STATE TRANSITION NEVER HAPPENS; TRANSITION IS ALWAYS BETWEEN STATES
3.7 STATE CAN ONLY TRANSIT TO ANOTHER STATE VIA RULE OF PARENT TO CHILD(NEVER GRAND
CHILD)/CHILD_TO_PARENT(NEVER GRAND PARENT)/SIBLINGS WITH SAMEGROUP/AND AS PER 3.6 //3.7   ON ONE
TRANSITION.
3.8 TRANSITION BETWEEN GROUPS/REGION WITH COMMON PARENT NOT ALLOWED
3.9 TRANSITION BETWEEN GROUPS (INNER STATES) WITH DIFFERENT PARENT ALLOWED, IT MEANS ONE GROUP STATE
TO ANOTHER GROUP STATE
3.10 TRANSITION BETWEEN A STATE TO A GROUP'S INNER STATE WILL HAPPEN IN TWO STEPS:
   a. STATE TO GROUP PARENT, then
   b. IMMEDIATELY GROUP PARENT TO GROUP INNER STATE DICTATED BY TRANSITION TABLE  OR IF NOT DEFAULT
3.11 A Transition is enabled if and only if:
3.11a  All of its source States are in the active state configuration.
3.11b  At least one of the triggers of the Transition has an Event that is matched by the Event type
of the dispatched Event occurrence. In case of Signal Events, any occurrence of the same or
compatible type as specified in the Trigger will match. If one of the Triggers is for an
AnyReceiveEvent, then either a Signal or CallEvent satisfies this Trigger, provided that there is no
other Signal or CallEvent Trigger for the same Transition or any other Transition having the same
source Vertex as the Transition with the AnyReceiveEvent trigger
3.11c If there exists at least one full path from the source state configuration to either the
target state configuration or to a dynamic choice Pseudostate in which all guard conditions are true
3.12 Transitions without guards are treated as if their guards are always true.


4. TRANSITION CONFLICT RESOLUTION
	 When multiple transitions are enabled for the same event, a deterministic priority scheme
must be applied.That is Higher INDEX MEANS HIGHER PRIORITY FOR ANY
GROUP(_priority)/TRANSIT(_priority)/STATE(_level*_id)
 For that Hierarchical (Depth-First)
Evaluation: 4.1: When an event occurs, first evaluate transitions in the most deeply nested active
state. If a transition is enabled in a nested state, it takes precedence over those in parent
states. 4.2: If no transitions are enabled in the current state, the event “bubbles up” to its
parent, and the evaluation continues until an enabled transition is found or the root is reached.

Transition Type Priority within Composite States:
4.3 (Internal Transitions):
Definition: An internal transition occurs without exiting or re-entering the source state (source
and target are the same).
Priority: These transitions have the highest priority because they preserve the current state
configuration.
4.4 (Local Transitions):
Definition: A local transition does not exit the composite (source) state but will exit and re-enter
any substate that is part of the current configuration.
Priority: These transitions come second in priority. They allow internal reconfiguration while still
preserving the composite boundary.
4.5 (External Transitions):
Definition: An external transition causes the composite (source) state to exit entirely.
Priority: These transitions have the lowest priority among the three, as they represent a full exit
from the composite state.
Ordering within the Same Transition Type:
4.6: When multiple transitions of the same type are enabled, their order of declaration (or a
predefined order in the state machine specification) is used to determine which one fires.
Concurrent (Orthogonal) Regions/GROUP:
4.7: For orthogonal regions, each region evaluates its transitions independently.
4.8: If transitions from different regions conflict (or interact with shared resources), a
predefined static or configurable region priority (or deterministic execution order) should be used
to resolve the conflict.Use Group priority for this
4.9 a Transition originating from a substate has higher priority than a conflicting Transition
originating from any of its containing States

4.10 The priority of a Transition is defined based on its source State
4.11 The priority of Transitions chained in a compound transition is based on the priority of the
Transition with the most deeply nested source State
4.12 The priority of Transitions chained in a compound transition is based on the priority of the
Transition with the most deeply nested source State

In general, if t1 is a Transition whose source State is s1, and t2 has source s2, then:
•     If s1 is a direct or indirectly nested substate of s2, then t1 has higher priority than t2.
•     If s1 and s2 are not in the same state configuration, then there is no priority difference
between t1 and t2.in this case if t1 and t2 both guards is true then they will be fired based on the
s1 and s2 level .if both are same then they will fired based on order of transition table
definition.
5


5.<-------------Event--------------->
//All event evaluated staring from root (No exception)
//
//
//
//A innerState of a Composite State can transit to  Outside Parent's Sibling and vice versa
//GUARD / ACTION WITH LOWER ID  WILL BE EVALUATED FIRST
//<--------------Event Propagation------------------>
//Event Source can be an External Class,StateMachine,Transition,External State,Parent State,Sibling
  State, own Group or External Group .
// Internal Events are those  whose source is Parent State,Sibling
  State, own Group
//External events can only be notified to state machine
//state machine will notify the event to currentState via handleEvent api
//Transitions


  // NO LINT

7. FOR THE SAME EVENT, IF TRANSITION GUARD IS TRUE IN MULTIPLE GROUPS WITH SAME LEVEL, ALL WILL
TRANSIT  // NO LINT

8. FOR THE SAME EVENT, IF TRANSITION GUARD IS TRUE IN MULTIPLE NESTED STATES, TRANSITIONS WILL OCCUR
IN ORDER OF LOWER LEVEL FIRST  // NO LINT

9. LOWEST LEVEL STATE WILL PROPAGATE EVENT UPWARDS  // NO LINT

10. HIGHER LEVEL STATE WILL PROPAGATE EVENTS BOTH WAYS, UPWARDS AND DOWNWARDS  // NO LINT

11. HIGHEST LEVEL STATE WILL PROPAGATE EVENT DOWNWARDS  // NO LINT

12. ENTRY AND EXIT ACTIONS
	12.1 Every state (simple or composite) must have clearly defined entry and exit actions.
	12.2 For composite states, the entry action must execute before nested default states are
activated; conversely, exit actions must be executed after all nested states have exited.

13.

14. INTERNAL VS. EXTERNAL TRANSITIONS
	14.1 Transitions are classified as internal or external.
	14.2 An internal transition does not trigger exit and re-entry actions for the source state,
while an external transition does. 14.3 The type of transition must be explicitly stated and handled
according to its classification.

15. EVENT CONSUMPTION AND PROPAGATION CONTROL
	15.1 Each state must indicate whether an event is “consumed” after processing or allowed to
propagate. 15.2 Once an event is consumed at a given level, it should not trigger further
transitions in parent states. 15.3 The propagation rules must be clearly defined to ensure
consistency across nested levels.

16. CONCURRENCY (ORTHOGONAL REGIONS)
	16.1 If concurrent state groups (orthogonal regions) are allowed, each region must process
events independently unless coordinated explicitly.
	16.2 Transitions in one region should not
interfere with those in another. 16.3 Synchronization mechanisms must be defined for inter-region
communication if needed.

//FOR FUTUTRE NOT NOW
17. ERROR HANDLING AND UNDEFINED TRANSITIONS
	17.1 A default error handling mechanism must be defined for events that do not match any valid
transition. 17.2 Fallback or error states may be specified to maintain system robustness. 17.3
Logging and notification procedures should be in place for undefined or unexpected transitions.

18. TIMING AND DELAYS
	18.1 Time-based transitions (such as timeouts or delays) must be integrated into the state
machine with clear rules. 18.2 Timing mechanisms should follow the same priority and conflict
resolution rules as event-driven transitions. 18.3 Rules for starting, stopping, and resetting
timers must be explicitly documented.
*/

constexpr int MAX_GROUP_SM = 4;
constexpr int MAX_STATE_SM = 20;
constexpr int MAX_INNER_STATE = 10;
constexpr int MAX_STATE_PER_GROUP = 5;
constexpr int MAX_GROUP_PER_STATE = 4;
constexpr int MAX_GROUP_PER_LEVEL = 4; // Total Group number not exceeding MAX_GROUP_SM
constexpr int MAX_TRANSITIONS = MAX_GROUP_PER_LEVEL * MAX_STATE_PER_GROUP;
constexpr int MAX_NESTING = 3;
static constexpr size_t MAX_EVENT_TYPE = 4;
static constexpr size_t MAX_LISTENER_PER_TYPE = 5;

using HandlerID_t = int32_t;
using TimerID_t = int32_t;
using EventID_t = int32_t;
using StateID_t = int32_t;
using SignalID_t = int32_t;
using Time_t = unsigned long;
using Trigger_t = std::function<void()>;
using EventHandler_t = std::function<bool(void)>;
using TimerHandler_t = std::function<bool(const TimerID_t)>;
using EnqueuedEventHandler_t = std::function<bool(const EventID_t)>;
using ActionHandler_t = std::function<void()>;

constexpr SignalID_t USER_SIGNAL_START = 0;
enum class SIGNAL : SignalID_t
{
	SIG_ENTRY = -1,
	SIG_EXIT = -2,
	SIG_INIT = -3,
	SIG_USER = USER_SIGNAL_START
};
enum class EventGroupType
{
	Signal,
	MSG,
	Call,
	Time,
	Change
};
enum class EventStatus : uint8_t
{
	OCURRED = 0,
	DISPATCHED = 1,
	DISCARDED = 2,
	ACKNOWLEDGED = 3,
	TRIGGERED = 4,
	COMPLETION = 5,
	ANY = 99
};
enum class EventOrigin : uint8_t
{
	INTERNAL_EV = 0,
	EXTERNAL_EV = 1,
	BROADCAST_EV = 2,
	STAE_MACHINE_EV = 3,
	SYSTE_EV = 4,
	NONE = 99
};
class IEvent
{
  public:
	IEvent(const IEvent&) = delete; // No copy
	IEvent& operator=(const IEvent&) = delete; // No copy-assign
	IEvent(IEvent&&) = default; // Allow move
	IEvent& operator=(IEvent&&) = default; // Allow move-assign

	virtual ~IEvent() = default;
	virtual EventGroupType getType() const = 0;
	virtual bool trigger(const Trigger_t& t) = 0;
	virtual bool create(EventOrigin org)
	{
		bool result = false;
		if(createImpl(EventOrigin org))
		{
			if(_birthTime != 0)
			{
				_birthTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
			}
			result = setOrigin(org);
			result = setStatus(EventStatus::OCCURRED);
		}
		return result;
	}
	virtual bool createImpl(EventOrigin org) = 0;
	EventID_t getId() const { return _id; }
	EventStatus status() const { return _status; }
	bool isStatus(EventStatus st) const { return _status == st ? true : false; }
	EventOrigin getOrigin() const { return _origin; }
	int32_t when() const { return _birthTime; }
	bool setStatus(EventStatus st)
	{
		if(st != _status)
		{
			_status = st;
			return true;
		}
		else
		{
			return false;
		}
	}
	constexpr bool operator==(const IEvent& other) const
	{
		return _id == other._type && _name == other._name && _data == other._data;
	}

  protected:
	bool setOrigin(EventOrigin ogn)
	{
		if(org == EventOrigin::NONE)
		{
			_origin = ogn;
			return true;
		}
		return false;
	}

  private:
	EventID_t _id;
	Time_t _birthTime = 0;
	EventStatus _status = EventStatus::ANY;
	EventOrigin _origin = EventOrigin::NONE;

	// Generate unique IDs (per event type)
	static EventID_t generateEventId()
	{
		static std::atomic<EventID_t> idCounter{0};
		return idCounter++;
	}

}; // namespace SM

template<typename T, typename Derived>
class EventBase : public IEvent
{
  public:
	// Constructor initializes common event properties
	EventBase(EventOrigin org)
	{

		if(create(org))
		{
			_id = generateEventId();
		}
	}

	// Implement IEvent's pure virtual functions via CRTP
	EventGroupType getType() const override
	{
		return Derived::EventType; // Requires Derived to define EventType
	}

	bool trigger(const Trigger_t& t) override
	{
		return static_cast<Derived*>(this)->triggerImpl(t);
	}

	bool createImpl() override { return static_cast<Derived*>(this)->createImplDerived(); }

	//=== Common Status Management ===
	bool markDispatched() { return setStatus(EventStatus::DISPATCHED); }
	bool discard() { return setStatus(EventStatus::DISCARDED); }
	bool acknowledge() { return setStatus(EventStatus::ACKNOWLEDGED); }
	bool triggered() { return setStatus(EventStatus::TRIGGERED); }
	bool markCompleted() { return setStatus(EventStatus::COMPLETION); }

  protected:
	T Data;
};

class IEventDispatcher
{
  public:
	virtual ~IEventDispatcher() = default;
	virtual EventGroupType getType() const = 0;
	virtual bool start() = 0;
	virtual void stop() = 0;
	virtual HandlerID_t registerEventHandler(const EventHandler_t& handler) = 0;
	virtual void unregisterEventHandler(const HandlerID_t handlerID) = 0;
	virtual void emitEvent(const HandlerID_t handlerID) = 0;
};

class Utility
{
  public:
	class ID
	{
	  public:
		// The generate function now accepts an optional "childs" parameter.
		// Here we shift the incremented counter by 4 bits and combine it with childs,
		// which should be in the range [0, 15]. This makes the generated id contain both
		// a unique part (from the counter) and the child order.
		inline static int generate(int childs = 0)
		{
			return ((++id_counter) << 4) | (childs & 0xF);
		}

	  private:
		static std::atomic<int> id_counter;
	};
};

// Static member initialization
std::atomic<int> Utility::ID::id_counter{0};

enum class EventType
{
	INTERNAL_EV,
	EXTERNAL_EV,
	BROADCAST_EV,
	NULL_EV
};
enum class TransitionType : uint8_t
{
	TRANS_INTERNAL = 0,
	TRANS_EXTERNAL = 1,
	UNDEFINED = 2
};

enum class StateActionTrigger
{
	ON_STATE_ENTRY,
	ON_STATE_EXIT,
	DURING,
};

struct Event
{
	constexpr Event() : _type(EventType::NULL_EV), _name("Null") {}
	constexpr Event(EventType type, const char* name) : _type(type), _name(name) {}
	constexpr Event(EventType type, const char* name,
					std::variant<int, float, const char*> eventData) :
		_type(type), _name(name), _data(eventData)
	{
	}
	constexpr EventType getType() const { return _type; }
	const char* getName() const { return _name; }
	template<typename T>
	constexpr T getData() const
	{
		return std::get<T>(_data);
	}
	constexpr Event& operator=(const Event& other)
	{
		if(this != &other)
		{
			_type = other._type;
			_name = other._name;
			_data = other._data;
		}
		return *this;
	}
	constexpr bool operator==(const Event& other) const
	{
		return _type == other._type && _name == other._name && _data == other._data;
	}
	constexpr bool hasData() const { return _data.index() != std::variant_npos; }
	const char* toString() const
	{
		switch(_type)
		{
			case EventType::INTERNAL_EV:
				return "Internal Event";
			case EventType::EXTERNAL_EV:
				return "External Event";
			case EventType::BROADCAST_EV:
				return "Broadcast Event";
			case EventType::NULL_EV:
				return "Null Event";
			default:
				return "Unknown Event";
		}
	}

  private:
	EventType _type;
	const char* _name;
	std::variant<int, float, const char*> _data;
};
constexpr Event NullEvent;
struct ValidatedContext
{
	SM::Event event;
};

//=========================================================
// Type trait to validate Context has an 'event' member of type Event
//=========================================================
namespace detail
{
template<typename T>
struct has_event_member
{
  private:
	template<typename U>
	static auto test(int) -> decltype(std::declval<U>().event, std::true_type{});

	template<typename U>
	static std::false_type test(...);

  public:
	static constexpr bool value = decltype(test<T>(0))::value;
};

template<typename T>
struct event_member_is_correct_type
{
  private:
	template<typename U>
	static auto test(int) -> std::is_same<decltype(std::declval<U>().event), Event>;

	template<typename U>
	static std::false_type test(...);

  public:
	static constexpr bool value = decltype(test<T>(0))::value;
};
} // namespace detail

class StateFunction
{
  public:
	constexpr StateFunction() : _id(Utility::ID::generate()) {}
	constexpr int getId() const { return _id; }

  private:
	int _id;
};

template<typename Context>
struct Guard : public StateFunction
{
  public:
	using FuncPtr = bool (*)(const std::optional<Context>&);
	constexpr Guard(FuncPtr func) : StateFunction(), _func(func) {}
	constexpr ~Guard() override = default;
	static_assert(detail::has_event_member<Context>::value, "Context must have an 'event' member");
	static_assert(detail::event_member_is_correct_type<Context>::value,
				  "Context's 'event' member must be of type SM::Event");
	constexpr bool evaluate(const std::optional<Context>& context) const
	{
		return _func ? _func(context) : true;
	}

  private:
	FuncPtr _func;
};

template<typename Context>
constexpr Guard<Context> NullGuard(nullptr);

template<typename Context>
struct Action : public StateFunction
{
  public:
	using FuncPtr = void (*)(std::optional<Context>&);
	constexpr Action(FuncPtr func) : StateFunction(), _func(func) {}
	constexpr ~Action() override = default;
	static_assert(detail::has_event_member<Context>::value, "Context must have an 'event' member");
	static_assert(detail::event_member_is_correct_type<Context>::value,
				  "Context's 'event' member must be of type SM::Event");
	constexpr void execute(std::optional<Context>& context) const
	{
		if(_func)
			_func(context);
	}

  private:
	FuncPtr _func;
};
template<typename Context>
constexpr Action<Context> NullAction(nullptr);

// inline constexpr Action<ValidatedContext> NullAction(nullptr);

template<typename Context, typename... States>
struct Group;
template<typename Context> // NoLint
struct StateBase
{
	using EventHandler = void (*)(const Event&);
	using Actions = Action<Context>;
	using Guards = Guard<Context>;
	using BaseState = StateBase<Context>;
	using Groups = Group<Context>;

  public:
	// Existing constructor with default parameters
	constexpr StateBase(const int id, const char* name,
						const Actions& entryAction = NullAction<Context>,
						const Actions& exitAction = NullAction<Context>,
						const Guards& entryGuard = NullGuard<Context>,
						const Guards& exitGuard = NullGuard<Context>, BaseState* parent = nullptr,
						Groups* group = nullptr) :
		_id(id), _name(name), _entryAction(entryAction), _exitAction(exitAction),
		_entryActionGuard(entryGuard), _exitActionGuard(exitGuard), _listenerCount{}, _listeners{},
		_parent(parent), _group(group)
	{
	}

	constexpr StateBase(BaseState* parent, const int id, const char* name,
						const Actions& entryAction = NullAction<Context>,
						const Actions& exitAction = NullAction<Context>,
						const Guards& entryGuard = NullGuard<Context>,
						const Guards& exitGuard = NullGuard<Context>) :
		StateBase(id, name, entryAction, exitAction, entryGuard, exitGuard, parent, nullptr)
	{
	}
	constexpr StateBase(BaseState* parent, Groups* group, const int id, const char* name,
						const Actions& entryAction = NullAction<Context>,
						const Actions& exitAction = NullAction<Context>,
						const Guards& entryGuard = NullGuard<Context>,
						const Guards& exitGuard = NullGuard<Context>) :
		StateBase(id, name, entryAction, exitAction, entryGuard, exitGuard, parent, group)
	{
	}
	constexpr StateBase(Group<Context>* group, const int id, const char* name,
						const Actions& entryAction = NullAction<Context>,
						const Actions& exitAction = NullAction<Context>,
						const Guards& entryGuard = NullGuard<Context>,
						const Guards& exitGuard = NullGuard<Context>) :
		StateBase(id, name, entryAction, exitAction, entryGuard, exitGuard, nullptr, group)
	{
	}
	static_assert(detail::has_event_member<Context>::value, "Context must have an 'event' member");
	static_assert(detail::event_member_is_correct_type<Context>::value,
				  "Context's 'event' member must be of type SM::Event");

	virtual bool isComposite() const { return false; }
	virtual bool handleEvent(const Event& ev) const { return false; };
	virtual void iterateNestedStates(std::function<void(StateBase&)> func) = 0;
	constexpr bool isActive() { return _active; }
	constexpr int getId() const { return _id; }
	constexpr const char* getName() const { return _name; }
	constexpr bool hasParent() const { return _parent ? true : false; }
	constexpr bool hasGroup() const { return _group ? true : false; }
	constexpr BaseState* getParent() { return _parent; }
	constexpr Groups* getGroup() { return _group; }

	constexpr bool hasEntryAction() const
	{
		return _entryAction != NullAction<Context> ? true : false;
	}
	constexpr bool hasExitAction() const
	{
		return _exitAction != NullAction<Context> ? true : false;
	}
	constexpr bool hasEntryGuard() const
	{
		return _entryActionGuard != NullGuard<Context> ? true : false;
	}
	constexpr bool hasExitGuard() const
	{
		return _exitActionGuard != NullGuard<Context> ? true : false;
	}

	constexpr bool canActOnEntry(Context& ctx) const
	{
		return hasEntryAction() ? _entryActionGuard.evaluate(ctx) : true;
	}
	constexpr bool canActOnExit(Context& ctx) const
	{
		return hasExitAction() ? _exitActionGuard.evaluate(ctx) : true;
	}
	void decativateGroup()
	{
		if(hasGroup())
		{
			if(getGroup()->isActive())
			{
				getGroup()->deactivate();
			}
		}
	}
	constexpr void onEntry(Context& ctx) const
	{
		if(hasGroup())
		{
			if(!getGroup()->isActive())
			{
				getGroup()->activate();
			}
		}
		if(canActOnEntry(ctx))
		{
			_entryAction.execute(ctx);
		}
	}
	constexpr void onExit(Context& ctx) const
	{
		iterateNestedStates([&](StateBase& state) { state.decativateGroup(); });
		if(canActOnExit(ctx))
		{
			_exitAction.execute(ctx);
		}
	}

	void registerEventListener(EventType eventType, EventHandler handler)
	{
		size_t eventTypeIndex = static_cast<size_t>(eventType);
		if(eventTypeIndex < MAX_EVENT_TYPE &&
		   _listenerCount[eventTypeIndex] < MAX_LISTENER_PER_TYPE)
		{
			_listeners[eventTypeIndex][_listenerCount[eventTypeIndex]++] = handler;
		}
	}

	constexpr bool operator==(const StateBase& rhs) const
	{
		return (_id == rhs._id) && (std::strcmp(_name, rhs._name) == 0);
	}

	constexpr bool operator!=(const StateBase& rhs) const { return !(*this == rhs); }
	constexpr int getNestingLevel() const { return _level; }

  protected:
	bool notified_by_parent = false;
	Event new_event;

	constexpr void setParent(StateBase<Context>* parent)
	{
		_parent = parent;
		_level = parent ? parent->_level + 1 : 0;
		if(_level > MAX_NESTING)
		{
			LOG::ERROR("STATE_BASE", "Cant have Nesting more than %d", MAX_NESTING);
		}
	}
	constexpr void setGroup(Group<Context>* group) { _group = group; }
	void dispatchEvent(const Event& event)
	{
		new_event = event;
		notified_by_parent = true;

		if(isComposite())
		{
			iterateNestedStates([&](StateBase& state) { state.dispatchEvent(event); });
		}

		bool handled = handleEvent(event);

		size_t eventTypeIndex = static_cast<size_t>(event.getType());
		if(eventTypeIndex < MAX_EVENT_TYPE)
		{
			for(size_t i = 0; i < _listenerCount[eventTypeIndex]; ++i)
			{
				_listeners[eventTypeIndex][i](event);
			}
		}
	}

  private:
	const int _id;
	const char* _name;
	int _level = 0;
	bool _active = false;
	bool _maintainHistory = false;
	int _entryCount = 0;

	Actions _entryAction;
	Actions _duringAction;
	Actions _exitAction;
	Guards _entryActionGuard;
	Guards _exitActionGuard;
	size_t _listenerCount[MAX_EVENT_TYPE];
	EventHandler _listeners[MAX_EVENT_TYPE][MAX_LISTENER_PER_TYPE];
	Groups* _parent = nullptr;
	Groups* _group = nullptr;
};
template<typename Context>
struct DefaultInnerState : public StateBase<Context>
{
  public:
	constexpr DefaultInnerState() :

		StateBase<Context>(Utility::ID::generate(), "DefaultInnerState")
	{
	}

	void iterateNestedStates(std::function<void(StateBase<Context>&)> func) override {}
};

template<typename Context, typename... InnerElements>
struct State;

namespace details
{
// Checks if T is a State derived from StateBase<Context>
template<typename Context, typename T>
struct is_state_type : std::is_base_of<StateBase<Context>, std::decay_t<T>>
{
};

// Checks if T is a State<> template instantiation
template<typename T>
struct is_state : std::false_type
{
};

template<typename Context, typename... InnerElements>
struct is_state<State<Context, InnerElements...>> : std::true_type
{
};

// Checks if T is a Group<Context, ...>
template<typename T, typename Context>
struct is_a_group : std::false_type
{
};

template<typename... States, typename Context>
struct is_a_group<Group<Context, States...>, Context> : std::true_type
{
};
} // namespace details
template<typename Context, typename... States> // Nolint
struct Group
{
  public:
	static_assert(sizeof...(States) >= 2, "Group must contain at least 2 states.");
	static_assert((details::is_state_type<Context, States>::value && ...),
				  "All elements in Group must be of State type.");

	constexpr Group(const char* name, States... states) : _name(name), _states(states...)
	{
		setGroupForElements(std::index_sequence_for<States...>{});
		setPriority();
	}

	const char* getGroupName() const { return _name; }
	constexpr bool operator==(const Group& rhs) const
	{
		return (std::strcmp(_name, rhs._name) == 0);
	}
	template<typename Func>
	void iterate(Func&& func) const
	{
		iterateGroup<0>(std::forward<Func>(func));
	}

	constexpr const StateBase<Context>& getDefaultState() const
	{
		return std::get<0>(_states); // Default state is the first element
	}
	constexpr void setParent(State<Context>* parent) { _commonParent = parent; }
	constexpr State<Context>* getCommonParent() const { return _commonParent; }
	constexpr void setPriority()
	{
		_priority = std::apply(
			[](const auto&... state) {
				return (0 + ... + (state.getId() * state.getNestingLevel()));
			},
			_states);
	}
	constexpr bool isActive() const { return _active; }
	constexpr void activate() { _active = true; }
	constexpr void deactivate() { _active = false; }
	constexpr int getPriority() const { return _priority; }

  private:
	bool _active = false;
	int _priority = -1;
	const char* _name;
	std::tuple<States...> _states;
	State<Context>* _commonParent;

	template<std::size_t Index = 0, typename Func>
	void iterateGroup(Func&& func) const
	{
		if constexpr(Index < std::tuple_size<decltype(_states)>::value)
		{
			func(std::get<Index>(_states));
			iterateGroup<Index + 1>(std::forward<Func>(func));
		}
	}
	template<std::size_t... Is>
	constexpr void setGroupForElements(std::index_sequence<Is...>)
	{
		(std::get<Is>(_states).setGroup(this), ...);
	}
};
template<typename Context>
constexpr Group<Context> NullGroup("NULL");

template<typename Context, typename... InnerElements>
struct State : public StateBase<Context>
{

	using BaseState = StateBase<Context>;
	using Actions = Action<Context>;
	using Guards = Guard<Context>;
	using Groups = Group<Context>;

	template<typename T>
	using IsState = details::is_state_type<Context, T>;

	template<typename T>
	using IsGroup = details::is_a_group<T, Context>;

	template<typename T>
	static constexpr bool IsState_v = IsState<T>::value;

	template<typename T>
	static constexpr bool IsGroup_v = IsGroup<T>::value;

  public:
	static_assert(sizeof...(InnerElements) == 0 || sizeof...(InnerElements) >= 2,
				  "State must have zero or at least two inner elements.");

	static_assert(
		((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...),
		"All elements of State must be either State or Group types with matching Context.");
	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(BaseState* parent, Groups* group, const Actions& entryAction,
					const Actions& exitAction, const Guards& entryGuard, const Guards& exitGuard,
					const char* name, InnerElements... elements) :
		BaseState(parent, group, Utility::ID::generate(), name, entryAction, exitAction, entryGuard,
				  exitGuard),
		elements_(std::make_tuple(elements...))
	{
		setParentForElements(std::index_sequence_for<InnerElements...>{});
	}
	constexpr State(const char* name, InnerElements... elements) :
		State(nullptr, nullptr, Utility::ID::generate(), name, NullAction<Context>,
			  NullAction<Context>, NullGuard<Context>, NullGuard<Context>, elements...)
	{
	}
	constexpr State(BaseState* parent, const char* name, InnerElements... elements) :
		State(parent, nullptr, Utility::ID::generate(), name, NullAction<Context>,
			  NullAction<Context>, NullGuard<Context>, NullGuard<Context>, elements...)
	{
	}
	constexpr State(Groups* group, const char* name, InnerElements... elements) :
		State(nullptr, group, Utility::ID::generate(), name, NullAction<Context>,
			  NullAction<Context>, NullGuard<Context>, NullGuard<Context>, elements...)
	{
	}
	constexpr State(const Actions& entryAction, const Actions& exitAction, const char* name,
					InnerElements... elements) :
		State(nullptr, nullptr, Utility::ID::generate(), name, entryAction, exitAction,
			  NullGuard<Context>, NullGuard<Context>, elements...)
	{
	}

	bool isComposite() const override { return sizeof...(InnerElements) > 0; }

	constexpr auto getDefaultState() const { return getDefaultStateImpl<0>(); }

	template<typename Func>
	void iterateStates(Func&& func) const
	{
		iterateElements<0>(std::forward<Func>(func));
	}
	void iterateNestedStates(std::function<void(BaseState&)> func) const override
	{

		iterateElements<0>([&func](const auto& element) {
			using ElementType = std::decay_t<decltype(element)>;
			if constexpr(std::is_base_of_v<BaseState, ElementType>)
			{
				func(element);
			}
			else if constexpr(IsGroup_v<ElementType>)
			{
				element.iterate([&func](const auto& groupElement) {
					using GroupElementType = std::decay_t<decltype(groupElement)>;
					if constexpr(std::is_base_of_v<BaseState, GroupElementType>)
					{
						func(groupElement);
					}
				});
			}
		});
	}

	bool handleEvent(const Event& event) const override
	{
		LOG::INFO("Handling event...");
		bool handled = true; /* custom handling logic */
		;
		if(!handled && this->hasParent())
		{
			handled = this->getParent()->handleEvent(event);
		}
		return handled;
	}
	void propagateEvent(const Event& event)
	{
		if(this->isComposite())
		{
			LOG::INFO("State", "\nState %s is composite, dispatching event to nested states",
					  this->getName());
			this->dispatchEvent(event);
		}
		else if(this->hasParent() && !this->notified_by_parent)
		{
			this->getParent().handleEvent(event);
		}
		else
		{
			LOG::INFO("State", "\nState %s is not composite handling the event instead",
					  this->getName());
			this->handleEvent(event);
		}
	}

  private:
	std::tuple<InnerElements...> elements_;
	static constexpr DefaultInnerState<Context> defaultState{};

	template<std::size_t Index = 0>
	constexpr auto getDefaultStateImpl() const
	{
		if constexpr(Index < std::tuple_size<decltype(elements_)>::value)
		{
			const auto& element = std::get<Index>(elements_);
			using ElementType = std::decay_t<decltype(element)>;
			if constexpr(IsState_v<ElementType>)
			{
				return element;
			}
			else
			{
				return element.getDefaultState();
			}
		}
		else
		{
			return defaultState;
		}
	}

	template<std::size_t Index = 0, typename Func>
	void iterateElements(Func&& func) const
	{
		if constexpr(Index < std::tuple_size<decltype(elements_)>::value)
		{
			const auto& element = std::get<Index>(elements_);
			func(element);
			using ElementType = std::decay_t<decltype(element)>;
			if constexpr(IsState_v<ElementType>)
			{
				if(element.isComposite())
				{
					element.iterateStates(func);
				}
			}
			else if constexpr(IsGroup_v<ElementType>)
			{
				element.iterate(func);
			}
			iterateElements<Index + 1>(std::forward<Func>(func));
		}
	}
	template<std::size_t... Is>
	constexpr void setParentForElements(std::index_sequence<Is...>)
	{
		(processElement(std::get<Is>(elements_)), ...);
	}

	template<typename Element>
	constexpr void processElement(Element& element)
	{
		using ElementType = std::decay_t<decltype(element)>;
		if constexpr(std::is_base_of_v<StateBase<Context>, ElementType>)
		{
			element.setParent(this);
		}
		else if constexpr(details::is_a_group<ElementType, Context>::value)
		{
			element.setParent(this); // Set Groups parent
			element.iterate([this](auto& groupElement) { groupElement.setParent(this); });
		}
	}
};

template<typename Context>
constexpr State<Context> NullState(-1, "NULL");
template<typename Context>
struct TransitionPath

{
	using tS = State<Context>;
	using Actions = Action<Context>;
	using Guards = Guard<Context>;
	using Groups = Group<Context>;

  public:
	constexpr TransitionPath(int id, const Guards& guard, const tS* to,
							 const Event& event = NullEvent,
							 const Actions& onTransitionAction = NullAction<Context>)
	{
	}

  private:
	const Guards& _guard;
	const tS* _to;
	const Event& _event;
	const Actions& _onTransitionAction;
	int _priority;
};

// Update Transition to use pointers
template<typename Context>
struct Transition
{
	using tS = State<Context>;
	using Actions = Action<Context>;
	using Guards = Guard<Context>;
	using Groups = Group<Context>;

  public:
	const int _id;

	constexpr Transition(int id, const tS* from, const tS* to, const Guards& guard,
						 const Event& event = NullEvent,
						 const Actions& onTransitionAction = NullAction<Context>) :
		_id(id), _from(from), _to(to), _guard(guard), _event(event),
		_onTransitionAction(onTransitionAction)
	{
		if((from->hasParent() && to->hasParent()) && (from->getParent() == to->getParent()))
		{
			_commonParent = from->getParent();
		}
		if((from->hasGroup() && to->hasGroup()) && (from->getGroup() == to->getGroup()))
		{
			_commonGroup = from->getGroup();
		}
		setTransitionType();
		setPriority();
	}

	bool canTransit(const std::optional<Context>& context) const
	{
		if(_type == TransitionType::TRANS_INTERNAL)
		{
			return _guard.evaluate(context);
		}
		else
		{

			if(!_guard.evaluate(context) || !_from->canActOnExit(context) ||
			   !_to->canActOnEntry(context))
				return false;

			return (sameGroup() || !groupsShareCommonParent());
		}
	}
	constexpr bool sameLevel() const
	{
		return _from->getNestingLevel() == _to->getNestingLevel() ? true : false;
	}
	constexpr bool hasCommonAncestor() const { return _commonParent ? true : false; }

	constexpr tS* getAncestor() { return _commonParent; }
	constexpr Groups* getCommonGroup() { return _commonGroup; }
	constexpr bool sameGroup() const
	{
		return (_from->getGroup() == _to->getGroup()) ? true : false;
	}
	constexpr bool hasCommonGroup() const
	{

		return sameGroup() && _from->hasGroup() && _to->hasGroup() ? true : false;
	}
	constexpr bool groupsShareCommonParent()
	{
		if(_from->hasGroup() && _to->hasGroup())
		{
			const auto* fromGroup = _from->getGroup();
			const auto* toGroup = _to->getGroup();
			bool sameGroup = (fromGroup == toGroup);
			return ((fromGroup->getCommonParent() == toGroup->getCommonParent()) && (!sameGroup));
		}
		else
		{
			return false;
		}
	}

	tS* executeTransition(std::optional<Context>& context) const
	{
		if(!context.has_value())
			return false;
		if(canTransit(context))
		{
			_onTransitionAction.execute(context);
			return _to;
		}
		return nullptr; // No transition
	}
	void setTransitionType()
	{
		bool fromOrToParent = (_to->hasParent() && (_to->getParent() == _from)) ||
							  (_from->hasParent() && (_from->getParent() == _to));

		if(fromOrToParent)
		{
			_type = TransitionType::TRANS_INTERNAL;
		}
		else
		{
			_type = TransitionType::TRANS_EXTERNAL;
		}
	}
	void setPriority()
	{ // Internal transition are set to be highest so that the composite state is stable before
	  // intearcting for new external transition
		if(_type == TransitionType::TRANS_INTERNAL)
		{
			_priority = _from->getNestingLevel() * _from->getId();
		}
		else
		{
			_priority = _from->getNestingLevel() * _from->getId() + MAX_STATE_SM;
		}
	}

	constexpr TransitionType getTransitionType() const { return _type; }
	constexpr int getPrioirty() const { return _priority; }
	const tS* getFromState() const { return _from; }
	const Event& getEvent() const { return _event; }
	const tS* getToState() const { return _to; }
	const Actions& getAction() const { return _onTransitionAction; }
	const Guards& getGuard() const { return _guard; }

  private:
	const tS* _from;
	const tS* _to;
	const Guards& _guard;
	const Event& _event;
	const Actions& _onTransitionAction;
	const tS* _commonParent;
	const Groups* _commonGroup;
	TransitionType _type = TransitionType::UNDEFINED;
	int _priority;
};

template<typename Context>
constexpr Transition<Context> NullTransition{
	-1, NullState<Context>, NullEvent, NullState<Context>, NullGuard<Context>, NullAction<Context>};
template<typename Context>
struct TransitionTable
{
	std::unordered_map<int, std::array<Transition<Context>, MAX_STATE_PER_GROUP>> stateTransitions;
	std::unordered_map<int, size_t> stateTransitionCounts;
	std::unordered_map<const Group<Context>*, std::array<Transition<Context>, MAX_STATE_PER_GROUP>>
		groupTransitions;
	std::unordered_map<const Group<Context>*, size_t> groupTransitionCounts;

	template<typename... Transitions>
	constexpr TransitionTable(Transitions... transitions)
	{
		static_assert(sizeof...(Transitions) <= MAX_TRANSITIONS,
					  "Too many transitions for capacity");

		processTransitions(std::make_index_sequence<sizeof...(Transitions)>{},
						   std::forward_as_tuple(transitions...));
	}

  private:
	template<std::size_t... Is, typename Tuple>
	void processTransitions(std::index_sequence<Is...>, Tuple&& transitions)
	{
		(processTransition(std::get<Is>(transitions)), ...);
	}

	void processTransition(const Transition<Context>& trans)
	{
		int stateId = trans.getFromState()->getId();
		auto fromState = trans.getFromState();
		auto& count = stateTransitionCounts[stateId];
		if(count < MAX_STATE_PER_GROUP)
		{
			stateTransitions[stateId][count++] = trans;
		}

		// Handle group-based transitions
		if(fromState->hasGroup())
		{
			auto group = fromState->getGroup();
			auto& groupTrans = groupTransitions[group];
			auto& count = groupTransitionCounts[group];
			if(count < MAX_STATE_PER_GROUP)
			{
				groupTrans[count++] = trans;
			}
		}
	}

  public:
	struct TransitionRange
	{
		const Transition<Context>* _begin;
		const Transition<Context>* _end;
		const Transition<Context>* begin() const { return _begin; }
		const Transition<Context>* end() const { return _end; }
		bool empty() const { return _begin == _end; }
	};

	TransitionRange findTransitions(const State<Context>* currentState, const Event& event) const
	{
		// Check state transitions first
		int stateId = currentState->getId();
		if(auto it = stateTransitions.find(stateId); it != stateTransitions.end())
		{
			size_t count = stateTransitionCounts.at(stateId);
			for(size_t i = 0; i < count; ++i)
			{
				if(it->second[i].getEvent() == event)
				{
					return {&it->second[i], &it->second[i] + 1};
				}
			}
		}

		// Then check group transitions
		if(currentState->hasGroup())
		{
			if(auto it = groupTransitions.find(currentState->getGroup());
			   it != groupTransitions.end())
			{
				const auto& transArray = it->second;
				size_t count = groupTransitionCounts.at(currentState->getGroup());
				for(size_t i = 0; i < count; ++i)
				{
					if(transArray[i].getEvent() == event)
					{
						return {&transArray[i], &transArray[i] + 1};
					}
				}
			}
		}

		return {nullptr, nullptr};
	}
};
template<typename Context, size_t N>
class FSM
{
  public:
	FSM(const State<Context>* topState, const TransitionTable<Context>& transitionsTable) :
		_topState(topState), _transitionsTable(transitionsTable)
	{
		initializeActiveStates();
		analyzeGroups();
	}

	void processEvent(const Event& event, std::optional<Context>& context)
	{
		// Structure to hold transition and its nesting level
		struct TransitionInfo
		{
			const Transition<Context>* trans;
			int level;
		};
		std::vector<TransitionInfo> transitionsToPerform;

		// Helper function to add a transition if its guard is true
		auto addTransition = [&](const State<Context>* state, int level) {
			auto range = _transitionsTable.findTransitions(state, event);
			for(const auto& trans: range)
			{
				if(evaluateCombinedGuard(trans, state, context))
				{
					const State<Context>* toState = trans.getToState();
					if(state->getGroup() != toState->getGroup() &&
					   findLCA(state, toState) != nullptr)
					{
						LOG::WARNING(
							"FSM",
							"Transition from %s to %s disallowed: groups with common ancestor",
							state->getName(), toState->getName());
						continue;
					}
					transitionsToPerform.push_back({&trans, level});
				}
			}
		};

		// Collect transition from the main active state
		if(_activeMainState)
		{
			addTransition(_activeMainState, _activeMainState->getNestingLevel());
		}

		// Collect transitions from active group states
		for(size_t i = 0; i < _groupCount; ++i)
		{
			if(_activeGroupStates[i])
			{
				addTransition(_activeGroupStates[i], _activeGroupStates[i]->getNestingLevel());
			}
		}

		// Sort transitions by level (descending order: deeper levels first)
		std::sort(transitionsToPerform.begin(), transitionsToPerform.end(),
				  [](const auto& a, const auto& b) { return a.level > b.level; });

		// Execute all collected transitions in order
		for(const auto& info: transitionsToPerform)
		{
			performTransition(*info.trans, context);
		}

		// Log if no transitions occurred
		if(transitionsToPerform.empty())
		{
			LOG::INFO("FSM", "No transition for event %s", event.toString());
		}
	}

	void updateContext(std::optional<Context>& context) { _ctx = context; }
	void updateContext(Event& ev) { _ctx.event = ev; }

  private:
	std::optional<Context> _ctx;
	const State<Context>* _topState;
	const State<Context>* _activeMainState = nullptr; // Main hierarchy active state
	std::array<const State<Context>*, MAX_GROUP_SM> _activeGroupStates = {nullptr}; // Group states
	std::array<const State<Context>*, MAX_GROUP_SM> _prevGroupStates = {nullptr};
	std::array<const Group<Context>*, MAX_GROUP_SM> _allGroups = {nullptr}; // All unique groups
	size_t _groupCount = 0;
	const TransitionTable<Context>& _transitionsTable;

	void initializeActiveStates()
	{
		_activeMainState = getDeepestDefaultState(_topState);
		if(!_activeMainState)
		{
			LOG::ERROR("FSM", "Failed to initialize active state due to cycle or null top state");
		}
		std::vector<const State<Context>*> entryPath;
		for(auto* p = _activeMainState; p != nullptr; p = p->getParent())
		{
			entryPath.push_back(p);
		}
		std::reverse(entryPath.begin(), entryPath.end());
		for(auto* state: entryPath)
		{
			enterState(state, _ctx);
		}
	}
	void analyzeGroups()
	{
		std::unordered_set<const Group<Context>*> visitedGroups;
		collectGroups(_topState, 0, visitedGroups);
		_groupCount = 0;
		for(const auto* group: visitedGroups)
		{
			if(_groupCount < MAX_GROUP_SM)
			{
				_allGroups[_groupCount++] = group;
			}
			else
			{
				LOG::WARNING("FSM", "Number of groups exceeds MAX_GROUP_SM (%d)", MAX_GROUP_SM);
				break;
			}
		}
	}
	void collectGroups(const State<Context>* state, int level,
					   std::vector<const Group<Context>*>& visitedGroups)
	{
		if(!state->isComposite())
			return;

		state->iterateStates([&](const auto& element) {
			using ElementType = std::decay_t<decltype(element)>;
			if constexpr(State<Context>::template IsGroup_v<ElementType>)
			{
				const Group<Context>* group = &element;
				if(std::find(visitedGroups.begin(), visitedGroups.end(), group) ==
				   visitedGroups.end())
				{
					visitedGroups.push_back(group);
				}
			}
			else if constexpr(State<Context>::template IsState_v<ElementType>)
			{
				collectGroups(&element, level + 1, visitedGroups);
			}
		});
	}
	const State<Context>* getDeepestDefaultState(const StateBase<Context>* state) const
	{
		std::unordered_set<const StateBase<Context>*> visited;
		while(state->isComposite())
		{
			if(visited.count(state))
			{
				LOG::ERROR("FSM", "Cycle detected in default states for state %s",
						   state->getName());
				return nullptr;
			}
			visited.insert(state);
			auto* nextState = state->getDefaultState();
			if(nextState == state)
				break; // Self-reference detected
			state = nextState;
		}
		return static_cast<const State<Context>*>(state);
	}
	const State<Context>* findLCA(const StateBase<Context>* s1, const StateBase<Context>* s2) const
	{
		std::vector<const StateBase<Context>*> s1Ancestors;
		for(auto* p = s1; p; p = p->getParent())
			s1Ancestors.push_back(p);
		for(auto* p = s2; p; p = p->getParent())
		{
			if(std::find(s1Ancestors.begin(), s1Ancestors.end(), p) != s1Ancestors.end())
			{
				return static_cast<const State<Context>*>(p);
			}
		}
		return _topState;
	}
	void performTransition(const Transition<Context>& trans, std::optional<Context>& context)
	{
		const State<Context>* oldState = trans.getFromState();
		const State<Context>* newState = trans.getToState();
		if(!oldState || !newState)
			return;

		const State<Context>* lca = findLCA(oldState, newState);
		if(!lca)
			return;

		// Exit up to LCA
		const State<Context>* currentExit = oldState;
		while(currentExit && currentExit != lca)
		{
			exitState(currentExit, context);
			currentExit = currentExit->getParent();
		}

		// Check if transitioning to a group's inner state
		std::vector<const State<Context>*> entryPath;
		for(auto* p = newState; p && p != lca; p = p->getParent())
		{
			entryPath.push_back(p);
		}
		std::reverse(entryPath.begin(), entryPath.end());

		if(newState->hasGroup() && entryPath.size() > 1)
		{
			const Group<Context>* group = newState->getGroup();
			const State<Context>* groupParent = group->getCommonParent();
			if(groupParent &&
			   std::find(entryPath.begin(), entryPath.end(), groupParent) == entryPath.end())
			{
				entryPath.insert(entryPath.begin(), groupParent);
			}
		}

		// Enter each state in the adjusted entry path
		for(auto* state: entryPath)
		{
			enterState(state, context);
		}

		trans.getAction().execute(context);
		if(oldState == _activeMainState)
			_activeMainState = newState;
	}
	void exitState(const State<Context>* state, std::optional<Context>& context)
	{
		if(!state)
			return;
		state->onExit(context);
		if(state->hasGroup())
		{
			size_t groupIdx = getGroupIndex(state->getGroup());
			if(groupIdx < _groupCount)
			{
				_activeGroupStates[groupIdx] = nullptr;
			}
		}
		if(state->isComposite())
		{
			state->iterateStates([&](const auto& element) {
				using ElementType = std::decay_t<decltype(element)>;
				if constexpr(State<Context>::template IsGroup_v<ElementType>)
				{
					size_t idx = getGroupIndex(&element);
					if(idx < _groupCount && _activeGroupStates[idx])
					{
						_activeGroupStates[idx]->onExit(context);
						_activeGroupStates[idx] = nullptr;
					}
				}
			});
		}
	}

	void enterState(const State<Context>* state, std::optional<Context>& context)
	{
		if(!state)
			return;
		state->onEntry(context);
		if(state->hasGroup())
		{
			size_t idx = getGroupIndex(state->getGroup());
			if(idx < _groupCount)
			{
				_activeGroupStates[idx] = state;
			}
		}
		if(state->isComposite())
		{
			const State<Context>* defaultState = getDeepestDefaultState(state);
			if(defaultState && defaultState != state)
			{
				enterState(defaultState, context);
			}
			state->iterateStates([&](const auto& element) {
				using ElementType = std::decay_t<decltype(element)>;
				if constexpr(State<Context>::template IsGroup_v<ElementType>)
				{
					size_t idx = getGroupIndex(&element);
					if(idx < _groupCount && !_activeGroupStates[idx])
					{
						const State<Context>* groupDefault =
							getDeepestDefaultState(element.getDefaultState());
						if(groupDefault)
						{
							enterState(groupDefault, context);
							_activeGroupStates[idx] = groupDefault;
						}
					}
				}
			});
		}
	}

	size_t getGroupIndex(const Group<Context>* group) const
	{
		for(size_t i = 0; i < _groupCount; ++i)
		{
			if(_allGroups[i] == group)
				return i;
		}
		return MAX_GROUP_SM; // Indicate invalid
	}

	/** Combine state's guards with transition's guard */
	bool evaluateCombinedGuard(const Transition<Context>& trans, const StateBase<Context>* state,
							   const std::optional<Context>& context) const
	{
		return trans.canTransit(context);
	}
};

} // namespace SM
