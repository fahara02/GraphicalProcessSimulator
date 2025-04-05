#pragma once
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>
#include "Logger.hpp"
#include <algorithm>
#include <unordered_set>
#include <atomic>

namespace SM
{
/*
 DESIGN GOAL RULES
1.THERE IS ALWAYS A TOP_STATE
2.STATE  CAN TRANSIT TO ANOTHER  STATE OF SAME LEVEL ONNLY AT FIRST ,THEN IF DICTATED BY
TRANSITION TO THE INNER STATE OF THE TO STATE FROM THE PARENT STATE(2 TRANSITION  SEQUENTIAL )
2.1 A COMPOSITE STATE WHEN ACTIVE  BY  DEFUALT WILL ACTIVATE ALL ITS NESTED DEFAULT STATE,
3.GROUP/GROUPS AND ITS MEMBERS ALWAYS HAVE COMMON PARENT as THERE IS ALWAYS A TOP STATE
4.TRANSITION BETWEEN GROUPS WITH COMMON PARENT NOT ALLOWED
5.TRANSITION BETWEEN GROUPS WITH DIFFERENT PARENT ALLOWED
6.TRANSITION BETWEEN A STATE TO A GROUP WILL HAPPEN IN TWO STEP STATE TO GROUP PARENT THEN
IMMEDIATELY GROUP PARENT TO GROUP DEFAULT STATE // NO LINT
7.FOR A SAME EVENT IF TRANSITION GUARD IS TRUE MULTIPLE GROUPS WITH SAME LEVEL WILL TRANSIT //NO
LINT
8.FOR A SAME EVENT IF TRANSITION GUARD IS TRUE MULTIPLE NESTED STATE WILL TRANSIT BUT IN ORDER
OF LOWER LEVEL FIRST //NO LINT
9.LOWEST LEVEL STATE WILL PROPAGATE EVENT UPWARDS //NO LINT
10.HIGHER LEVEL STATE WILL PROPAGATE LEVELS BOTH WAYS UPWARDS AND DOWNWARDS //NO LINT
11.HIGHEST LEVEL STATE WILL PROPAGATE EVENT  DOWSNWARDS //NO LINT

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

class Utility
{
  public:
	class ID
	{
	  public:
		inline static int generate() { return ++id_counter; }

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

// inline constexpr Action<ValidatedContext> NullAction(nullptr);
template<typename Context>
constexpr Action<Context> NullAction(nullptr);

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
	constexpr int getId() const { return _id; }
	constexpr const char* getName() const { return _name; }
	constexpr bool hasParent() const { return _parent ? true : false; }
	constexpr bool hasGroup() const { return _group ? true : false; }
	constexpr BaseState* getParent() { return _parent; }
	constexpr Groups* getGroup() { return _group; }

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

	constexpr void onEntry(Context& ctx) const
	{
		if(canActOnEntry(ctx))
		{
			_entryAction.execute(ctx);
		}
	}
	constexpr void onExit(Context& ctx) const
	{
		if(canActOnExit(ctx))
		{
			_exitAction.execute(ctx);
		}
	}

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

	virtual void iterateNestedStates(std::function<void(StateBase&)> func) = 0;

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

  private:
	const int _id;
	const char* _name;
	int _level = 0;

	Actions _entryAction;
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

		StateBase<Context>(0, "DefaultInnerState")
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

  private:
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
	void iterateNestedStates(std::function<void(BaseState&)> func) override
	{
		LOG::INFO("State", "\nIterating Nested States...from State %s", this->getName());
		iterateElements<0>([&func](const auto& element) {
			using ElementType = std::decay_t<decltype(element)>;
			if constexpr(std::is_base_of_v<BaseState, ElementType>)
			{
				func(const_cast<ElementType&>(element));
			}
			else if constexpr(IsGroup_v<ElementType>)
			{
				element.iterate([&func](const auto& groupElement) {
					using GroupElementType = std::decay_t<decltype(groupElement)>;
					if constexpr(std::is_base_of_v<BaseState, GroupElementType>)
					{
						func(const_cast<GroupElementType&>(groupElement));
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
		else if(this->hasParent() && this->notified_by_parent)
		{
			this->getParent().handleEvent(event);
			this->notified_by_parent = false;
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
		_id(id), _fromState(from), _toState(to), _guard(guard), _event(event),
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
	}

	// In Transition::canTransit, adjust conditions
	bool canTransit(const std::optional<Context>& context) const
	{
		bool sameGroup = (_fromState->getGroup() == _toState->getGroup());
		bool commonParent = hasCommonAncestor();

		// Allow transitions within the same group or between groups with different parents
		return _guard.evaluate(context) &&
			   (sameGroup || (!commonParent && _fromState->getGroup() != _toState->getGroup())) &&
			   _fromState->canActOnExit(context) && _toState->canActOnEntry(context);
	}
	constexpr bool hasCommonAncestor() const { return _commonParent ? true : false; }
	constexpr tS* getAncestor() { return _commonParent; }
	constexpr bool hasCommonGroup() const { return _commonGroup ? true : false; }

	constexpr Groups* getCommonGroup() { return _commonGroup; }

	tS* executeTransition(std::optional<Context>& context) const
	{
		if(!context.has_value())
			return false;
		if(canTransit(context))
		{
			_onTransitionAction.execute(context);
			return _toState;
		}
		return nullptr; // No transition
	}

	const tS* getFromState() const { return _fromState; }
	const Event& getEvent() const { return _event; }
	const tS* getToState() const { return _toState; }
	const Actions& getAction() const { return _onTransitionAction; }
	const Guards& getGuard() const { return _guard; }

  private:
	const tS* _fromState;
	const tS* _toState;
	const Guards& _guard;
	const Event& _event;
	const Actions& _onTransitionAction;
	const tS* _commonParent;
	const Groups* _commonGroup;
};

template<typename Context>
constexpr Transition<Context> NullTransition{
	-1, NullState<Context>, NullEvent, NullState<Context>, NullGuard<Context>, NullAction<Context>};
template<typename Context>
struct TransitionTable
{
	std::array<std::array<Transition<Context>, MAX_STATE_PER_GROUP>, MAX_INNER_STATE>
		stateTransitions;
	std::array<size_t, MAX_INNER_STATE> stateTransitionCounts{};
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
		const State<Context>* fromState = trans.getFromState();

		// Handle state-based transitions
		if(const int stateId = fromState->getId(); stateId >= 0 && stateId < MAX_INNER_STATE)
		{
			auto& count = stateTransitionCounts[stateId];
			if(count < MAX_STATE_PER_GROUP)
			{
				stateTransitions[stateId][count++] = trans;
			}
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
		if(const int stateId = currentState->getId(); stateId >= 0 && stateId < MAX_INNER_STATE)
		{
			const size_t count = stateTransitionCounts[stateId];
			for(size_t i = 0; i < count; ++i)
			{
				if(stateTransitions[stateId][i].getEvent() == event)
				{
					return {&stateTransitions[stateId][i], &stateTransitions[stateId][i] + 1};
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
					transitionsToPerform.push_back({&trans, level});
					break; // One transition per state per event
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
		for(size_t i = 0; i < MAX_GROUP_SM; ++i)
		{
			_activeGroupStates[i] = nullptr;
			_prevGroupStates[i] = nullptr;
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
		while(state->isComposite())
		{
			auto* nextState = state->getDefaultState();
			if(nextState == state)
			{ // Prevent infinite loop
				LOG::ERROR("FSM", "Infinite loop in getDeepestDefaultState for state %s",
						   state->getName());
				break;
			}
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
		const State<Context>* lca = findLCA(oldState, newState);

		// Exit up to LCA
		const State<Context>* currentExit = oldState;
		while(currentExit != lca)
		{
			exitState(currentExit, context);
			currentExit = currentExit->getParent();
		}

		// Perform first transition
		trans.getAction().execute(context);
		newState->onEntry(context);

		// Second transition to inner state if composite
		if(newState->isComposite())
		{
			const State<Context>* innerState = getDeepestDefaultState(newState->getDefaultState());
			context->event = Event(EventType::INTERNAL_EV, "EnterInner");
			innerState->onEntry(context);
			if(!oldState->hasGroup())
			{
				_activeMainState = innerState;
			}
			else
			{
				size_t groupIdx = getGroupIndex(oldState->getGroup());
				_activeGroupStates[groupIdx] = innerState;
			}
		}
		else
		{
			if(!oldState->hasGroup())
			{
				_activeMainState = newState;
			}
			else
			{
				size_t groupIdx = getGroupIndex(oldState->getGroup());
				_activeGroupStates[groupIdx] = newState;
			}
		}
	}

	void exitState(const State<Context>* state, std::optional<Context>& context)
	{
		state->onExit(context);
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
		state->onEntry(context);
		if(state->isComposite())
		{
			state->iterateStates([&](const auto& element) {
				using ElementType = std::decay_t<decltype(element)>;
				if constexpr(State<Context>::template IsGroup_v<ElementType>)
				{
					size_t idx = getGroupIndex(&element);
					if(idx < _groupCount)
					{
						const State<Context>* defaultState =
							getDeepestDefaultState(element.getDefaultState());
						defaultState->onEntry(context);
						_activeGroupStates[idx] = defaultState;
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
