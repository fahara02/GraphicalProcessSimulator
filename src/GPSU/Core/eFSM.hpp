#pragma once
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>
#include "Logger.hpp"
#include <algorithm>

namespace SM
{

constexpr int MAX_INNER_STATE = 10;
constexpr int MAX_STATE_PER_GROUP = 5;
constexpr int MAX_NESTING = 3;
static constexpr size_t MAX_EVENT_TYPE = 4;
static constexpr size_t MAX_LISTENER_PER_TYPE = 5;

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
	constexpr Event& operator=(Event& other)
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

struct StateFunction
{
  public:
	constexpr StateFunction() : _id(-1), id_updated(false) {}
	constexpr StateFunction(int id) : _id(id), id_updated(true) {}
	constexpr virtual ~StateFunction() = default;
	constexpr int getId() const { return _id; }
	constexpr bool setId(int id)
	{
		if(!id_updated)
		{
			_id = id;
			id_updated = true;
			return true;
		}
		return false;
	}

  private:
	int _id;
	bool id_updated = false;
};

template<typename Context>
struct Guard : public StateFunction
{
  public:
	using FuncPtr = bool (*)(const std::optional<Context>&);
	constexpr Guard(FuncPtr func) : StateFunction(), _func(func) {}
	constexpr Guard(int id, FuncPtr func) : StateFunction(id), _func(func) {}
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
	constexpr Action(int id, FuncPtr func) : StateFunction(id), _func(func) {}
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
		_entryAction.setId(_id);
		_exitAction.setId(_id);
		_entryActionGuard.setId(_id);
		_exitActionGuard.setId(_id);
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
	virtual void handleEvent(const Event& ev) const;
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

		handleEvent(event);

		size_t eventTypeIndex = static_cast<size_t>(event.getType());
		if(eventTypeIndex < MAX_EVENT_TYPE)
		{
			for(size_t i = 0; i < _listenerCount[eventTypeIndex]; ++i)
			{
				_listeners[eventTypeIndex][i](event);
			}
		}

		if(isComposite())
		{
			iterateNestedStates([&](StateBase& state) { state.dispatchEvent(event); });
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

  private:
	const int _id;
	const char* _name;

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

	constexpr Group(const char* name, States... states) : _name(name), _states(states...) {}

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

  private:
	const char* _name;
	std::tuple<States...> _states;

	template<std::size_t Index = 0, typename Func>
	void iterateGroup(Func&& func) const
	{
		if constexpr(Index < std::tuple_size<decltype(_states)>::value)
		{
			func(std::get<Index>(_states));
			iterateGroup<Index + 1>(std::forward<Func>(func));
		}
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
	constexpr State(int id, const char* name, InnerElements... elements) :
		BaseState(id, name, NullAction<Context>, NullAction<Context>, NullGuard<Context>,
				  NullGuard<Context>),
		elements_(std::make_tuple(elements...))
	{
	}

	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(int id, const Actions& entryAction, const Actions& exitAction, const char* name,
					InnerElements... elements) :
		BaseState(id, name, entryAction, exitAction, NullGuard<Context>, NullGuard<Context>),
		elements_(std::make_tuple(elements...))
	{
	}
	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(int id, const Actions& entryAction, const Actions& exitAction,
					const Guards& entryGuard, const Guards& exitGuard, const char* name,
					InnerElements... elements) :
		BaseState(id, name, entryAction, exitAction, entryGuard, exitGuard),
		elements_(std::make_tuple(elements...))
	{
	}
	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(BaseState* parent, int id, const Actions& entryAction,
					const Actions& exitAction, const Guards& entryGuard, const Guards& exitGuard,
					const char* name, InnerElements... elements) :
		BaseState(parent, id, name, entryAction, exitAction, entryGuard, exitGuard),
		elements_(std::make_tuple(elements...))
	{
	}
	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(BaseState* parent, Group<Context> group, int id, const Actions& entryAction,
					const Actions& exitAction, const Guards& entryGuard, const Guards& exitGuard,
					const char* name, InnerElements... elements) :
		BaseState(parent, group, id, name, entryAction, exitAction, entryGuard, exitGuard),
		elements_(std::make_tuple(elements...))
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

	void handleEvent(const Event& event) const override
	{
		LOG::INFO("State", "\n Handling Event %s to State %s", event.toString(), this->getName());
	};
	void propagateEvent(const Event& event)
	{
		if(this->isComposite())
		{
			LOG::INFO("State", "\nState %s is composite, dispatching event to nested states",
					  this->getName());
			this->dispatchEvent(event);
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

	bool canTransit(const std::optional<Context>& context) const
	{
		return _guard.evaluate(context) && (_fromState->canActOnExit(context)) &&
			   (_toState->canActOnEntry(context)) && !hasDifferentGroup();
	}
	constexpr bool hasCommonAncestor() const { return _commonParent ? true : false; }
	constexpr tS* getAncestor() { return _commonParent; }
	constexpr bool hasCommonGroup() const { return _commonGroup ? true : false; }
	constexpr bool hasDifferentGroup() const
	{
		return ((_fromState->hasGroup() && _toState->hasGroup()) &&
				(_fromState->getGroup() != _toState->getGroup())) ?
				   true :
				   false;
	}
	constexpr Groups* getCommonGroup() { return _commonGroup; }

	tS* executeTransition(std::optional<Context>& context) const
	{
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

template<typename Context, typename... Transitions>
struct TransitionsTable
{
  public:
	constexpr TransitionsTable(Transitions... transitions) :
		transitions_(std::make_tuple(transitions...))
	{
	}
	template<std::size_t Index = 0>
	constexpr const Transition<Context>& getTransition(std::size_t index) const
	{
		if constexpr(sizeof...(Transitions) == 0)
		{
			// Handle the empty tuple case
			LOG::ERROR("SM_TST", "TransitionTable is empty.\n");
			return NullTransition<Context>;
		}
		else
		{
			if(index == Index)
			{
				return std::get<Index>(transitions_);
			}
			else if constexpr(Index + 1 < sizeof...(Transitions))
			{
				return getTransition<Index + 1>(index);
			}
			else
			{
				LOG::ERROR("SM_TST", "Index out of bounds: %zu\n", index);
				return NullTransition<Context>;
			}
		}
	}
	constexpr const Transition<Context>& operator[](std::size_t index) const
	{
		return getTransition(index);
	}

	constexpr const Transition<Context>& findTransition(const State<Context>& currState,
														const Event& evt) const
	{
		return findTransitionRecursive(currState, evt,
									   std::make_index_sequence<sizeof...(Transitions)>{});
	}
	void printTransitions() const
	{
		printHelper(std::make_index_sequence<sizeof...(Transitions)>{});
	}

  private:
	std::tuple<Transitions...> transitions_;
	template<std::size_t Index, std::size_t... Rest>
	constexpr const Transition<Context>&
		findTransitionRecursive(const State<Context>& currState, const Event& evt,
								std::index_sequence<Index, Rest...>) const
	{
		if(std::get<Index>(transitions_).getFromState() == currState &&
		   std::get<Index>(transitions_).getEvent() == evt)
		{
			return std::get<Index>(transitions_);
		}
		if constexpr(sizeof...(Rest) > 0)
		{
			return findTransitionRecursive(currState, evt, std::index_sequence<Rest...>{});
		}
		return NullTransition<Context>;
	}
	template<std::size_t... Is>
	void printHelper(custom::index_sequence<Is...>) const
	{
		int dummy[] = {
			0,
			(LOG::INFO("SM_TST", "Transition %d: From %s, Event %s, To %s, Action %s, Guard %s\n",
					   std::get<Is>(transitions_).id,
					   std::get<Is>(transitions_).getFromState().getName(),
					   std::get<Is>(transitions_).getEvent().getName(),
					   std::get<Is>(transitions_).getToState().getName(),
					   std::get<Is>(transitions_).getAction().getName(),
					   std::get<Is>(transitions_).getGuard().getName()),
			 0)...};
		(void)dummy;
	}
};

template<typename Context>
using TransitionTable = std::array<Transition<Context>, MAX_STATE_PER_GROUP>;

template<typename Context, size_t N>
class FSM
{
  public:
	FSM(const State<Context>* topState,
		const std::array<TransitionTable<Context>, N>& transitionTables) :
		_topState(topState), _currentState(getDeepestDefaultState(topState))
	{
		// Initialize transitions using static arrays based on MAX_INNER_STATE and
		// MAX_STATE_PER_GROUP
		for(const auto& table: transitionTables)
		{
			for(const auto& trans: table)
			{
				const State<Context>* fromState = trans.getFromState();
				const int fromId = fromState->getId();

				// Populate state transitions
				if(fromId >= 0 && fromId < MAX_INNER_STATE)
				{
					if(_stateTransitionCounts[fromId] < MAX_STATE_PER_GROUP)
					{
						_stateTransitions[fromId][_stateTransitionCounts[fromId]++] = trans;
					}
				}

				// Populate group transitions if applicable
				if(fromState->hasGroup())
				{
					const Group<Context>* group = fromState->getGroup();
					auto& groupTrans = _groupTransitions[group];
					auto& count = _groupTransitionCounts[group];
					if(count < MAX_STATE_PER_GROUP)
					{
						groupTrans[count++] = trans;
					}
				}
			}
		}
	}
	void updateContext(std::optional<Context>& context) { _ctx = context; }
	void updateContext(Event& ev) { _ctx.event = ev; }
	void processEvent(const Event& event, std::optional<Context>& context)
	{

		const State<Context>* state = _currentState;
		for(int i = 0; i < MAX_NESTING && state != nullptr; ++i)
		{
			// Check group transitions first
			if(state->hasGroup())
			{
				const Group<Context>* group = state->getGroup();
				auto git = _groupTransitions.find(group);
				if(git != _groupTransitions.end())
				{
					const auto& transArray = git->second;
					size_t count = _groupTransitionCounts[git->first];
					for(size_t j = 0; j < count; ++j)
					{
						const auto& trans = transArray[j];
						if(trans.getEvent() == event &&
						   evaluateCombinedGuard(trans, state, context))
						{
							performTransition(trans, context);
							return;
						}
					}
				}
			}
			// Check state transitions
			const int stateId = state->getId();
			if(stateId >= 0 && stateId < MAX_INNER_STATE)
			{
				const size_t count = _stateTransitionCounts[stateId];
				for(size_t j = 0; j < count; ++j)
				{
					const auto& trans = _stateTransitions[stateId][j];
					if(trans.getEvent() == event && evaluateCombinedGuard(trans, state, context))
					{
						performTransition(trans, context);
						return;
					}
				}
			}

			state = state->getParent();
		}
		LOG::INFO("FSM", "No transition for event %s in state %s", event.toString(),
				  _currentState->getName());
	}

  private:
	std::optional<Context> _ctx;
	const State<Context>* _topState;
	const State<Context>* _currentState;
	// Static arrays for transitions indexed by state ID
	std::array<std::array<Transition<Context>, MAX_STATE_PER_GROUP>, MAX_INNER_STATE>
		_stateTransitions;
	std::array<size_t, MAX_INNER_STATE> _stateTransitionCounts{};
	// Group transitions using pointers but fixed-size arrays
	std::unordered_map<const Group<Context>*, std::array<Transition<Context>, MAX_STATE_PER_GROUP>>
		_groupTransitions;
	std::unordered_map<const Group<Context>*, size_t> _groupTransitionCounts;

	const State<Context>* getDeepestDefaultState(const StateBase<Context>* state) const
	{
		for(int i = 0; i < MAX_NESTING && state->isComposite(); ++i)
		{
			state = state->getDefaultState();
		}
		return state;
	}

	const State<Context>* findLCA(const StateBase<Context>* s1, const StateBase<Context>* s2) const
	{
		const StateBase<Context>* ancestors[MAX_NESTING];
		size_t s1Depth = 0;
		for(const State<Context>* p = s1; p && s1Depth < MAX_NESTING; p = p->getParent())
		{
			ancestors[s1Depth++] = p;
		}

		for(const State<Context>* p = s2; p; p = p->getParent())
		{
			for(size_t i = 0; i < s1Depth; ++i)
			{
				if(p == ancestors[i])
				{
					return p;
				}
			}
			if(p == _topState)
				break;
		}
		return _topState;
	}

	void performTransition(const Transition<Context>& trans, std::optional<Context>& context)
	{
		const State<Context>* target = trans.getToState();
		const State<Context>* lca = findLCA(_currentState, target);

		// Exit states up to LCA
		const State<Context>* currentExit = _currentState;
		for(int i = 0; i < MAX_NESTING && currentExit != lca; ++i)
		{
			currentExit->onExit(context);
			currentExit = currentExit->getParent();
		}
		// Execute transition action
		trans.getAction().execute(context);
		// Enter states from LCA to target
		const State<Context>* enterPath[MAX_NESTING];
		size_t pathLen = 0;
		for(const State<Context>* s = target; s != lca && pathLen < MAX_NESTING; s = s->getParent())
		{
			enterPath[pathLen++] = s;
		}
		for(int i = pathLen - 1; i >= 0; --i)
		{
			enterPath[i]->onEntry(context);
		}
		_currentState = getDeepestDefaultState(target);
	}

	/** Combine state's guards with transition's guard */
	bool evaluateCombinedGuard(const Transition<Context>& trans, const StateBase<Context>* state,
							   const std::optional<Context>& context) const
	{
		return trans.canTransit(context);
	}
};

} // namespace SM

// template<typename Context>
// using TransitionTable = std::vector<Transition<Context>>;

// template<typename Context, size_t N>
// class FSM
// {
//   public:
// 	/** Constructor taking top state and array of transition tables */
// 	FSM(const StateBase<Context>* topState,
// 		const std::array<TransitionTable<Context>, N>& transitionTables) :
// 		_topState(topState), _currentState(getDeepestDefaultState(topState))
// 	{
// 		// Populate the main transition map and group transition map
// 		for(const auto& table: transitionTables)
// 		{
// 			for(const auto& trans: table)
// 			{
// 				const StateBase<Context>* fromState = trans.getFromState();
// 				transitionMap_[fromState].push_back(trans);
// 				// If the fromState belongs to a group, add to _groupTransitions
// 				if(fromState->getGroup())
// 				{
// 					_groupTransitions[fromState->getGroup()].push_back(trans);
// 				}
// 			}
// 		}
// 		// Copy inner transitions for composite states and groups
// 		copyInnerTransitions(_topState);
// 	}

// 	/** Process an event and perform state transition if applicable */
// 	void processEvent(const Event& event, std::optional<Context>& context)
// 	{
// 		const StateBase<Context>* state = _currentState;
// 		while(state)
// 		{
// 			// Check group transitions first if the state is in a group
// 			if(state->getGroup())
// 			{
// 				auto groupIt = _groupTransitions.find(state->getGroup());
// 				if(groupIt != _groupTransitions.end())
// 				{
// 					for(const auto& trans: groupIt->second)
// 					{
// 						if(trans.getEvent() == event &&
// 						   evaluateCombinedGuard(trans, state, context))
// 						{
// 							performTransition(trans, context);
// 							return;
// 						}
// 					}
// 				}
// 			}
// 			// Check regular transitions
// 			auto it = transitionMap_.find(state);
// 			if(it != transitionMap_.end())
// 			{
// 				for(const auto& trans: it->second)
// 				{
// 					if(trans.getEvent() == event && evaluateCombinedGuard(trans, state, context))
// 					{
// 						performTransition(trans, context);
// 						return;
// 					}
// 				}
// 			}
// 			state = state->getParent();
// 		}
// 		LOG::INFO("FSM", "No transition found for event %s in state %s", event.toString(),
// 				  _currentState->getName());
// 	}

// 	/** Get the current state */
// 	const StateBase<Context>* getCurrentState() const { return _currentState; }

//   private:
// 	const StateBase<Context>* _topState;
// 	const StateBase<Context>* _currentState;
// 	std::unordered_map<const StateBase<Context>*, std::vector<Transition<Context>>> transitionMap_;
// 	std::unordered_map<const Group<Context>*, std::vector<Transition<Context>>> _groupTransitions;

// 	/** Get the deepest default state for initialization */
// 	const StateBase<Context>* getDeepestDefaultState(const StateBase<Context>* state) const
// 	{
// 		const StateBase<Context>* current = state;
// 		while(current->isComposite())
// 		{
// 			current = current->getDefaultState();
// 		}
// 		return current;
// 	}

// 	/** Copy inner transitions from composite states and groups */
// 	void copyInnerTransitions(const StateBase<Context>* state)
// 	{
// 		if(state->isComposite())
// 		{
// 			state->iterateNestedStates([this](StateBase<Context>& nestedState) {
// 				// If nestedState is in a group, its transitions are already in _groupTransitions
// 				if(!nestedState.getGroup())
// 				{
// 					auto it = transitionMap_.find(&nestedState);
// 					if(it != transitionMap_.end())
// 					{
// 						transitionMap_[&nestedState] = it->second; // Copy transitions
// 					}
// 				}
// 				copyInnerTransitions(&nestedState);
// 			});
// 		}
// 	}

// 	/** Combine state's guards with transition's guard */
// 	bool evaluateCombinedGuard(const Transition<Context>& trans, const StateBase<Context>* state,
// 							   const std::optional<Context>& context) const
// 	{
// 		return trans.canTransit(context);
// 	}

// 	/** Find the Least Common Ancestor (LCA) */
// 	const StateBase<Context>* findLCA(const StateBase<Context>* s1,
// 									  const StateBase<Context>* s2) const
// 	{
// 		std::unordered_set<const StateBase<Context>*> ancestors;
// 		const StateBase<Context>* temp = s1;
// 		while(temp)
// 		{
// 			ancestors.insert(temp);
// 			temp = temp->getParent();
// 		}
// 		temp = s2;
// 		while(temp)
// 		{
// 			if(ancestors.find(temp) != ancestors.end())
// 				return temp;
// 			temp = temp->getParent();
// 		}
// 		return _topState; // Default to top state if no LCA found
// 	}

// 	/** Exit states up to the LCA */
// 	void exitUpTo(const StateBase<Context>* from, const StateBase<Context>* ancestor) const
// 	{
// 		const StateBase<Context>* current = from;
// 		while(current != ancestor)
// 		{
// 			current->onExit();
// 			current = current->getParent();
// 		}
// 	}

// 	/** Enter states from LCA to target */
// 	void enterPath(const StateBase<Context>* ancestor, const StateBase<Context>* target) const
// 	{
// 		std::vector<const StateBase<Context>*> path;
// 		const StateBase<Context>* s = target;
// 		while(s != ancestor)
// 		{
// 			path.push_back(s);
// 			s = s->getParent();
// 		}
// 		for(auto it = path.rbegin(); it != path.rend(); ++it)
// 		{
// 			(*it)->onEntry();
// 		}
// 	}

// 	/** Perform the state transition */
// 	void performTransition(const Transition<Context>& trans, std::optional<Context>& context)
// 	{
// 		const StateBase<Context>* target = trans.getToState();
// 		const StateBase<Context>* lca = findLCA(_currentState, target);
// 		exitUpTo(_currentState, lca);
// 		trans.getAction().execute(context);
// 		enterPath(lca, target);
// 		_currentState = getDeepestDefaultState(target);
// 		LOG::INFO("FSM", "Transitioned from %s to %s", _currentState->getName(), target->getName());
// 	}
//};