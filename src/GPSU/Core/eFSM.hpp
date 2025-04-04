#pragma once
#include <optional>
#include <tuple>
#include <type_traits>
#include <variant>
#include "Logger.hpp"

namespace SM
{

constexpr int MAX_INNER_STATE = 10;
static constexpr size_t MAX_EVENT_TYPE = 4;
static constexpr size_t MAX_LISTENER_PER_TYPE = 12;
class FSM;

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
	constexpr StateFunction(int id) : _id(id) {}
	constexpr virtual ~StateFunction() = default;
	constexpr int getId() const { return _id; }

  private:
	const int _id;
};

template<typename Context>
struct Guard : public StateFunction
{
  public:
	using FuncPtr = bool (*)(const std::optional<Context>&);
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
static_assert(std::is_trivially_destructible_v<ValidatedContext>,
			  "ValidatedContext must be trivially destructible");
inline constexpr Guard<ValidatedContext> NullGuard(-1, nullptr);

template<typename Context>
struct Action : public StateFunction
{
  public:
	using FuncPtr = void (*)(std::optional<Context>&);
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

inline constexpr Action<ValidatedContext> NullAction(-1, nullptr);
template<typename Context> // NoLint
struct StateBase
{
	using EventHandler = void (*)(const Event&);

  public:
	constexpr StateBase(const int id, const char* name,
						const Action<Context>& entryAction = NullAction,
						const Action<Context>& exitAction = NullAction,
						const Guard<Context>& entryGuard = NullGuard,
						const Guard<Context>& exitGuard = NullGuard) :
		_id(id), _name(name), _entryAction(entryAction), _exitAction(exitAction),
		_entryActionGuard(entryGuard), _exitActionGuard(exitGuard), _listenerCount{}, _listeners{}
	{
	}

	static_assert(detail::has_event_member<Context>::value, "Context must have an 'event' member");
	static_assert(detail::event_member_is_correct_type<Context>::value,
				  "Context's 'event' member must be of type SM::Event");

	virtual bool isComposite() const { return false; }
	virtual void handleEvent(const Event& ev) const;

	constexpr int getId() const { return _id; }
	constexpr const char* getName() const { return _name; }

	constexpr bool canActOnEntry() const { return _entryActionGuard.evaluate(_ctx); }
	constexpr bool canActOnExit() const { return _exitActionGuard.evaluate(_ctx); }
	constexpr void onEntry() const
	{
		if(canActOnEntry())
		{
			_entryAction.execute(_ctx);
		}
	}
	constexpr void onExit() const
	{
		if(canActOnExit())
		{
			_exitAction.execute(_ctx);
		}
	}

	void dispatchEvent(const Event& event)
	{
		_ctx.event = event;
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
	Context _ctx;
	Action<Context> _entryAction;
	Action<Context> _exitAction;
	Guard<Context> _entryActionGuard;
	Guard<Context> _exitActionGuard;
	size_t _listenerCount[MAX_EVENT_TYPE];
	EventHandler _listeners[MAX_EVENT_TYPE][MAX_LISTENER_PER_TYPE];
};

struct DefaultInnerState : public StateBase<ValidatedContext>
{
  public:
	constexpr DefaultInnerState() : StateBase(0, "DefaultInnerState", NullAction, NullAction) {}
	void iterateNestedStates(std::function<void(StateBase&)> func) override {}
};

template<typename Context, typename... InnerElements>
struct State;
template<typename Context, typename... States>
struct Group;
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
		BaseState(id, name, NullAction, NullAction, NullGuard, NullGuard),
		elements_(std::make_tuple(elements...))
	{
	}

	template<typename =
				 std::enable_if_t<((IsState_v<InnerElements> || IsGroup_v<InnerElements>) && ...)>>
	constexpr State(int id, const Actions& entryAction, const Actions& exitAction, const char* name,
					InnerElements... elements) :
		BaseState(id, name, entryAction, exitAction, NullGuard, NullGuard),
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
	static constexpr DefaultInnerState defaultState{};

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
using OptState = StateBase<Context>*;

// Update Transition to use pointers
template<typename Context>
struct Transition
{
	using BaseState = StateBase<Context>;
	using Actions = Action<Context>;
	using Guards = Guard<Context>;

  public:
	const int _id;
	constexpr Transition(int id, const BaseState* from, const BaseState* to, const Guards& guard,
						 const Event& event = NullEvent,
						 const Actions& onTransitionAction = NullAction) :
		_id(id), _fromState(from), _toState(to), _guard(guard), _event(event),
		_onTransitionAction(onTransitionAction)
	{
	}

	bool canTransit(const std::optional<Context>& context) const
	{
		return _guard.evaluate(context);
	}

	OptState<Context> executeTransition(std::optional<Context>& context) const
	{
		if(canTransit(context))
		{
			_onTransitionAction.execute(context);
			return _toState;
		}
		return nullptr; // No transition
	}

	const BaseState* getFromState() const { return _fromState; }
	const Event& getEvent() const { return _event; }
	const BaseState* getToState() const { return _toState; }
	const Actions& getAction() const { return _onTransitionAction; }
	const Guards& getGuard() const { return _guard; }

  private:
	const BaseState* _fromState;
	const BaseState* _toState;
	const Guards& _guard;
	const Event& _event;
	const Actions& _onTransitionAction;
};
} // namespace SM