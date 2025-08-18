from statemachine import State, StateMachine

class TrafficLightMachine(StateMachine):
    green = State(initial=True)
    yellow = State()
    red = State()

    slowdown = green.to(yellow)
    stop = yellow.to(red)
    go = red.to(green)

    cycle = slowdown | stop | go

    def before_slowdown(self):
        print("Slowdown")

    def before_cycle(self, event: str, source: State, target: State, message: str = ""):
        message = ". " + message if message else ""
        return f"Running {event} from {source.id} to {target.id}{message}"

    def on_enter_red(self):
        print("Don't move.")

    def on_exit_red(self):
        print("Go ahead!")

# Run a transition
sm = TrafficLightMachine()
sm.send("cycle")
