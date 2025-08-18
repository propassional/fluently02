from transitions import Machine

class PlayerStateMachine:
    states = ["idle", "move", "attack"]

    def __init__(self):
        self.machine = Machine(model=self, states=self.states, initial="idle")
        self.machine.add_transition("player_move", "idle", "move")
        self.machine.add_transition("player_attack", "move", "attack")

class EnemyStateMachine:
    states = ["idle", "chase", "attack"]

    def __init__(self):
        self.machine = Machine(model=self, states=self.states, initial="idle")
        self.machine.add_transition("enemy_chase", "idle", "chase")
        self.machine.add_transition("enemy_attack", "chase", "attack")

# Create instances of both state machines
player = PlayerStateMachine()
enemy = EnemyStateMachine()

# Shared event: Player moves
player.player_move()
print(f"Player state: {player.state}, Enemy state: {enemy.state}")

# Shared event: Enemy chases
enemy.enemy_chase()
print(f"Player state: {player.state}, Enemy state: {enemy.state}")
