# Parent class for the different states
class State:
    def __init__(self, state_name, state_priority):
        self.name = state_name
        self.priority = state_priority

    # 'Abstract method' 
    def execute_state(self, robot): pass
