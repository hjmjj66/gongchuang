"""Communication pauses preserve tasks only within the same board session."""


def communication_lost(board):
    return board is not None and board.fault_reason in (
        'serial disconnected', 'STATUS missing or timed out', 'ODOM missing or timed out')
