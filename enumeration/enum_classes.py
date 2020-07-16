import enum


class State(enum.Enum):
    Load_env = 1
    Edit_env_obj = 2
    Get_order = 3
    Get_komo = 4
    Fetch_item = 5
    Reach_user = 6
    Place_item = 7


class Items(enum.Enum):
    Coffee = 1
    Sprite = 2
    Juice = 3
    Invalid = 7


class ItemColor(enum.Enum):
    Red = 1
    Green = 2
    Yellow = 3
    White = 4
    Blue = 5
    Black = 6
    Invalid = 7


class ProgressState(enum.Enum):
    Init = 0
    Started = 1
    InProgress = 2
    Finished = 3
