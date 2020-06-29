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
    Cola = 1
    Pepsi = 2
    Coffee = 3


class ItemColor(enum.Enum):
    Red = 1
    Blue = 2
    White = 3
    Green = 4
    Black = 5
    Yellow = 6


class ProgressState(enum.Enum):
    Init = 0
    Started = 1
    InProgress = 2
    Finished = 3
