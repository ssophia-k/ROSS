from typing import TypedDict
from numpy.typing import NDArray
import numpy as np

class _MessageContent(TypedDict):
    id: int # either the id of the neighboring robot or the voronoi seed point
    vector: NDArray[np.float64] # vector from the sensed robot/point to the sender of this message

class NeighborMessage:
    def __init__(self, sender_id: int, content: _MessageContent) -> None:
        self.sender_id = sender_id
        self.content = content

class VoronoiPointMessage:
    def __init__(self, sender_id: int, content: _MessageContent) -> None:
        self.sender_id = sender_id
        self.content = content


# the below code is weird and i dont get why it is quite necessary / the improvement
'''
from enum           import Enum
from typing         import TypedDict, Union, Literal
from numpy.typing   import NDArray
import numpy as np

class MessageKind(Enum):
    NEIGHBOR = "neighbor"
    VORONOI  = "voronoi"

class NeighborContent(TypedDict):
    kind:        Literal[MessageKind.NEIGHBOR]
    neighbor_id: int
    vector:      NDArray[np.float64]

class VoronoiContent(TypedDict):
    kind:      Literal[MessageKind.VORONOI]
    point_id:  int
    vector:    NDArray[np.float64]

AnyContent = Union[NeighborContent, VoronoiContent]

class Message:
    def __init__(self, sender_id: int, content: AnyContent) -> None:
        self.sender_id = sender_id
        self.content   = content
'''


