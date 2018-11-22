# Python stubs generated by omniidl from leap.idl
# DO NOT EDIT THIS FILE!

import omniORB, _omnipy
from omniORB import CORBA, PortableServer
_0_CORBA = CORBA


_omnipy.checkVersion(4,2, __file__, 1)

try:
    property
except NameError:
    def property(*args):
        return None


#
# Start of module "ssr"
#
__name__ = "ssr"
_0_ssr = omniORB.openModule("ssr", r"leap.idl")
_0_ssr__POA = omniORB.openModule("ssr__POA", r"leap.idl")


# struct Vector
_0_ssr.Vector = omniORB.newEmptyClass()
class Vector (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/Vector:1.0"

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

_0_ssr.Vector = Vector
_0_ssr._d_Vector  = (omniORB.tcInternal.tv_struct, Vector, Vector._NP_RepositoryId, "Vector", "x", omniORB.tcInternal.tv_double, "y", omniORB.tcInternal.tv_double, "z", omniORB.tcInternal.tv_double)
_0_ssr._tc_Vector = omniORB.tcInternal.createTypeCode(_0_ssr._d_Vector)
omniORB.registerType(Vector._NP_RepositoryId, _0_ssr._d_Vector, _0_ssr._tc_Vector)
del Vector

# struct Orientation
_0_ssr.Orientation = omniORB.newEmptyClass()
class Orientation (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/Orientation:1.0"

    def __init__(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

_0_ssr.Orientation = Orientation
_0_ssr._d_Orientation  = (omniORB.tcInternal.tv_struct, Orientation, Orientation._NP_RepositoryId, "Orientation", "pitch", omniORB.tcInternal.tv_double, "roll", omniORB.tcInternal.tv_double, "yaw", omniORB.tcInternal.tv_double)
_0_ssr._tc_Orientation = omniORB.tcInternal.createTypeCode(_0_ssr._d_Orientation)
omniORB.registerType(Orientation._NP_RepositoryId, _0_ssr._d_Orientation, _0_ssr._tc_Orientation)
del Orientation

# typedef ... Zone
class Zone:
    _NP_RepositoryId = "IDL:ssr/Zone:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_ssr.Zone = Zone
_0_ssr._d_Zone  = omniORB.tcInternal.tv_long
_0_ssr._ad_Zone = (omniORB.tcInternal.tv_alias, Zone._NP_RepositoryId, "Zone", omniORB.tcInternal.tv_long)
_0_ssr._tc_Zone = omniORB.tcInternal.createTypeCode(_0_ssr._ad_Zone)
omniORB.registerType(Zone._NP_RepositoryId, _0_ssr._ad_Zone, _0_ssr._tc_Zone)
del Zone
_0_ssr.ZONE_NONE = 0
_0_ssr.ZONE_HOVERING = 1
_0_ssr.ZONE_TOUCHING = 2

# struct Finger
_0_ssr.Finger = omniORB.newEmptyClass()
class Finger (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/Finger:1.0"

    def __init__(self, tipPosition, touchZone):
        self.tipPosition = tipPosition
        self.touchZone = touchZone

_0_ssr.Finger = Finger
_0_ssr._d_Finger  = (omniORB.tcInternal.tv_struct, Finger, Finger._NP_RepositoryId, "Finger", "tipPosition", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "touchZone", omniORB.typeMapping["IDL:ssr/Zone:1.0"])
_0_ssr._tc_Finger = omniORB.tcInternal.createTypeCode(_0_ssr._d_Finger)
omniORB.registerType(Finger._NP_RepositoryId, _0_ssr._d_Finger, _0_ssr._tc_Finger)
del Finger

# struct Hand
_0_ssr.Hand = omniORB.newEmptyClass()
class Hand (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/Hand:1.0"

    def __init__(self, palmPosition, palmNormal, palmDirection, sphereCenter, sphereRadius, palmOrientation, fingers):
        self.palmPosition = palmPosition
        self.palmNormal = palmNormal
        self.palmDirection = palmDirection
        self.sphereCenter = sphereCenter
        self.sphereRadius = sphereRadius
        self.palmOrientation = palmOrientation
        self.fingers = fingers

_0_ssr.Hand = Hand
_0_ssr._d_Hand  = (omniORB.tcInternal.tv_struct, Hand, Hand._NP_RepositoryId, "Hand", "palmPosition", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "palmNormal", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "palmDirection", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "sphereCenter", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "sphereRadius", omniORB.tcInternal.tv_double, "palmOrientation", omniORB.typeMapping["IDL:ssr/Orientation:1.0"], "fingers", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:ssr/Finger:1.0"], 0))
_0_ssr._tc_Hand = omniORB.tcInternal.createTypeCode(_0_ssr._d_Hand)
omniORB.registerType(Hand._NP_RepositoryId, _0_ssr._d_Hand, _0_ssr._tc_Hand)
del Hand

# typedef ... GestureState
class GestureState:
    _NP_RepositoryId = "IDL:ssr/GestureState:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_ssr.GestureState = GestureState
_0_ssr._d_GestureState  = omniORB.tcInternal.tv_long
_0_ssr._ad_GestureState = (omniORB.tcInternal.tv_alias, GestureState._NP_RepositoryId, "GestureState", omniORB.tcInternal.tv_long)
_0_ssr._tc_GestureState = omniORB.tcInternal.createTypeCode(_0_ssr._ad_GestureState)
omniORB.registerType(GestureState._NP_RepositoryId, _0_ssr._ad_GestureState, _0_ssr._tc_GestureState)
del GestureState
_0_ssr.STATE_INVALID = -1
_0_ssr.STATE_START = 1
_0_ssr.STATE_UPDATE = 2
_0_ssr.STATE_STOP = 3

# struct CircleGesture
_0_ssr.CircleGesture = omniORB.newEmptyClass()
class CircleGesture (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/CircleGesture:1.0"

    def __init__(self, state, progress, radius, center, normal):
        self.state = state
        self.progress = progress
        self.radius = radius
        self.center = center
        self.normal = normal

_0_ssr.CircleGesture = CircleGesture
_0_ssr._d_CircleGesture  = (omniORB.tcInternal.tv_struct, CircleGesture, CircleGesture._NP_RepositoryId, "CircleGesture", "state", omniORB.typeMapping["IDL:ssr/GestureState:1.0"], "progress", omniORB.tcInternal.tv_double, "radius", omniORB.tcInternal.tv_double, "center", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "normal", omniORB.typeMapping["IDL:ssr/Vector:1.0"])
_0_ssr._tc_CircleGesture = omniORB.tcInternal.createTypeCode(_0_ssr._d_CircleGesture)
omniORB.registerType(CircleGesture._NP_RepositoryId, _0_ssr._d_CircleGesture, _0_ssr._tc_CircleGesture)
del CircleGesture

# struct SwipeGesture
_0_ssr.SwipeGesture = omniORB.newEmptyClass()
class SwipeGesture (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/SwipeGesture:1.0"

    def __init__(self, state, startPosition, position, direction, speed):
        self.state = state
        self.startPosition = startPosition
        self.position = position
        self.direction = direction
        self.speed = speed

_0_ssr.SwipeGesture = SwipeGesture
_0_ssr._d_SwipeGesture  = (omniORB.tcInternal.tv_struct, SwipeGesture, SwipeGesture._NP_RepositoryId, "SwipeGesture", "state", omniORB.typeMapping["IDL:ssr/GestureState:1.0"], "startPosition", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "position", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "direction", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "speed", omniORB.tcInternal.tv_double)
_0_ssr._tc_SwipeGesture = omniORB.tcInternal.createTypeCode(_0_ssr._d_SwipeGesture)
omniORB.registerType(SwipeGesture._NP_RepositoryId, _0_ssr._d_SwipeGesture, _0_ssr._tc_SwipeGesture)
del SwipeGesture

# struct KeyTapGesture
_0_ssr.KeyTapGesture = omniORB.newEmptyClass()
class KeyTapGesture (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/KeyTapGesture:1.0"

    def __init__(self, state, direction, position):
        self.state = state
        self.direction = direction
        self.position = position

_0_ssr.KeyTapGesture = KeyTapGesture
_0_ssr._d_KeyTapGesture  = (omniORB.tcInternal.tv_struct, KeyTapGesture, KeyTapGesture._NP_RepositoryId, "KeyTapGesture", "state", omniORB.typeMapping["IDL:ssr/GestureState:1.0"], "direction", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "position", omniORB.typeMapping["IDL:ssr/Vector:1.0"])
_0_ssr._tc_KeyTapGesture = omniORB.tcInternal.createTypeCode(_0_ssr._d_KeyTapGesture)
omniORB.registerType(KeyTapGesture._NP_RepositoryId, _0_ssr._d_KeyTapGesture, _0_ssr._tc_KeyTapGesture)
del KeyTapGesture

# struct ScreenTapGesture
_0_ssr.ScreenTapGesture = omniORB.newEmptyClass()
class ScreenTapGesture (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/ScreenTapGesture:1.0"

    def __init__(self, state, direction, position):
        self.state = state
        self.direction = direction
        self.position = position

_0_ssr.ScreenTapGesture = ScreenTapGesture
_0_ssr._d_ScreenTapGesture  = (omniORB.tcInternal.tv_struct, ScreenTapGesture, ScreenTapGesture._NP_RepositoryId, "ScreenTapGesture", "state", omniORB.typeMapping["IDL:ssr/GestureState:1.0"], "direction", omniORB.typeMapping["IDL:ssr/Vector:1.0"], "position", omniORB.typeMapping["IDL:ssr/Vector:1.0"])
_0_ssr._tc_ScreenTapGesture = omniORB.tcInternal.createTypeCode(_0_ssr._d_ScreenTapGesture)
omniORB.registerType(ScreenTapGesture._NP_RepositoryId, _0_ssr._d_ScreenTapGesture, _0_ssr._tc_ScreenTapGesture)
del ScreenTapGesture

# typedef ... GestureType
class GestureType:
    _NP_RepositoryId = "IDL:ssr/GestureType:1.0"
    def __init__(self, *args, **kw):
        raise RuntimeError("Cannot construct objects of this type.")
_0_ssr.GestureType = GestureType
_0_ssr._d_GestureType  = omniORB.tcInternal.tv_long
_0_ssr._ad_GestureType = (omniORB.tcInternal.tv_alias, GestureType._NP_RepositoryId, "GestureType", omniORB.tcInternal.tv_long)
_0_ssr._tc_GestureType = omniORB.tcInternal.createTypeCode(_0_ssr._ad_GestureType)
omniORB.registerType(GestureType._NP_RepositoryId, _0_ssr._ad_GestureType, _0_ssr._tc_GestureType)
del GestureType
_0_ssr.TYPE_INVALID = -1
_0_ssr.TYPE_SWIPE = 1
_0_ssr.TYPE_CIRCLE = 4
_0_ssr.TYPE_SCREEN_TAP = 5
_0_ssr.TYPE_KEY_TAP = 6

# struct GestureFrame
_0_ssr.GestureFrame = omniORB.newEmptyClass()
class GestureFrame (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/GestureFrame:1.0"

    def __init__(self, id, type, circle, swipe, key, screen):
        self.id = id
        self.type = type
        self.circle = circle
        self.swipe = swipe
        self.key = key
        self.screen = screen

_0_ssr.GestureFrame = GestureFrame
_0_ssr._d_GestureFrame  = (omniORB.tcInternal.tv_struct, GestureFrame, GestureFrame._NP_RepositoryId, "GestureFrame", "id", omniORB.tcInternal.tv_long, "type", omniORB.typeMapping["IDL:ssr/GestureType:1.0"], "circle", omniORB.typeMapping["IDL:ssr/CircleGesture:1.0"], "swipe", omniORB.typeMapping["IDL:ssr/SwipeGesture:1.0"], "key", omniORB.typeMapping["IDL:ssr/KeyTapGesture:1.0"], "screen", omniORB.typeMapping["IDL:ssr/ScreenTapGesture:1.0"])
_0_ssr._tc_GestureFrame = omniORB.tcInternal.createTypeCode(_0_ssr._d_GestureFrame)
omniORB.registerType(GestureFrame._NP_RepositoryId, _0_ssr._d_GestureFrame, _0_ssr._tc_GestureFrame)
del GestureFrame

# struct Frame
_0_ssr.Frame = omniORB.newEmptyClass()
class Frame (omniORB.StructBase):
    _NP_RepositoryId = "IDL:ssr/Frame:1.0"

    def __init__(self, id, timestamp, hands, gestures):
        self.id = id
        self.timestamp = timestamp
        self.hands = hands
        self.gestures = gestures

_0_ssr.Frame = Frame
_0_ssr._d_Frame  = (omniORB.tcInternal.tv_struct, Frame, Frame._NP_RepositoryId, "Frame", "id", omniORB.tcInternal.tv_long, "timestamp", omniORB.tcInternal.tv_longlong, "hands", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:ssr/Hand:1.0"], 0), "gestures", (omniORB.tcInternal.tv_sequence, omniORB.typeMapping["IDL:ssr/GestureFrame:1.0"], 0))
_0_ssr._tc_Frame = omniORB.tcInternal.createTypeCode(_0_ssr._d_Frame)
omniORB.registerType(Frame._NP_RepositoryId, _0_ssr._d_Frame, _0_ssr._tc_Frame)
del Frame

#
# End of module "ssr"
#
__name__ = "leap_idl"

_exported_modules = ( "ssr", )

# The end.
