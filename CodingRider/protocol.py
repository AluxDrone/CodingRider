import os
import abc 
import numpy as np
from struct import *
from enum import Enum

from CodingRider.system import *


# ISerializable Start


class ISerializable:
    
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def getSize(self):
        pass

    @abc.abstractmethod
    def ToArray(self):
        pass


# ISerializable End



# DataType Start


class DataType(Enum):
    
    None_						= 0x00		# 없음
    Ping						= 0x01		# 통신 확인
    Ack							= 0x02		# 데이터 수신에 대한 응답
    Error						= 0x03		# 오류
    Request						= 0x04		# 지정한 타입의 데이터 요청
    Information					= 0x07		# 펌웨어 및 장치 정보
    Control						= 0x10		# 조종
    
    Command						= 0x11		# 명령
    Pairing						= 0x12		# 페어링
    ResponseRate				= 0x13		# ResponseRate
    
    # Light
    LightManual					= 0x20		# LED 수동 제어
    LightMode					= 0x21		# LED 모드 지정
    LightEvent					= 0x22		# LED 이벤트
    
    # 센서 RAW 데이터
    RawMotion					= 0x30		# Motion 센서 데이터 RAW 값
    
    # 상태,  센서
    State						= 0x40		# 드론의 상태(비행 모드, 방위기준, 배터리량)
    Altitude					= 0x43		# 높이, 고도
    Motion						= 0x44		# Motion 센서 데이터 처리한 값(IMU)
    VisionSensor				= 0x47		# Vision Sensor X, Y, Z
    
    # 설정
    Count						= 0x50		# 카운트
    Bias						= 0x51		# 엑셀, 자이로 바이어스 값
    Trim						= 0x52		# 트림
    LostConnection				= 0x54		# 연결이 끊긴 후 반응 시간 설정
    
    # Device
    Motor						= 0x60		# 모터 제어 및 현재 제어값 확인
    Buzzer						= 0x62		# 버저 제어
    Battery						= 0x64		# 배터리
    
    # Input
    Button						= 0x70		# 버튼
    Joystick					= 0x71		# 조이스틱
    
    # Information Assembled
    InformationAssembledForController			= 0xA0		# 데이터 모음
    
    EndOfType                   = 0xDC


# DataType End



# CommandType Start


class CommandType(Enum):
    
    None_						= 0x00		# 이벤트 없음
    
    Stop						= 0x01		# 정지
    
    ModeControlFlight			= 0x02		# 비행 제어 모드 설정
    Headless					= 0x03		# 이스케이프 모드 설정
    ControlSpeed				= 0x04		# 제어 속도 설정
    
    ClearBias					= 0x05		# 자이로/엑셀 바이어스 리셋(트림도 같이 초기화 됨)
    ClearTrim					= 0x06		# 트림 초기화
    
    FlightEvent					= 0x07		# 비행 이벤트 실행
    
    SetDefault					= 0x08		# 기본 설정으로 초기화
    ModeController				= 0x0A		# 조종기 동작 모드(0x10:조종, 0x80:링크)
    Link						= 0x0B		# 링크 제어(0:Client Mode, 1:Server Mode, 2:Pairing Start)
    LoadDefaultColor			= 0x0C		# 기본 색상으로 변경
    
    Trim						= 0x0D		# 트림
    SetSwarm				    = 0x0F		# 군집 고도제어 설정
    
    ModeTest					= 0xF0		# 테스트 락(테스트를 완료하기 전까진 사용 불가 / 27:활성화, 11:해제))
    
    EndOfType                   = 0xEC


# CommandType End



# Header Start


class Header(ISerializable):

    def __init__(self):
        self.dataType    = DataType.None_
        self.length      = 0
        self.from_       = DeviceType.None_
        self.to_         = DeviceType.None_


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<BBBB', self.dataType.value, self.length, self.from_.value, self.to_.value)


    @classmethod
    def parse(cls, dataArray):
        header = Header()

        if len(dataArray) != cls.getSize():
            return None

        header.dataType, header.length, header.from_, header.to_ = unpack('<BBBB', dataArray)

        header.dataType = DataType(header.dataType)
        header.from_ = DeviceType(header.from_)
        header.to_ = DeviceType(header.to_)

        return header


# Header End



# Common Start


class Ping(ISerializable):

    def __init__(self):
        self.systemTime     = 0


    @classmethod
    def getSize(cls):
        return 8


    def toArray(self):
        return pack('<Q', self.systemTime)


    @classmethod
    def parse(cls, dataArray):
        data = Ping()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.systemTime, = unpack('<Q', dataArray)
        return data



class Ack(ISerializable):

    def __init__(self):
        self.systemTime     = 0
        self.dataType       = DataType.None_
        self.crc16          = 0


    @classmethod
    def getSize(cls):
        return 11


    def toArray(self):
        return pack('<QBH', self.systemTime, self.dataType.value, self.crc16)


    @classmethod
    def parse(cls, dataArray):
        data = Ack()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.systemTime, data.dataType, data.crc16 = unpack('<QBH', dataArray)
        data.dataType = DataType(data.dataType)

        return data



class Error(ISerializable):

    def __init__(self):
        self.systemTime             = 0
        self.errorFlagsForSensor    = 0
        self.errorFlagsForState     = 0


    @classmethod
    def getSize(cls):
        return 16


    def toArray(self):
        return pack('<QII', self.systemTime, self.errorFlagsForSensor, self.errorFlagsForState)


    @classmethod
    def parse(cls, dataArray):
        data = Error()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.systemTime, data.errorFlagsForSensor, data.errorFlagsForState = unpack('<QII', dataArray)

        return data



class Request(ISerializable):

    def __init__(self):
        self.dataType    = DataType.None_


    @classmethod
    def getSize(cls):
        return 1


    def toArray(self):
        return pack('<B', self.dataType.value)


    @classmethod
    def parse(cls, dataArray):
        data = Request()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.dataType, = unpack('<B', dataArray)
        data.dataType = DataType(data.dataType)

        return data



class RequestOption(ISerializable):

    def __init__(self):
        self.dataType   = DataType.None_
        self.option     = 0


    @classmethod
    def getSize(cls):
        return 5


    def toArray(self):
        return pack('<BI', self.dataType.value, self.option)


    @classmethod
    def parse(cls, dataArray):
        data = RequestOption()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.dataType, data.option = unpack('<BI', dataArray)
        data.dataType = DataType(data.dataType)

        return data



class Message():

    def __init__(self):
        self.message    = ""


    def getSize(self):
        return len(self.message)


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.message.encode('ascii', 'ignore'))
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = Message()
        
        if len(dataArray) == 0:
            return ""

        data.message = dataArray[0:len(dataArray)].decode()
        
        return data



class SystemInformation(ISerializable):

    def __init__(self):
        self.crc32bootloader    = 0
        self.crc32application   = 0


    @classmethod
    def getSize(cls):
        return 8


    def toArray(self):
        return pack('<II', self.crc32bootloader, self.crc32application)


    @classmethod
    def parse(cls, dataArray):
        data = SystemInformation()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.crc32bootloader, data.crc32application = unpack('<II', dataArray)

        return data



class Version(ISerializable):

    def __init__(self):
        self.build          = 0
        self.minor          = 0
        self.major          = 0

        self.v              = 0         # build, minor, major을 하나의 UInt32로 묶은 것(버젼 비교 시 사용)


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<HBB', self.build, self.minor, self.major)


    @classmethod
    def parse(cls, dataArray):
        data = Version()
        
        if len(dataArray) != cls.getSize():
            return None

        data.v, = unpack('<I', dataArray)

        data.build, data.minor, data.major = unpack('<HBB', dataArray)

        return data



class Information(ISerializable):

    def __init__(self):
        self.modeUpdate     = ModeUpdate.None_

        self.modelNumber    = ModelNumber.None_
        self.version        = Version()

        self.year           = 0
        self.month          = 0
        self.day            = 0


    @classmethod
    def getSize(cls):
        return 13


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<B', self.modeUpdate.value))
        dataArray.extend(pack('<I', self.modelNumber.value))
        dataArray.extend(self.version.toArray())
        dataArray.extend(pack('<H', self.year))
        dataArray.extend(pack('<B', self.month))
        dataArray.extend(pack('<B', self.day))
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = Information()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd = 1;                    data.modeUpdate,    = unpack('<B', dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 4;                   data.modelNumber,   = unpack('<I', dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += Version.getSize();   data.version        = Version.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 2;                   data.year,          = unpack('<H', dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 1;                   data.month,         = unpack('<B', dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 1;                   data.day,           = unpack('<B', dataArray[indexStart:indexEnd])

        data.modeUpdate     = ModeUpdate(data.modeUpdate)
        data.modelNumber    = ModelNumber(data.modelNumber)

        return data



class UpdateLocation(ISerializable):

    def __init__(self):
        self.indexBlockNext    = 0


    @classmethod
    def getSize(cls):
        return 2


    def toArray(self):
        return pack('<H', self.indexBlockNext)


    @classmethod
    def parse(cls, dataArray):
        data = UpdateLocation()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.indexBlockNext, = unpack('<H', dataArray)

        return data



class Address(ISerializable):

    def __init__(self):
        self.address    = bytearray()


    @classmethod
    def getSize(cls):
        return 16


    def toArray(self):
        return self.address


    @classmethod
    def parse(cls, dataArray):
        data = Address()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.address = dataArray[0:16]
        return data



class Pairing(ISerializable):

    def __init__(self):
        self.address0       = 0
        self.address1       = 0
        self.address2       = 0
        self.address3       = 0
        self.address4       = 0
        self.channel0       = 0


    @classmethod
    def getSize(cls):
        return 11


    def toArray(self):
        return pack('<BBBBBB', self.address0, self.address1, self.address2, self.address3, self.address4, self.channel0)


    @classmethod
    def parse(cls, dataArray):
        data = Pairing()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.address0, data.address1, data.address2, data.address3, data.address4, data.channel0 = unpack('<BBBBBB', dataArray)
        return data


class ResponseRate(ISerializable) :
    
    def __init__(self):
        self.responseRate = 0
        
    @classmethod
    def getSize(cls):
        return 1


    def toArray(self):
        return pack('<B', self.responseRate)


    @classmethod
    def parse(cls, dataArray):
        data = ResponseRate()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.responseRate, = unpack('<B', dataArray)

        return data



class Rssi(ISerializable):

    def __init__(self):
        self.rssi       = 0


    @classmethod
    def getSize(cls):
        return 1


    def toArray(self):
        return pack('<b', self.rssi)


    @classmethod
    def parse(cls, dataArray):
        data = Rssi()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.rssi, = unpack('<b', dataArray)

        return data



class Command(ISerializable):

    def __init__(self):
        self.commandType    = CommandType.None_
        self.option         = ModeControlFlight.None_


    @classmethod
    def getSize(cls):
        return 2


    def toArray(self):
        return pack('<BB', self.commandType.value, self.option)


    @classmethod
    def parse(cls, dataArray):
        data = Command()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.commandType, data.option = unpack('<BB', dataArray)
        data.commandType = CommandType(data.commandType)

        return data



class CommandLightEvent(ISerializable):

    def __init__(self):
        self.command    = Command()
        self.event      = LightEvent()


    @classmethod
    def getSize(cls):
        return Command.getSize() + LightEvent.getSize()


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.command.toArray())
        dataArray.extend(self.event.toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = CommandLightEvent()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd = Command.getSize();         data.command    = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += LightEvent.getSize();     data.event      = LightEvent.parse(dataArray[indexStart:indexEnd])
        
        return data
        


class CommandLightEventColor(ISerializable):

    def __init__(self):
        self.command    = Command()
        self.event      = LightEvent()
        self.color      = Color()


    @classmethod
    def getSize(cls):
        return Command.getSize() + LightEvent.getSize() + Color.getSize()


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.command.toArray())
        dataArray.extend(self.event.toArray())
        dataArray.extend(self.color.toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = CommandLightEventColor()
        
        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0;        indexEnd = Command.getSize();       data.command    = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += LightEvent.getSize();   data.event      = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += Color.getSize();        data.color      = Color.parse(dataArray[indexStart:indexEnd])
        
        return data
        


class CommandLightEventColors(ISerializable):

    def __init__(self):
        self.command    = Command()
        self.event      = LightEvent()
        self.colors     = Colors.Black


    @classmethod
    def getSize(cls):
        return Command.getSize() + LightEvent.getSize() + 1


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.command.toArray())
        dataArray.extend(self.event.toArray())
        dataArray.extend(pack('<B', self.colors.value))
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = CommandLightEventColors()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd = Command.getSize();       data.command    = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += LightEvent.getSize();   data.event      = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 1;                      data.colors    = unpack('<B', dataArray[indexStart:indexEnd])

        data.colors     = Colors(data.colors)

        return data


# Common End


# Control Start


class ControlQuad8(ISerializable):

    def __init__(self):
        self.roll       = 0
        self.pitch      = 0
        self.yaw        = 0
        self.throttle   = 0


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<bbbb', self.roll, self.pitch, self.yaw, self.throttle)


    @classmethod
    def parse(cls, dataArray):
        data = ControlQuad8()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.roll, data.pitch, data.yaw, data.throttle = unpack('<bbbb', dataArray)
        return data



class ControlQuad8AndRequestData(ISerializable):

    def __init__(self):
        self.roll       = 0
        self.pitch      = 0
        self.yaw        = 0
        self.throttle   = 0
        self.dataType   = DataType.None_


    @classmethod
    def getSize(cls):
        return 5


    def toArray(self):
        return pack('<bbbbb', self.roll, self.pitch, self.yaw, self.throttle, self.dataType)


    @classmethod
    def parse(cls, dataArray):
        data = ControlQuad8AndRequestData()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.roll, data.pitch, data.yaw, data.throttle, data.dataType = unpack('<bbbbb', dataArray)
        
        data.dataType = DataType(data.dataType)
        
        return data



class ControlPosition16(ISerializable):

    def __init__(self):
        self.positionX          = 0
        self.positionY          = 0
        self.positionZ          = 0

        self.velocity           = 0
        
        self.heading            = 0
        self.rotationalVelocity = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<hhhhhh', self.positionX, self.positionY, self.positionZ, self.velocity, self.heading, self.rotationalVelocity)


    @classmethod
    def parse(cls, dataArray):
        data = ControlPosition16()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.positionX, data.positionY, data.positionZ, data.velocity, data.heading, data.rotationalVelocity = unpack('<hhhhhh', dataArray)
        return data



class ControlPosition(ISerializable):

    def __init__(self):
        self.positionX          = 0
        self.positionY          = 0
        self.positionZ          = 0

        self.velocity           = 0

        self.heading            = 0
        self.rotationalVelocity = 0


    @classmethod
    def getSize(cls):
        return 20


    def toArray(self):
        return pack('<ffffhh', self.positionX, self.positionY, self.positionZ, self.velocity, self.heading, self.rotationalVelocity)


    @classmethod
    def parse(cls, dataArray):
        data = ControlPosition()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.positionX, data.positionY, data.positionZ, data.velocity, data.heading, data.rotationalVelocity = unpack('<ffffhh', dataArray)
        return data


# Control End



# Light Start


class LightModeDrone(Enum):
    
    None_                   = 0x00
    
    # Team (Forward, Rear를 동시에 제어)
    TeamRgbNone				= 0x10
    TeamRgbManual			= 0x11		# 수동 제어
    TeamRgbHold				= 0x12		# 지정한 색상을 계속 켬
    TeamRgbFlicker			= 0x13		# 깜빡임			
    TeamRgbFlickerDouble	= 0x14		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    TeamRgbDimming			= 0x15		# 밝기 제어하여 천천히 깜빡임
    TeamRgbSunrise			= 0x16		# 꺼진 상태에서 점점 밝아짐
    TeamRgbSunset			= 0x17		# 켜진 상태에서 점점 어두워짐
    TeamRgbRainbow			= 0x18		# 무지개색
    TeamRgbRainbow2			= 0x19		# 무지개색
    TeamRgbRedBlue			= 0x1A		# Red-Blue
    
    TeamFlowForward			= 0x1E		# 앞으로 흐름 
    TeamWarning				= 0x1F		# 경고
    
    # Body
    BodyNone				= 0x20		
    BodyManual				= 0x21		# 수동 제어
    BodyHold				= 0x22		# 지정한 색상을 계속 켬
    BodyFlicker				= 0x23		# 깜빡임			
    BodyFlickerDouble		= 0x24		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    BodyDimming				= 0x25		# 밝기 제어하여 천천히 깜빡임
    BodySunrise				= 0x26		# 꺼진 상태에서 점점 밝아짐
    BodySunset				= 0x27		# 켜진 상태에서 점점 어두워짐
    BodyRainbow				= 0x28		# 무지개색
    BodyRainbow2			= 0x29		# 무지개색
    BodyRedBlue				= 0x2A		# Red-Blue
    BodyCard				= 0x2B		# 카드 색상 표시(이벤트용)
    BodyWarning				= 0x2F		# 경고
    
    # Link
    LinkNone				= 0x30		
    LinkManual				= 0x31		# 수동 제어
    LinkHold				= 0x32		# 지정한 색상을 계속 켬
    LinkFlicker				= 0x33		# 깜빡임			
    LinkFlickerDouble		= 0x34		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    LinkDimming				= 0x35		# 밝기 제어하여 천천히 깜빡임
    LinkSunrise				= 0x36		# 꺼진 상태에서 점점 밝아짐
    LinkSunset				= 0x37		# 켜진 상태에서 점점 어두워짐
    
    EndOfType               = 0x60



class LightFlagsDrone(Enum):
    
    None_			= 0x0000
    
    BodyRed			= 0x0001
    BodyGreen		= 0x0002
    BodyBlue		= 0x0004
    
    TeamRed			= 0x0008
    TeamBlue		= 0x0010
    
    Link			= 0x0080



class LightModeController(Enum):
    
    # Team
    TeamNone						= 0x10
    TeamManual						= 0x11		# 수동 제어
    TeamHold						= 0x12		# 지정한 색상을 계속 켬
    TeamFlicker						= 0x13		# 깜빡임			
    TeamFlickerDouble				= 0x14		# 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)			
    TeamDimming						= 0x15		# 밝기 제어하여 천천히 깜빡임
    TeamSunrise						= 0x16		# 꺼진 상태에서 점점 밝아짐
    TeamSunset						= 0x17		# 켜진 상태에서 점점 어두워짐
    TeamRedBlue						= 0x1A		# Red-Blue
    
    # Array 6
    Array6None						= 0x30
    Array6Manual					= 0x31		# 수동 제어
    Array6Hold						= 0x32		# [전체] 지정한 색상을 계속 켬
    Array6Flicker					= 0x33		# [전체] 깜빡임			
    Array6FlickerDouble				= 0x34		# [전체] 깜빡임(두 번 깜빡이고 깜빡인 시간만큼 꺼짐)		
    Array6Dimming					= 0x35		# [전체] 밝기 제어하여 천천히 깜빡임
    
    # Array 6 Value
    Array6ValueNone					= 0x40		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueHold					= 0x42		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueFlicker				= 0x43		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueFlickerDouble		= 0x44		# [개별] 0 ~ 255 사이의 값 표시
    Array6ValueDimming				= 0x45		# [개별] 0 ~ 255 사이의 값 표시
    
    # Array 6 Function
    Array6FunctionNone				= 0x50
    Array6Pendulum					= 0x51
    Array6FlowLeft					= 0x52		# [개별] 왼쪽으로 흐름
    Array6FlowRight					= 0x53		# [개별] 오른쪽으로 흐름
    
    EndOfType                       = 0x60



class LightFlagsController(Enum):
    
    None_               = 0x0000

    TeamRed		        = 0x0001
    TeamBlue	        = 0x0002
            
    E0			        = 0x0004
    E1			        = 0x0008
    E2			        = 0x0010
    E3			        = 0x0020
    E4			        = 0x0040
    E5			        = 0x0080



class Color(ISerializable):

    def __init__(self):
        self.r      = 0
        self.g      = 0
        self.b      = 0


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<BBB', self.r, self.g, self.b)


    @classmethod
    def parse(cls, dataArray):
        data = Color()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.r, data.g, data.b = unpack('<BBB', dataArray)
        return data



class Colors(Enum):

    AliceBlue              = 0
    AntiqueWhite           = 1
    Aqua                   = 2
    Aquamarine             = 3
    Azure                  = 4
    Beige                  = 5
    Bisque                 = 6
    Black                  = 7
    BlanchedAlmond         = 8
    Blue                   = 9
    BlueViolet             = 10
    Brown                  = 11
    BurlyWood              = 12
    CadetBlue              = 13
    Chartreuse             = 14
    Chocolate              = 15
    Coral                  = 16
    CornflowerBlue         = 17
    Cornsilk               = 18
    Crimson                = 19
    Cyan                   = 20
    DarkBlue               = 21
    DarkCyan               = 22
    DarkGoldenRod          = 23
    DarkGray               = 24
    DarkGreen              = 25
    DarkKhaki              = 26
    DarkMagenta            = 27
    DarkOliveGreen         = 28
    DarkOrange             = 29
    DarkOrchid             = 30
    DarkRed                = 31
    DarkSalmon             = 32
    DarkSeaGreen           = 33
    DarkSlateBlue          = 34
    DarkSlateGray          = 35
    DarkTurquoise          = 36
    DarkViolet             = 37
    DeepPink               = 38
    DeepSkyBlue            = 39
    DimGray                = 40
    DodgerBlue             = 41
    FireBrick              = 42
    FloralWhite            = 43
    ForestGreen            = 44
    Fuchsia                = 45
    Gainsboro              = 46
    GhostWhite             = 47
    Gold                   = 48
    GoldenRod              = 49
    Gray                   = 50
    Green                  = 51
    GreenYellow            = 52
    HoneyDew               = 53
    HotPink                = 54
    IndianRed              = 55
    Indigo                 = 56
    Ivory                  = 57
    Khaki                  = 58
    Lavender               = 59
    LavenderBlush          = 60
    LawnGreen              = 61
    LemonChiffon           = 62
    LightBlue              = 63
    LightCoral             = 64
    LightCyan              = 65
    LightGoldenRodYellow   = 66
    LightGray              = 67
    LightGreen             = 68
    LightPink              = 69
    LightSalmon            = 70
    LightSeaGreen          = 71
    LightSkyBlue           = 72
    LightSlateGray         = 73
    LightSteelBlue         = 74
    LightYellow            = 75
    Lime                   = 76
    LimeGreen              = 77
    Linen                  = 78
    Magenta                = 79
    Maroon                 = 80
    MediumAquaMarine       = 81
    MediumBlue             = 82
    MediumOrchid           = 83
    MediumPurple           = 84
    MediumSeaGreen         = 85
    MediumSlateBlue        = 86
    MediumSpringGreen      = 87
    MediumTurquoise        = 88
    MediumVioletRed        = 89
    MidnightBlue           = 90
    MintCream              = 91
    MistyRose              = 92
    Moccasin               = 93
    NavajoWhite            = 94
    Navy                   = 95
    OldLace                = 96
    Olive                  = 97
    OliveDrab              = 98
    Orange                 = 99
    OrangeRed              = 100
    Orchid                 = 101
    PaleGoldenRod          = 102
    PaleGreen              = 103
    PaleTurquoise          = 104
    PaleVioletRed          = 105
    PapayaWhip             = 106
    PeachPuff              = 107
    Peru                   = 108
    Pink                   = 109
    Plum                   = 110
    PowderBlue             = 111
    Purple                 = 112
    RebeccaPurple          = 113
    Red                    = 114
    RosyBrown              = 115
    RoyalBlue              = 116
    SaddleBrown            = 117
    Salmon                 = 118
    SandyBrown             = 119
    SeaGreen               = 120
    SeaShell               = 121
    Sienna                 = 122
    Silver                 = 123
    SkyBlue                = 124
    SlateBlue              = 125
    SlateGray              = 126
    Snow                   = 127
    SpringGreen            = 128
    SteelBlue              = 129
    Tan                    = 130
    Teal                   = 131
    Thistle                = 132
    Tomato                 = 133
    Turquoise              = 134
    Violet                 = 135
    Wheat                  = 136
    White                  = 137
    WhiteSmoke             = 138
    Yellow                 = 139
    YellowGreen            = 140
    
    EndOfType              = 141



class LightManual(ISerializable):

    def __init__(self):
        self.flags          = 0
        self.brightness     = 0


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<HB', self.flags, self.brightness)


    @classmethod
    def parse(cls, dataArray):
        data = LightManual()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.flags, data.brightness = unpack('<HB', dataArray)
        return data



class LightMode(ISerializable):

    def __init__(self):
        self.mode        = 0
        self.interval    = 0


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<BH', self.mode, self.interval)


    @classmethod
    def parse(cls, dataArray):
        data = LightMode()
        
        if len(dataArray) != cls.getSize():
            return None

        data.mode, data.interval = unpack('<BH', dataArray)
        return data



class LightEvent(ISerializable):

    def __init__(self):
        self.event      = 0
        self.interval   = 0
        self.repeat     = 0


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<BHB', self.event, self.interval, self.repeat)


    @classmethod
    def parse(cls, dataArray):
        data = LightEvent()
        
        if len(dataArray) != cls.getSize():
            return None

        data.event, data.interval, data.repeat = unpack('<BHB', dataArray)

        return data



class LightModeColor(ISerializable):
    
    def __init__(self):
        self.mode       = LightMode()
        self.color      = Color()


    @classmethod
    def getSize(cls):
        return LightMode.getSize() + Color.getSize()


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.mode.toArray())
        dataArray.extend(self.color.toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = LightModeColor()
        
        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0;        indexEnd = LightMode.getSize();      data.mode       = LightMode.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += Color.getSize();         data.color      = Color.parse(dataArray[indexStart:indexEnd])
        return data



class LightModeColors(ISerializable):
    
    def __init__(self):
        self.mode       = LightMode()
        self.colors     = Colors.Black


    @classmethod
    def getSize(cls):
        return LightMode.getSize() + 1


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.mode.toArray())
        dataArray.extend(pack('<B', self.colors.value))
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = LightModeColors()
        
        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0;        indexEnd = LightMode.getSize();      data.mode       = LightMode.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 1;                       data.colors,    = unpack('<B', dataArray[indexStart:indexEnd])

        data.colors     = Colors(data.colors)

        return data



class LightEventColor(ISerializable):
    
    def __init__(self):
        self.event      = LightEvent()
        self.color      = Color()


    @classmethod
    def getSize(cls):
        return LightEvent.getSize() + Color.getSize()


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.event.toArray())
        dataArray.extend(self.color.toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = LightEventColor()
        
        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0;        indexEnd = LightEvent.getSize();     data.event      = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += Color.getSize();         data.command    = Color.parse(dataArray[indexStart:indexEnd])
        
        return data



class LightEventColors(ISerializable):
    
    def __init__(self):
        self.event      = LightEvent()
        self.colors     = Colors.Black


    @classmethod
    def getSize(cls):
        return LightEvent.getSize() + 1


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.event.toArray())
        dataArray.extend(pack('<B', self.colors.value))
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = LightEventColors()
        
        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0;        indexEnd = LightEvent.getSize();     data.event      = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += 1;                       data.colors,    = unpack('<B', dataArray[indexStart:indexEnd])

        data.colors     = Colors(data.colors)

        return data


# Light End


# Buzzer Start


class BuzzerMode(Enum):

    Stop                = 0    # 정지(Mode에서의 Stop은 통신에서 받았을 때 Buzzer를 끄는 용도로 사용, set으로만 호출)

    MuteInstantly       = 1    # 묵음 즉시 적용
    MuteContinually     = 2    # 묵음 예약

    ScaleInstantly      = 3    # 음계 즉시 적용
    ScaleContinually    = 4    # 음계 예약

    HzInstantly         = 5    # 주파수 즉시 적용
    HzContinually       = 6    # 주파수 예약

    EndOfType           = 7



class BuzzerScale(Enum):

    C1 = 0x00; CS1 = 0x01; D1 = 0x02; DS1 = 0x03; E1 = 0x04; F1 = 0x05; FS1 = 0x06; G1 = 0x07; GS1 = 0x08; A1 = 0x09; AS1 = 0x0A; B1 = 0x0B
    C2 = 0x0C; CS2 = 0x0D; D2 = 0x0E; DS2 = 0x0F; E2 = 0x10; F2 = 0x11; FS2 = 0x12; G2 = 0x13; GS2 = 0x14; A2 = 0x15; AS2 = 0x16; B2 = 0x17
    C3 = 0x18; CS3 = 0x19; D3 = 0x1A; DS3 = 0x1B; E3 = 0x1C; F3 = 0x1D; FS3 = 0x1E; G3 = 0x1F; GS3 = 0x20; A3 = 0x21; AS3 = 0x22; B3 = 0x23
    C4 = 0x24; CS4 = 0x25; D4 = 0x26; DS4 = 0x27; E4 = 0x28; F4 = 0x29; FS4 = 0x2A; G4 = 0x2B; GS4 = 0x2C; A4 = 0x2D; AS4 = 0x2E; B4 = 0x2F

    C5 = 0x30; CS5 = 0x31; D5 = 0x32; DS5 = 0x33; E5 = 0x34; F5 = 0x35; FS5 = 0x36; G5 = 0x37; GS5 = 0x38; A5 = 0x39; AS5 = 0x3A; B5 = 0x3B
    C6 = 0x3C; CS6 = 0x3D; D6 = 0x3E; DS6 = 0x3F; E6 = 0x40; F6 = 0x41; FS6 = 0x42; G6 = 0x43; GS6 = 0x44; A6 = 0x45; AS6 = 0x46; B6 = 0x47
    C7 = 0x48; CS7 = 0x49; D7 = 0x4A; DS7 = 0x4B; E7 = 0x4C; F7 = 0x4D; FS7 = 0x4E; G7 = 0x4F; GS7 = 0x50; A7 = 0x51; AS7 = 0x52; B7 = 0x53
    C8 = 0x54; CS8 = 0x55; D8 = 0x56; DS8 = 0x57; E8 = 0x58; F8 = 0x59; FS8 = 0x5A; G8 = 0x5B; GS8 = 0x5C; A8 = 0x5D; AS8 = 0x5E; B8 = 0x5F

    EndOfType   = 0x60

    Mute        = 0xEE  # 묵음
    Fin         = 0xFF  # 악보의 끝





class BuzzerMelody(Enum):
    
    DoMiSol     = 0x00		# 도미솔
    SolMiDo     = 0x01		# 솔미도
    LaLa        = 0x02		# 라라
    SiRaSiRa    = 0x03		# 시라시라
    
    Warning1    = 0x04		# 경고 1
    Warning2    = 0x05		# 경고 2
    Warning3    = 0x06		# 경고 3
    Warning4    = 0x07		# 경고 4
    
    Du          = 0x08		# Trim -
    DuDu        = 0x09		# Trim - End
    DiDic       = 0x0A		# Trim Center
    DiDic2      = 0x0B		# Trim Center 2
    Di          = 0x0C		# Trim +
    DiDi        = 0x0D		# Trim + End
    
    BuzzSound1   = 0x0E
    BuzzSound2   = 0x0F
    BuzzSound3   = 0x10
    BuzzSound4   = 0x11
    
    Button       = 0x12
    Shot         = 0x13
    
    EndOfType    = 0x14




class Buzzer(ISerializable):

    def __init__(self):
        self.mode       = BuzzerMode.Stop
        self.value      = 0
        self.time       = 0


    @classmethod
    def getSize(cls):
        return 5


    def toArray(self):
        return pack('<BHH', self.mode.value, self.value, self.time)


    @classmethod
    def parse(cls, dataArray):
        data = Buzzer()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.mode, data.value, data.time = unpack('<BHH', dataArray)

        data.mode = BuzzerMode(data.mode)

        return data


# Buzzer End


# Button Start


class ButtonFlagController(Enum):

    None_   			= 0x0000
                
    # 버튼
    FrontLeft			= 0x0001
    FrontRight			= 0x0002

    MidUpLeft			= 0x0004
    MidUpRight			= 0x0008

    MidUp				= 0x0010
    MidLeft				= 0x0020
    MidRight			= 0x0040
    MidDown				= 0x0080

    BottomLeft			= 0x0100
    BottomRight			= 0x0200



class ButtonFlagDrone(Enum):

    None_               = 0x0000
    
    Reset               = 0x0001



class ButtonEvent(Enum):

    None_               = 0x00
    
    Down                = 0x01  # 누르기 시작
    Press               = 0x02  # 누르는 중
    Up                  = 0x03  # 뗌
    
    EndContinuePress    = 0x04  # 연속 입력 종료



class Button(ISerializable):

    def __init__(self):
        self.button     = 0
        self.event      = ButtonEvent.None_


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<HB', self.button, self.event.value)


    @classmethod
    def parse(cls, dataArray):
        data = Button()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.button, data.event = unpack('<HB', dataArray)

        data.event = ButtonEvent(data.event)
        
        return data


# Button End



# Joystick Start


class JoystickDirection(Enum):

    None_   = 0x00      # 정의하지 않은 영역(무시함)

    VT      = 0x10      #   위(세로)
    VM      = 0x20      # 중앙(세로)
    VB      = 0x40      # 아래(세로)

    HL      = 0x01      #   왼쪽(가로)
    HM      = 0x02      #   중앙(가로)
    HR      = 0x04      # 오른쪽(가로)

    TL = 0x11  
    TM = 0x12  
    TR = 0x14
    ML = 0x21  
    CN = 0x22  
    MR = 0x24
    BL = 0x41  
    BM = 0x42  
    BR = 0x44



class JoystickEvent(Enum):

    None_       = 0     # 이벤트 없음
    
    In          = 1     # 특정 영역에 진입
    Stay        = 2     # 특정 영역에서 상태 유지
    Out         = 3     # 특정 영역에서 벗어남
    
    EndOfType   = 4



class JoystickBlock(ISerializable):

    def __init__(self):
        self.x          = 0
        self.y          = 0
        self.direction  = JoystickDirection.None_
        self.event      = JoystickEvent.None_


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<bbBB', self.x, self.y, self.direction.value, self.event.value)


    @classmethod
    def parse(cls, dataArray):
        data = JoystickBlock()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.x, data.y, data.direction, data.event = unpack('<bbBB', dataArray)

        data.direction  = JoystickDirection(data.direction)
        data.event      = JoystickEvent(data.event)

        return data



class Joystick(ISerializable):

    def __init__(self):
        self.left       = JoystickBlock()
        self.right      = JoystickBlock()


    @classmethod
    def getSize(cls):
        return JoystickBlock().getSize() * 2


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.left.toArray())
        dataArray.extend(self.right.toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = Joystick()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd  = JoystickBlock.getSize();     data.left   = JoystickBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += JoystickBlock.getSize();     data.right  = JoystickBlock.parse(dataArray[indexStart:indexEnd])

        return data


# Joystick End



# Sensor Raw Start


class RawMotion(ISerializable):

    def __init__(self):
        self.accelX     = 0
        self.accelY     = 0
        self.accelZ     = 0
        self.gyroRoll   = 0
        self.gyroPitch  = 0
        self.gyroYaw    = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<hhhhhh', self.accelX, self.accelY, self.accelZ, self.gyroRoll, self.gyroPitch, self.gyroYaw)


    @classmethod
    def parse(cls, dataArray):
        data = RawMotion()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.accelX, data.accelY, data.accelZ, data.gyroRoll, data.gyroPitch, data.gyroYaw = unpack('<hhhhhh', dataArray)
        
        return data



# Sensor Raw End



# Information Start


class State(ISerializable):

    def __init__(self):
        self.modeSystem         = ModeSystem.None_
        self.modeFlight         = ModeFlight.None_
        self.modeControlFlight  = ModeControlFlight.None_
        self.modeMovement       = ModeMovement.None_
        self.headless           = Headless.None_
        self.controlSpeed       = 0
        self.sensorOrientation  = SensorOrientation.None_
        self.battery            = 0


    @classmethod
    def getSize(cls):
        return 8


    def toArray(self):
        return pack('<BBBBBBBB', self.modeSystem.value, self.modeFlight.value, self.modeControlFlight.value, self.modeMovement.value, self.headless.value, self.controlSpeed, self.sensorOrientation.value, self.battery)


    @classmethod
    def parse(cls, dataArray):
        data = State()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.modeSystem, data.modeFlight, data.modeControlFlight, data.modeMovement, data.headless, data.controlSpeed, data.sensorOrientation, data.battery = unpack('<BBBBBBBB', dataArray)

        data.modeSystem         = ModeSystem(data.modeSystem)
        data.modeFlight         = ModeFlight(data.modeFlight)
        data.modeControlFlight  = ModeControlFlight(data.modeControlFlight)
        data.modeMovement       = ModeMovement(data.modeMovement)
        data.headless           = Headless(data.headless)
        data.sensorOrientation  = SensorOrientation(data.sensorOrientation)
        
        return data



class Attitude(ISerializable):

    def __init__(self):
        self.roll       = 0
        self.pitch      = 0
        self.yaw        = 0


    @classmethod
    def getSize(cls):
        return 6


    def toArray(self):
        return pack('<hhh', self.roll, self.pitch, self.yaw)


    @classmethod
    def parse(cls, dataArray):
        data = Attitude()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.roll, data.pitch, data.yaw = unpack('<hhh', dataArray)
        
        return data
        


class Position(ISerializable):

    def __init__(self):
        self.x      = 0
        self.y      = 0
        self.z      = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<fff', self.x, self.y, self.z)


    @classmethod
    def parse(cls, dataArray):
        data = Position()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.x, data.y, data.z = unpack('<fff', dataArray)
        
        return data



class Altitude(ISerializable):

    def __init__(self):
        self.temperature    = 0
        self.pressure       = 0
        self.altitude       = 0
        self.rangeHeight    = 0


    @classmethod
    def getSize(cls):
        return 16


    def toArray(self):
        return pack('<ffff', self.temperature, self.pressure, self.altitude, self.rangeHeight)


    @classmethod
    def parse(cls, dataArray):
        data = Altitude()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.temperature, data.pressure, data.altitude, data.rangeHeight = unpack('<ffff', dataArray)
        
        return data



class Motion(ISerializable):

    def __init__(self):
        self.accelX     = 0
        self.accelY     = 0
        self.accelZ     = 0
        self.gyroRoll   = 0
        self.gyroPitch  = 0
        self.gyroYaw    = 0
        self.angleRoll  = 0
        self.anglePitch = 0
        self.angleYaw   = 0


    @classmethod
    def getSize(cls):
        return 18


    def toArray(self):
        return pack('<hhhhhhhhh', self.accelX, self.accelY, self.accelZ, self.gyroRoll, self.gyroPitch, self.gyroYaw, self.angleRoll, self.anglePitch, self.angleYaw)


    @classmethod
    def parse(cls, dataArray):
        data = Motion()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.accelX, data.accelY, data.accelZ, data.gyroRoll, data.gyroPitch, data.gyroYaw, data.angleRoll, data.anglePitch, data.angleYaw = unpack('<hhhhhhhhh', dataArray)
        
        return data



class Range(ISerializable):

    def __init__(self):
        self.left       = 0
        self.front      = 0
        self.right      = 0
        self.rear       = 0
        self.top        = 0
        self.bottom     = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<hhhhhh', self.left, self.front, self.right, self.rear, self.top, self.bottom)


    @classmethod
    def parse(cls, dataArray):
        data = Range()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.left, data.front, data.right, data.rear, data.top, data.bottom = unpack('<hhhhhh', dataArray)
        
        return data



class Trim(ISerializable):

    def __init__(self):
        self.roll       = 0
        self.pitch      = 0
        self.yaw        = 0
        self.throttle   = 0


    @classmethod
    def getSize(cls):
        return 8


    def toArray(self):
        return pack('<hhhh', self.roll, self.pitch, self.yaw, self.throttle)


    @classmethod
    def parse(cls, dataArray):
        data = Trim()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.roll, data.pitch, data.yaw, data.throttle = unpack('<hhhh', dataArray)
        
        return data


# Information End



# Sensor Start


class VisionSensor(ISerializable):
    
    def __init__(self):
        self.x      = 0
        self.y      = 0
        self.z      = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<fff', self.x, self.y, self.z)


    @classmethod
    def parse(cls, dataArray):
        data = VisionSensor()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.x, data.y, data.z = unpack('<fff', dataArray)
        
        return data



class Count(ISerializable):

    def __init__(self):
        self.timeFlight     = 0

        self.countTakeOff   = 0
        self.countLanding   = 0
        self.countAccident  = 0


    @classmethod
    def getSize(cls):
        return 14


    def toArray(self):
        return pack('<QHHH', self.timeFlight, self.countTakeOff, self.countLanding, self.countAccident)


    @classmethod
    def parse(cls, dataArray):
        data = Count()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.timeFlight, data.countTakeOff, data.countLanding, data.countAccident = unpack('<QHHH', dataArray)
        
        return data



class Bias(ISerializable):
    
    def __init__(self):
        self.accelX     = 0
        self.accelY     = 0
        self.accelZ     = 0
        self.gyroRoll   = 0
        self.gyroPitch  = 0
        self.gyroYaw    = 0


    @classmethod
    def getSize(cls):
        return 12


    def toArray(self):
        return pack('<hhhhhh', self.accelX, self.accelY, self.accelZ, self.gyroRoll, self.gyroPitch, self.gyroYaw)


    @classmethod
    def parse(cls, dataArray):
        data = Bias()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.accelX, data.accelY, data.accelZ, data.gyroRoll, data.gyroPitch, data.gyroYaw = unpack('<hhhhhh', dataArray)
        
        return data



class Weight(ISerializable):

    def __init__(self):
        self.weight     = 0


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<f', self.weight)


    @classmethod
    def parse(cls, dataArray):
        data = Weight()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.weight, = unpack('<f', dataArray)
        
        return data



class LostConnection(ISerializable):

    def __init__(self):
        self.timeNeutral    = 0
        self.timeLanding    = 0
        self.timeStop       = 0


    @classmethod
    def getSize(cls):
        return 8


    def toArray(self):
        return pack('<HHI', self.timeNeutral, self.timeLanding, self.timeStop)


    @classmethod
    def parse(cls, dataArray):
        data = LostConnection()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.timeNeutral, data.timeLanding, data.timeStop = unpack('<HHI', dataArray)
        
        return data


# Sensor End



# Device Start


class MotorBlock(ISerializable):

    def __init__(self):
        self.rotation   = Rotation.None_
        self.value      = 0


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<Bh', self.rotation.value, self.value)


    @classmethod
    def parse(cls, dataArray):
        data = MotorBlock()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.rotation, data.value = unpack('<Bh', dataArray)
        data.rotation = Rotation(data.rotation)
        
        return data



class Motor(ISerializable):

    def __init__(self):
        self.motor      = []
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())


    @classmethod
    def getSize(cls):
        return MotorBlock.getSize() * 4


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.motor[0].toArray())
        dataArray.extend(self.motor[1].toArray())
        dataArray.extend(self.motor[2].toArray())
        dataArray.extend(self.motor[3].toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = Motor()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd  = MotorBlock.getSize();    data.motor[0]   = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlock.getSize();    data.motor[1]   = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlock.getSize();    data.motor[2]   = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlock.getSize();    data.motor[3]   = MotorBlock.parse(dataArray[indexStart:indexEnd])
        
        return data


class MotorBlockV(ISerializable):

    def __init__(self):
        self.value      = 0


    @classmethod
    def getSize(cls):
        return 2


    def toArray(self):
        return pack('<h', self.value)


    @classmethod
    def parse(cls, dataArray):
        data = MotorBlockV()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.value, = unpack('<h', dataArray)
        
        return data



class MotorV(ISerializable):

    def __init__(self):
        self.motor      = []
        self.motor.append(MotorBlockV())
        self.motor.append(MotorBlockV())
        self.motor.append(MotorBlockV())
        self.motor.append(MotorBlockV())


    @classmethod
    def getSize(cls):
        return MotorBlockV.getSize() * 4


    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.motor[0].toArray())
        dataArray.extend(self.motor[1].toArray())
        dataArray.extend(self.motor[2].toArray())
        dataArray.extend(self.motor[3].toArray())
        return dataArray


    @classmethod
    def parse(cls, dataArray):
        data = MotorV()
        
        if len(dataArray) != cls.getSize():
            return None
        
        indexStart = 0;        indexEnd  = MotorBlockV.getSize();    data.motor[0]   = MotorBlockV.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlockV.getSize();    data.motor[1]   = MotorBlockV.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlockV.getSize();    data.motor[2]   = MotorBlockV.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd; indexEnd += MotorBlockV.getSize();    data.motor[3]   = MotorBlockV.parse(dataArray[indexStart:indexEnd])
        
        return data



class MotorSingle(ISerializable):

    def __init__(self):
        self.target     = 0
        self.rotation   = Rotation.None_
        self.value      = 0


    @classmethod
    def getSize(cls):
        return 4


    def toArray(self):
        return pack('<BBh', self.target, self.rotation.value, self.value)


    @classmethod
    def parse(cls, dataArray):
        data = MotorSingle()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.target, data.rotation, data.value = unpack('<BBh', dataArray)
        data.rotation = Rotation(data.rotation)
        
        return data



class MotorSingleV(ISerializable):

    def __init__(self):
        self.target     = 0
        self.value      = 0


    @classmethod
    def getSize(cls):
        return 3


    def toArray(self):
        return pack('<Bh', self.target, self.value)


    @classmethod
    def parse(cls, dataArray):
        data = MotorSingleV()
        
        if len(dataArray) != cls.getSize():
            return None
        
        data.target, data.value = unpack('<Bh', dataArray)
        
        return data



class InformationAssembledForController(ISerializable):

    def __init__(self):
        self.angleRoll              = 0
        self.anglePitch             = 0
        self.angleYaw               = 0
        
        self.rpm                    = 0
        
        self.positionX              = 0
        self.positionY              = 0
        self.positionZ              = 0
        
        self.speedX                 = 0
        self.speedY                 = 0
        
        self.rangeHeight            = 0
        
        self.rssi                   = 0


    @classmethod
    def getSize(cls):
        return 18


    def toArray(self):
        return pack('<hhhHhhhbbBb', self.angleRoll, self.anglePitch, self.angleYaw, 
                                    self.rpm,
                                    self.positionX, self.positionY, self.positionZ, 
                                    self.speedX, self.speedY, 
                                    self.rangeHeight,
                                    self.rssi)


    @classmethod
    def parse(cls, dataArray):
        data = InformationAssembledForController()
        
        if len(dataArray) != cls.getSize():
            return None
        
        (data.angleRoll, data.anglePitch, data.angleYaw, 
        data.rpm,
        data.positionX, data.positionY, data.positionZ, 
        data.speedX, data.speedY, 
        data.rangeHeight,
        data.rssi) = unpack('<hhhHhhhbbBb', dataArray)
        
        return data



class InformationAssembledForEntry(ISerializable):

    def __init__(self):
        self.angleRoll      = 0
        self.anglePitch     = 0
        self.angleYaw       = 0
        
        self.positionX      = 0
        self.positionY      = 0
        self.positionZ      = 0

        self.rangeHeight    = 0
        self.altitude       = 0


    @classmethod
    def getSize(cls):
        return 18


    def toArray(self):
        return pack('<hhhhhhhf',    self.angleRoll, self.anglePitch, self.angleYaw,
                                    self.positionX, self.positionY, self.positionZ,
                                    self.rangeHeight, self.altitude)


    @classmethod
    def parse(cls, dataArray):
        data = InformationAssembledForEntry()
        
        if len(dataArray) != cls.getSize():
            return None
        
        (data.angleRoll, data.anglePitch, data.angleYaw, 
        data.positionX, data.positionY, data.positionZ, 
        data.rangeHeight, data.altitude) = unpack('<hhhhhhhf', dataArray)
        
        return data



# Device End




# Card Start




# Card End

