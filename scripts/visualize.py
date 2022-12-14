#!python3
import numpy as np
import matplotlib.pyplot as plt
import functools
from   matplotlib.animation import FuncAnimation
import io
from typing import Mapping, Any, Callable, BinaryIO, Dict, List, Tuple, Protocol
import serial

STREAM_FILE=("/dev/ttyUSB1", "serial")
HeaderSpec = Mapping[str, Callable[[BinaryIO], int]]
Header = Mapping[str, Any]
SerialData = Tuple[Header, List[float], List[float]]

# ********************************* Parsers ********************************* #
def stdint_read(f:BinaryIO, size_bytes=2, signed=False) -> int:
    raw = bytes()
    for _ in range(size_bytes):
        raw += f.read(1)
    return int.from_bytes(raw, "little", signed=signed)

# ********************************* Streams ********************************* #
class Streamable(Protocol):
    def open(self) -> BinaryIO:
        ...
    
    def flush(self):
        ...

class SerialStreamable(Streamable):
    def __init__(self, port:str='/dev/ttyUSB1') -> None:
        self.port = port
    
    def open(self) -> BinaryIO:
        self.f = serial.Serial(port=self.port, baudrate=460800, timeout=None)
        return self.f  # type: ignore
    
    def flush(self):
        self.f.flushInput()

class FileStreamable(Streamable):
    def __init__(self, sz_bytes, filepath:str='serial') -> None:
        self.fpath = filepath
        self.sz = sz_bytes

    def open(self) -> BinaryIO:
        self.f = open(self.fpath, "rb", 0)
        self.flush()
        return self.f
    
    def flush(self):
        self.f.seek(2*self.sz, io.SEEK_END)

# ****************************** Serial Manager ***************************** #
class SerialHeaderManager:
    HEAD = b'head'
    TAIL = b'tail'
    def __init__(self, file:BinaryIO, spec: HeaderSpec, def_packet:Header) -> None:
        self._file = file 
        self._spec = spec 
        self._last_packet = def_packet
    
    def wait_for_packet(self) -> SerialData:
        return (
            self.__find_header(), 
            self.__read_data(self._last_packet['N']),
            self.__read_data(self._last_packet['Ndbg'])) 
    
    def __find_header(self) -> Mapping[str, Any]:
        found = False 
        ret: Dict[str, Any] = {"head": SerialHeaderManager.HEAD}
        while not found:
            self.__find_head()
            for key, parser in self._spec.items():
                ret.update({key: parser(self._file)})
            found = self.__find_tail()
        self._last_packet = ret
        return ret
    
    def __read_data(self, n:int) -> List[float]:
        parse = lambda: stdint_read(self._file, signed=True) / 0xFFFF * 1.65
        return [parse() for _ in range(n)]

    def __find_head(self):
        data=bytearray(len(SerialHeaderManager.HEAD))
        while data!=SerialHeaderManager.HEAD:
            raw=self._file.read(1)
            data+=raw
            data[:]=data[-4:]
    def __find_tail(self):
        data=bytearray(b'1234')
        for _ in range(4):
            data+=self._file.read(1)
            data[:]=data[-4:]
        return data == SerialHeaderManager.TAIL

# ********************************* Plotting ******************************** #
fig = plt.figure(1)


adcAxe = fig.add_subplot ( 2,1,1                  )
adcLn, = plt.plot        ( [],[],'r-',linewidth=4 )
adcAxe.grid              ( True                   )
adcAxe.set_ylim          ( -1.65 ,1.65            )
adcAxe.set_title('Scope')
adcAxe.set_xlabel('Time [Sec]')
adcAxe.set_ylabel('Mag')

fftAxe = fig.add_subplot ( 2,1,2                  )
fftLn, = plt.plot        ( [],[],'b-',linewidth=4 )
fftAxe.grid              ( True                   )
# fftAxe.set_ylim          ( 0 ,0.25                )
fftAxe.set_ylim          ( -0.05 ,0.05            )
fftAxe.set_title('Dbg Signal')
fftAxe.set_xlabel('Time [Sec]')
fftAxe.set_ylabel('Mag')

plt.tight_layout()

header = {
    "head": b"head", 
    "id": 0, 
    "N":1024, 
    "Ndbg": 273,
    "fs": 8000, 
    "dgb1": 0, 
    "dgb2": 0, 
    "dgb3": 0,  
    "tail":b"tail"
}

header_spec = {
    "id": functools.partial(stdint_read, size_bytes=4), 
    "N": stdint_read, 
    "Ndbg": stdint_read, 
    "fs": stdint_read, 
    "dgb1": stdint_read, 
    "dgb2": stdint_read, 
    "dgb3": lambda x: '0x' + hex(stdint_read(x)) ,  
}


stream = SerialStreamable()
stream_file = stream.open()
serial_manager = SerialHeaderManager(stream_file, header_spec, header)

rec=np.ndarray(1).astype(np.int16)

def init():
    global rec
    rec=np.ndarray(1).astype(np.int16)
    return adcLn, fftLn

def update(t):
    global header,rec
    found_h, raw_data, dbg_data = serial_manager.wait_for_packet() 
    id, N, fs = found_h["id"], found_h["N"], found_h["fs"]
    print(found_h)
    
    adc   = np.array(raw_data)
    time  = np.arange(0, N/fs, 1/fs)

    dbg = np.array(dbg_data)
    time2 = np.arange(len(dbg_data))

    adcAxe.set_xlim ( 0    ,N/fs )
    adcLn.set_data  ( time ,adc  )

    fftAxe.set_xlim (0 , time2[-1])
    # fftAxe.set_ylim (min(np.min(dbg) * 1.05, 0),  max(np.max(dbg) * 1.05, 0))
    fftLn.set_data (time2, dbg)

    rec=np.concatenate((rec,((adc/1.65)*2**(15-1)).astype(np.int16)))

    clean = False
    with open('./scripts/comms.txt', 'rt') as f:
        for l in f.readline():
            stream_file.write(l.encode())
            clean = True
    if clean:
        with open('./scripts/comms.txt', 'w'):
            pass

    return adcLn, fftLn


if __name__ == '__main__':
    ani=FuncAnimation(fig, update, 128, init_func=init, blit=True, interval=1, 
                    repeat=True)
    plt.draw()
    plt.show()
    stream_file.close()