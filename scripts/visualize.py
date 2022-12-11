#!python3
import numpy as np
import matplotlib.pyplot as plt
from   matplotlib.animation import FuncAnimation
import io
from typing import Mapping, Any, Callable, BinaryIO, Dict, List, Tuple
import serial

STREAM_FILE=("/dev/ttyUSB1", "serial")
HeaderSpec = Mapping[str, Callable[[BinaryIO], int]]
Header = Mapping[str, Any]
SerialData = Tuple[Header, List[float]]


# ********************************* Parsers ********************************* #
def stdint_read(f:BinaryIO, size_bytes=2, signed=False) -> int:
    raw = bytes()
    for _ in range(size_bytes):
        raw += f.read(1)
    return int.from_bytes(raw, "little", signed=signed)

# ****************************** Serial Manager ***************************** #
class SerialHeaderManager:
    HEAD = b'head'
    TAIL = b'tail'
    def __init__(self, file:BinaryIO, spec: HeaderSpec, def_packet:Header) -> None:
        self._file = file 
        self._spec = spec 
        self._last_packet = def_packet
        self._data = np.zeros(self._last_packet['N'])
    
    def wait_for_packet(self) -> SerialData:
        return (self.__find_header(), self.__read_data()) 
    
    def __find_header(self) -> Mapping[str, Any]:
        found = False 
        ret: Dict[str, Any] = {"head": SerialHeaderManager.HEAD}
        while not found:
            self.__find_head()
            print('Packet found!')
            for key, parser in self._spec.items():
                ret.update({key: parser(self._file)})
            print(ret)
            found = self.__find_tail()
        self._last_packet = ret
        print(self._last_packet)
        return ret
    
    def __read_data(self) -> List[float]:
        parse = lambda: stdint_read(self._file, signed=True) / 0xFFFF * 1.65
        return [parse() for _ in range(self._last_packet['N'])]

    def __find_head(self):
        data=bytearray(len(SerialHeaderManager.HEAD))
        print(data)
        while data!=SerialHeaderManager.HEAD:
            raw=self._file.read(1)
            data+=raw
            data[:]=data[-4:]
        self._file.read(2)  # Terminator.
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

fftAxe = fig.add_subplot ( 2,1,2                  )
fftLn, = plt.plot        ( [],[],'b-',linewidth=4 )
fftAxe.grid              ( True                   )
fftAxe.set_ylim          ( 0 ,0.25                )

header = {
    "head": b"head", 
    "id": 0, 
    "N":1024, 
    "fs": 8000, 
    "dgb1": 0, 
    "dgb2": 0, 
    "dgb3": 0,  
    "tail":b"tail"
}

header_spec = {
    "id": stdint_read, 
    "N": stdint_read, 
    "fs": stdint_read, 
    "dgb1": stdint_read, 
    "dgb2": stdint_read, 
    "dgb3": stdint_read,  
}


def flushStream(f,h):
    if(STREAM_FILE[1]=="serial"): #pregunto si estoy usando la bibioteca pyserial o un file
        f.flushInput()
    else:
        f.seek ( 2*h["N"],io.SEEK_END)

if(STREAM_FILE[1]=="serial"):
    streamFile = serial.Serial(port=STREAM_FILE[0],baudrate=460800,timeout=None)
else:
    streamFile=open(STREAM_FILE[0], "rb", 0)
    flushStream(streamFile, header)

serial_manager = SerialHeaderManager(streamFile, header_spec, header)

rec=np.ndarray(1).astype(np.int16)
def init():
    global rec
    rec=np.ndarray(1).astype(np.int16)
    return adcLn, fftLn

def update(t):
    global header,rec
    found_h, raw_data = serial_manager.wait_for_packet() 
    id, N, fs = found_h["id"], found_h["N"], found_h["fs"]
    
    adc   = np.array(raw_data)
    time  = np.arange(0, N/fs, 1/fs)

#    adcAxe.set_xlim ( 0    ,N/fs )
#    adcLn.set_data  ( time ,adc  )

    fft=np.abs (1 / N * np.fft.fft(adc))**2
    fftAxe.set_ylim (0, np.max(fft)+0.05)
    fftAxe.set_xlim (0 ,fs/2 )
    fftLn.set_data ( (fs/N )*fs*time ,fft)

    rec=np.concatenate((rec,((adc/1.65)*2**(15-1)).astype(np.int16)))
    clean = False
    with open('./scripts/comms.txt', 'rt') as f:
        for l in f.readline():
            streamFile.write(l.encode())
            clean = True
    if clean:
        with open('./scripts/comms.txt', 'w'):
            pass
    return adcLn, fftLn

#seleccionar si usar la biblioteca pyserial o leer desde un archivo log.bin

if __name__ == '__main__':
    try:
        while True:
            serial_manager.wait_for_packet()
            print('Hi packet')
    except KeyboardInterrupt:
        pass
    finally:
        streamFile.close()
else:
    ani=FuncAnimation(fig, update, 128, init_func=init, blit=True, interval=1, 
                    repeat=True)
    plt.draw()
    plt.show()
