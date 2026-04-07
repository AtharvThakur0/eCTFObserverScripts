import pyvisa
import serial
import numpy as np
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--port", default="COM3", help="Serial port")
parser.add_argument("--baudrate", type=int, default=921600, help="Baud rate")
args = parser.parse_args()

'''RIGOL DS1054 settings:
AX 4.240ms
AY 54.49 mV
BX = 4.1 ms
BY 47.43 mV
BX-AX = -140 us
BY-AY = -7.06 mV
1/dX = 7.14 kHz
500 uS
50.0 MSa/s
600k pts
High Res Acquire
CHANNEL 1 POS 660 mV
CHANNEL 1 SCALE 10 mV
CHANNEL 2 POS -40 mV
CHANNEL 2 SCALE 9.8 mV
'''

rm = pyvisa.ResourceManager()
res = rm.list_resources()
print(res)
scope = rm.open_resource(res[0])
# print(inst.query('*IDN?'))

#scope.write(":CHAN1:DISP ON")
#scope.write(":CHAN1:SCAL 1.000000e-02")  
#scope.write(":CHAN1:OFFS 0")
# scope.write(":CHAN1:PROB 1") 
#scope.write(":CHAN1:RANG .5")

#scope.write(":ACQ:MDEP 12000")
#scope.write(":TIM:MAIN:SCAL .5e-03")
print(f"Acquiring at {scope.query(':ACQ:SRAT?')}")

#scope.write(":TIMebase:DELay:OFFSet -1")

#scope.write(":TRIG:MODE EDGE")
#scope.write(":TRIG:EDGE:SOUR CHAN2")
#scope.write(":TRIG:EDGE:SLOP POS")
#scope.write(":TRIG:EDGE:LEV 0.5")   

scope.write(":WAV:SOUR CHAN1")
scope.write(":WAV:MODE RAW")
scope.write(":WAV:FORM BYTE")

scope.write(":WAV:STAR 100000")
scope.write(":WAV:STOP 475000")

i=0
with serial.Serial(args.port, args.baudrate) as ser:
    while True:
        ser.write(b'p')
        print("Sent trigger to device, waiting for data...")
        print(scope.query("*OPC?"))

        '''
        print(scope.query(":TRIG:STAT?"))
        print(scope.query(":ACQ:SRAT?"))
        print(scope.query(":WAV:PRE?"))
        '''
         
        while (ser.in_waiting):
            ser.read_all();
        #input("Press Enter to continue...")
        data = scope.query_binary_values(":WAV:DATA?", datatype='B')
        data = np.array(data)


        if (len(data) > 0):
            np.savetxt(f"power_traces/{i}.csv",data, delimiter=',');
            print("Saved data to file   "+f"power_traces/{i}.csv")
            i+=1
        else:
            print("No data saved")

        #scope.write(":SING")
        break;