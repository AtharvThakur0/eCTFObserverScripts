import pyvisa
import serial
import numpy as np
from smooth import smooth
import time
import argparse

ACK = b'%A\x00\x00'

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

wav_start = 2.5e5
wav_stop = 4e5
wav_tolerance = int(1.5e5)
sample_start = int(wav_start-wav_tolerance)
scope.write(f":WAV:STAR {sample_start}")
scope.write(f":WAV:STOP {wav_stop}")

scope.write(":WAV:SOUR CHAN1")
scope.write(":WAV:MODE RAW")
scope.write(":WAV:FORM BYTE")

traces_to_average = 1

trig_idx = None

with serial.Serial(args.port, args.baudrate) as ser:
    while True:
        traces = np.zeros((traces_to_average, int(wav_stop-wav_start)), dtype=np.uint8)
        for i in range(traces_to_average):
        
            ser.write(b'p')
            #print("Sent trig

            '''
            print(scope.query(":TRIG:STAT?"))
            print(scope.query(":ACQ:SRAT?"))
            print(scope.query(":WAV:PRE?"))
            '''
            time.sleep(.1)
            while (ser.in_waiting):
                ser.read_all();
                ser.write(ACK);
                time.sleep(.01)

            
            trigger_status = scope.query(":TRIG:STAT?");
            while (trigger_status[0] != "S"):
                #print(f"STATUS: {trigger_status}")
                trigger_status = scope.query(":TRIG:STAT?");
            #input("Press Enter to continue...")
            data = scope.query_binary_values(":WAV:DATA?", datatype='B')
            data = -np.array(data, dtype=np.uint8)
            if (len(data) == 0):
                print(f"No data received from scope on trace {i}, exiting.")
                exit(-1)

            if (trig_idx is None): # the RIGOL trig pos query is unreliable
                scope.write(":WAV:SOUR CHAN2")
                
                scope.write(f":WAV:STOP {wav_start+wav_tolerance}")
                time.sleep(.1)
                trigger_data = scope.query_binary_values(":WAV:DATA?", datatype='B')
                trig_idx = np.argmax(np.abs(np.diff(trigger_data)))
                print(f"Calculated idx: {trig_idx}")

                
                scope.write(":WAV:SOUR CHAN1")
                scope.write(f":WAV:STOP {wav_stop}")

            data = data[(trig_idx):(trig_idx+int(wav_stop-wav_start))]


            '''
            average_adjacent = 2
            averaged_trace_size = len(data)//average_adjacent
            trace = np.zeros(averaged_trace_size, dtype=np.uint16)
            for j in range(average_adjacent): # average across adjacent samples
                trace += data[j::average_adjacent][:averaged_trace_size] # cut off extras
            trace //= (average_adjacent if average_adjacent > 0 else 1)
            '''

            if (len(data) > 0):
                pass
            else:
                print("No data saved")
                exit(-1)
            
            traces[i,:] = data
        
            if (i < traces_to_average-1):
                scope.write(":SING")
            
            status = scope.query("*OPC?");
            while ( status[0] != '1'):
                #print(f"STATUS: {status}")
                status = scope.query("*OPC?");

            time.sleep(.1)

        traces = smooth(traces)
        trace = np.mean(traces, axis=0)
        np.savetxt(f"power_traces/{i}.csv",trace, delimiter=',');
        print("Saved data to file   "+f"power_traces/{i}.csv")
        
        break;