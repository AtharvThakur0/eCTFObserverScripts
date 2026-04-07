import serial
import struct
import matplotlib.pyplot as plt
import numpy as np
import time
import argparse
import random
import traceback
from tqdm import tqdm
import pyvisa

PACKET_SIZE = 128
KEY_SIZE = 16
ACK = b'%A\x00\x00'

def main():
    
    parser = argparse.ArgumentParser(description='AES Side-Channel Analysis')
    parser.add_argument('--test', action='store_true', help='Test serial communication with the device')
    parser.add_argument('--generate_plaintexts', action='store_true', help='Generate plaintexts for power trace collection')
    parser.add_argument('--correlation', '-c', default='', help='Perform correlation analysis on collected traces')
    parser.add_argument('--differential', '-d', default='', help='Perform differential analysis on collected traces')
    parser.add_argument("--partial", type=int, default=-1,help='Restart from partial trace #')
    parser.add_argument("--average", action='store_true', help='Collect power traces and compute averages for each byte value (OUTDATED)')
    parser.add_argument("--traces_per_byte", "-m", type=int, default=128, help='Number of plaintexts to trace')
    parser.add_argument("--traces_per_plaintext", "-n", type=int, default=1, help='Number of traces to request per plaintext')
    parser.add_argument("--samples", "-s", type=int, default=1536, help='Number of samples to request per trace')
    parser.add_argument("--null_plaintext", action='store_true', help='Collect traces for null plaintext')
    parser.add_argument("--port", type=str, default='COM3', help='Serial port to use for communication (default: COM10)')
    parser.add_argument("--baud", type=int, default=921600, help='Baud rate for serial communication (default: 921600)')
    parser.add_argument("--use_scope", action='store_true', default=True,help='Use oscilloscope for trace acquisition instead of ADC on the device')
    parser.add_argument("--average_adjacent", type=int, default=5, help='Number of adjacent samples to average for noise reduction when using oscilloscope (default: 5). Set to 0 or 1 to disable averaging.')
    parser.add_argument("--wav_start", type=int, default=1e5, help='Starting sample index for oscilloscope acquisition (default: 100000)')
    parser.add_argument("--wav_stop", type=int, default=3.75e5, help='Stopping sample index for oscilloscope acquisition (default: 375000)')
    parser.add_argument("target_byte_indices", nargs='*', type=int, default=[0], help='Indices of the target bytes to analyze (default: [0])')
    

    args = parser.parse_args()

    if args.test:
        testFunctionality(port=args.port, baud=args.baud)
        return
    if args.generate_plaintexts:
        generatePlaintexts(args.traces_per_byte)
        return
    if args.correlation:
        Sbox = getSbox()
        for idx in args.target_byte_indices:
            correlationAnalysis(Sbox, args.correlation, idx)
        return
    if args.differential:
        Sbox = getSbox()
        for idx in args.target_byte_indices:
            differentialAnalysis(Sbox, args.differential, idx)
        return
    
    traces_per_plaintext = args.traces_per_plaintext
    traces_per_byte = args.traces_per_byte 
    target_byte_indices = args.target_byte_indices
    
    if (args.average):
        powerTraceAvgs(traces_per_byte, traces_per_plaintext, target_byte_indices, partial=args.partial,port=args.port, baud=args.baud)
    else:
        if args.use_scope:
            powerTraceTimeSeriesOscilloscope(traces_per_byte, traces_per_plaintext, target_byte_indices, port=args.port, baud=args.baud, partial=args.partial, wav_start=args.wav_start, wav_stop=args.wav_stop, average_adjacent=args.average_adjacent)
        else:
            powerTraceTimeSeriesADC(traces_per_byte, traces_per_plaintext, target_byte_indices, samples=args.samples, partial=args.partial, port=args.port, baud=args.baud)


def generatePlaintexts(traces_per_byte):
    plaintexts = np.zeros((traces_per_byte, KEY_SIZE), dtype=np.uint8)
    for i in range(traces_per_byte):
        plaintexts[i] = np.random.randint(0, 256, KEY_SIZE, dtype=np.uint8)
    np.save('plaintexts.npy', plaintexts)

def correlationAnalysis(Sbox, filename, byte_idx):
    traces = np.load(filename)
    plaintexts = np.load('plaintexts.npy')
    
    rho_vs_t = np.zeros((256, traces.shape[1]),  dtype=np.float32)

    for key_guess in tqdm(range(256), desc="Performing correlation analysis"):
        hypothetical_distances = np.array([hammingWeight(Sbox[plaintext_byte^key_guess]) for plaintext_byte in plaintexts[:,byte_idx]], dtype=np.float32)
        for t in range(traces.shape[1]):
            rho_vs_t[key_guess, t] = np.corrcoef(hypothetical_distances, traces[:,t],dtype=np.float32)[0,1]
    np.save(f'cc_xor_to_sbox_diff', rho_vs_t)

    optimal_vals = np.max(np.abs(rho_vs_t),axis=1)
    best_key_guess = np.argmax(optimal_vals)
    print(f"Best key guess: {best_key_guess} with correlation {optimal_vals[best_key_guess]}")


    plt.plot(range(256), optimal_vals)
    plt.xlabel('Key Guess')
    plt.ylabel('Correlation Coefficient')
    plt.title(f'Correlation Analysis for byte {filename[filename.index(".npy")-1]}, Best Guess: {best_key_guess}')
    plt.show()

def hammingWeight(x):
    return bin(x).count('1')
def hammingDistance(a,b):
    return bin(a ^ b).count('1')

def differentialAnalysis(Sbox, filename,byte_idx):
    traces = np.load(filename)
    plaintexts = np.load('plaintexts.npy')
    key_guesses = range(256)
    differences = np.zeros((256, traces.shape[1]), dtype=np.int16)

    for key_guess in key_guesses:
        for i in range(plaintexts.shape[1]):
            if (Sbox[plaintexts[i, byte_idx] ^ key_guess] & 128):
                differences[key_guess] += traces[i]
            else:                
                differences[key_guess] -= traces[i]
            
    
    np.save(f'differential_traces_byte_idx_{byte_idx}.npy', differences)
    plt.bar(key_guesses, np.max(differences, axis=1))
    plt.xlabel('Key Guess')
    plt.ylabel('Differences')
    best_guess = np.argmax(np.abs(np.mean(differences, axis=1)))
    plt.title(f'Differential Power Analysis for {filename}, Best Guess: {best_guess}')
    print(f"Best key guess: {best_guess} with difference {np.max(differences[best_guess])}")
    plt.show()

# samples % PACKET_SIZE must equal 0 
def powerTraceTimeSeriesADC(traces_per_byte, traces_per_plaintext, target_byte_indices, port='COM10', baud=921600, partial=-1, samples=1408):

    
    def gracefulErrorHandler(ser, traces, byte_idx, trace_idx):
        
        ser.read_all()
        traces[trace_idx,:] = 0
        np.save(f'temporal_traces_byte_idx_{byte_idx}_partial_{trace_idx-1}.npy', traces)
        if trace_idx > 0:
            ser.close()       
            powerTraceTimeSeriesADC(traces_per_byte, traces_per_plaintext, target_byte_indices, port, baud, partial=trace_idx-1, samples=samples)
        
    plaintexts = np.load('plaintexts.npy')

    for byte_idx in target_byte_indices:
        if (partial >= 0):
            traces = np.load(f'temporal_traces_byte_idx_{byte_idx}_partial_{partial}.npy')
            print(f"On byte index {byte_idx}, resuming from byte value {partial}")
        else:
            traces = np.zeros((traces_per_byte,samples), dtype=np.int16)
        
        with serial.Serial(port, baud) as ser:
            while (ser.in_waiting > 0):
                ser.read_all()  # Clear the buffer before starting
            
            #                     2 3 4 5 6 7|8|9|10 11
            msg =  bytearray(b'p_\0\0\0\0\0\0\0\0\0\0')
            msg[9] = np.int8(np.log2(traces_per_plaintext,)) # accepts left shift as argument for fast arithmetic
            msg[-2:] = struct.pack('<H', samples) 
            
            starting_trace_idx = partial if (partial > 0) else 0
            print(f"Starting from trace index {starting_trace_idx}")
            for trace_idx in tqdm(range(starting_trace_idx, traces_per_byte), desc=f"Collecting traces for byte index {byte_idx}", unit="trace", leave=False):
                try:
                    msg[2:8] = bytes(plaintexts[trace_idx,:6])
                    
                    ser.write(msg)
                    while (ser.in_waiting < 2):
                        pass # to allow interrupt
                    count = -1
                    try:
                        count = struct.unpack('<H', ser.read(2))[0]
                    except struct.error:
                        print(f"Warning: Failed to unpack trace data for trace index {trace_idx}. Skipping this trace.")
                        gracefulErrorHandler(ser, traces, byte_idx, trace_idx)
                        return
                    else:
                        if (count <= 0):
                            print(f"Warning: Received an invalid trace ({count} as count byte).")
                            gracefulErrorHandler(ser, traces, byte_idx, trace_idx)                                
                        elif (count > samples*2):
                            print(f"Warning: Received more data than expected. Cutting down {count} bytes to the expected {samples*2} bytes.")
                            count = samples*2

                    pkt_end = 0 
                    for pkt_start in range(0, count>>1, PACKET_SIZE>>1):
                        ser.write(b'A') # Acknowledge the received packet to continue the data transmission
                        time.sleep(.0075) # experimentally necessary for stability
                        while (ser.in_waiting < PACKET_SIZE):
                            pass # to allow interrupt
                        raw_data = ser.read(PACKET_SIZE) 
                        pkt_end += PACKET_SIZE >> 1
                        try:
                            trace = np.array(struct.unpack('<'+'H'*(PACKET_SIZE>>1), raw_data))
                            if (trace > 4095).any() or (trace == 0).any(): # sanity check for invalid ADC values
                                print(f"Warning: Received trace data contains invalid ADC values. Trace Index {trace_idx}, Start: {pkt_start}, Trace: {trace}.")
                                gracefulErrorHandler(ser, traces, byte_idx, trace_idx)
                        except struct.error:
                            print(f"Warning: Failed to unpack trace data for trace index {trace_idx}. Raw data: {raw_data}. Skipping this trace.")
                            gracefulErrorHandler(ser, traces, byte_idx, trace_idx)
                            return
                        else: 
                            traces[trace_idx, pkt_start:pkt_end] = trace
                            # print(f"Received trace packet for byte value {byte_val}, trace index {trace_idx}, packet start {pkt_start}, packet end {pkt_end}, with values: {trace}")    
                except KeyboardInterrupt as e:
                    val = 'v'
                    print(f"{e}\nData collection interrupted by user.")
                    traceback.print_exc()
                    while val == 'v':
                        val = input("Type 'v' to see current values, reset the device and type \'c\' to continue, or enter any other key to exit and save collected traces...")
                        if val == 'v':
                            print(f"Current values: {traces[:trace_idx,:]}")
                        elif val == 'c':
                            print("Resuming data collection...")
                            gracefulErrorHandler(ser, traces, trace_idx)
                            return
                        else:
                            np.save(f'temporal_traces_byte_idx_{byte_idx}_partial_{trace_idx-1}.npy', traces)
                            exit()
                            
    np.save(f'temporal_traces_byte_idx_{byte_idx}.npy', traces)

def powerTraceTimeSeriesOscilloscope(traces_per_byte, traces_per_plaintext, target_byte_indices, port='COM3', baud=921600, partial=-1, wav_start=1e5, wav_stop=3.75e5, average_adjacent=5):

    
    def gracefulErrorHandler(ser, traces, byte_idx, trace_idx):
        
        ser.read_all()
        traces[trace_idx,:] = 0
        np.save(f'temporal_traces_byte_idx_{byte_idx}_partial_{trace_idx-1}.npy', traces)
        if trace_idx > 0:
            ser.close()       
            powerTraceTimeSeriesOscilloscope(traces_per_byte, traces_per_plaintext, target_byte_indices, port, baud, partial=trace_idx-1, wav_start=wav_start, wav_stop=wav_stop, average_adjacent=average_adjacent)
        
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
    print(f"Acquiring from scope at {scope.query(':ACQ:SRAT?')} Sa/s")

    #scope.write(":TIMebase:DELay:OFFSet -1")

    #scope.write(":TRIG:MODE EDGE")
    #scope.write(":TRIG:EDGE:SOUR CHAN2")
    #scope.write(":TRIG:EDGE:SLOP POS")
    #scope.write(":TRIG:EDGE:LEV 0.5")   

    scope.write(":WAV:SOUR CHAN1")
    scope.write(":WAV:MODE RAW")
    scope.write(":WAV:FORM BYTE")

    scope.write(f":WAV:STAR {wav_start}")
    scope.write(f":WAV:STOP {wav_stop}")

    averaged_trace_size = int(wav_stop-wav_start)//(1 if average_adjacent <= 0 else average_adjacent)

    plaintexts = np.load('plaintexts.npy')
    
    for byte_idx in target_byte_indices:
        if (partial >= 0):
            traces = np.load(f'temporal_traces_byte_idx_{byte_idx}_partial_{partial}.npy')
            print(f"On byte index {byte_idx}, resuming from byte value {partial}")
        else:
            traces = np.zeros((traces_per_byte, averaged_trace_size), dtype=np.int8)
        
        with serial.Serial(port, baud) as ser:
            while (ser.in_waiting > 0):
                ser.read_all()  # Clear the buffer before starting
            
            #                     2 3 4 5 6 7|8|9|10 11
            msg =  bytearray(b'p_\0\0\0\0\0\0\0\0\0\0')
            msg[9] = np.int8(np.log2(traces_per_plaintext,)) # accepts left shift as argument for fast arithmetic
            msg[-2:] = struct.pack('<H', 2048) # 2048 is dummy value since we will be acquiring the trace directly from the oscilloscope instead of the ADC on the device 
            
            starting_trace_idx = partial if (partial > 0) else 0
            print(f"Starting from trace index {starting_trace_idx}")
            for trace_idx in tqdm(range(starting_trace_idx, traces_per_byte), desc=f"Collecting traces for byte index {byte_idx}", unit="trace", leave=False):
                try:
                    msg[2:8] = bytes(plaintexts[trace_idx,:6])
                    
                    ser.write(msg)
                    while (ser.in_waiting < 2):
                        pass # to allow interrupt
                    while (ser.in_waiting):
                        ser.read_all();
                        ser.write(ACK);
                        time.sleep(.01)
                     
                    trigger_status = scope.query(":TRIG:STAT?");
                    while (trigger_status[0] != "S"):
                        print(f"STATUS: {trigger_status}")
                        trigger_status = scope.query(":TRIG:STAT?");

                    data = np.array(scope.query_binary_values(":WAV:DATA?", datatype='B'))
                    
                    if (len(data) > 0):
                        trace = np.zeros((averaged_trace_size,), dtype=np.int16)
                        for i in range(average_adjacent): # average across adjacent samples
                            trace += data[i::average_adjacent][:averaged_trace_size] # cut off extras
                        trace //= (average_adjacent if average_adjacent > 0 else 1)
                        traces[trace_idx,:] = trace
                    else:
                        print(f"No data saved on byte index {byte_idx}, trace index {trace_idx}")
                        gracefulErrorHandler(ser, traces, byte_idx, trace_idx)
                        return

                    scope.write(":SING")
                    
                    # wait for it reset
                    status = scope.query("*OPC?");
                    while ( status[0] != '1'):
                        #print(f"STATUS: {status}")
                        status = scope.query("*OPC?");


                    # print(f"Received trace packet for byte value {byte_val}, trace index {trace_idx}, packet start {pkt_start}, packet end {pkt_end}, with values: {trace}")    
                except KeyboardInterrupt as e:
                    val = 'v'
                    print(f"{e}\nData collection interrupted by user.")
                    traceback.print_exc()
                    while val == 'v':
                        val = input("Type 'v' to see current values, reset the device and type \'c\' to continue, or enter any other key to exit and save collected traces...")
                        if val == 'v':
                            print(f"Current values: {traces[:trace_idx,:]}")
                        elif val == 'c':
                            print("Resuming data collection...")
                            gracefulErrorHandler(ser, traces, trace_idx)
                            return
                        else:
                            np.save(f'temporal_traces_byte_idx_{byte_idx}_partial_{trace_idx-1}.npy', traces)
                            exit()
                            
    np.save(f'temporal_traces_byte_idx_{byte_idx}.npy', traces)


def powerTraceAvgs(traces_per_value, traces_per_plaintext, target_byte_indices, port='COM10', baud=115200, partial=-1):

    
    def gracefulErrorHandler(ser, traces, byte_val, trace_idx):
        
        ser.read_all()  # Clear the buffer
        traces[byte_val, trace_idx] = traces[byte_val, max(0, trace_idx-1)]

    for byte_idx in target_byte_indices:
        if (partial >= 0):
            traces = np.load(f'traces_byte_idx_{byte_idx}_partial_{partial}.npy')
            print(f"On byte index {byte_idx}, resuming from byte value {partial}")
            print(f"Current Averages: {np.average(traces[0:partial,:], axis=1)}")
        else:
            traces = np.zeros((256,traces_per_value), dtype=np.float32)
        with serial.Serial(port, baud) as ser:
            while (ser.in_waiting > 0):
                ser.read_all()  # Clear the buffer before starting
            for byte_val in tqdm(range(partial+1, 256), desc=f"Collecting traces for key index {byte_idx}", unit="byte"):
                msg =  bytearray('p_\0\0\0\0\0\0\0\0'.encode())
                msg[-1] = traces_per_plaintext

                for trace_idx in tqdm(range(traces_per_value), desc=f"Collecting traces for byte value {byte_val}", unit="trace", leave=False):
                    try:
                        msg[2:8] = random.randbytes(6)
                        msg[2+byte_idx] = byte_val
                        # print(msg)
                        ser.write(msg)
                        while (ser.in_waiting < 8):
                            pass
                        
                        raw_data = ser.read_all()
                        
                        try:
                            trace = struct.unpack('<II', raw_data)
                            #print(trace)
                        except struct.error:
                            print(f"Warning: Failed to unpack trace data for byte value {byte_val}, trace index {trace_idx}. Raw data: {raw_data}. Skipping this trace.")
                            gracefulErrorHandler(ser, traces, byte_val, trace_idx)
                            continue
                        else: 
                            if (trace[0] == 0):
                                print(f"Warning: Received an invalid trace. Please reset the device.")
                                input("Press Enter after resetting the device...")
                                gracefulErrorHandler(ser, traces, byte_val, trace_idx)
                            else:
                                traces[byte_val, trace_idx] = 4095 - trace[1]/trace[0]
                            # print(traces[byte_val, trace_idx])

                    except KeyboardInterrupt as e:
                        val = 'a'
                        print(f"{e}\nData collection interrupted by user.")
                        traceback.print_exc()
                        while val == 'a':
                            val = input("Type 'a' to see current average, reset the device and type \'c\' to continue, or enter any other key to exit and save collected traces...")
                            if val == 'a':
                                print(f"Current averages: {np.average(traces[:byte_val,:np.argmin(traces[byte_val,:])], axis=1)}")
                            elif val == 'c':
                                gracefulErrorHandler(ser, traces, byte_val, trace_idx)
                                print("Continuing data collection...")
                            else:
                                np.save(f'traces_byte_idx_{byte_idx}_partial_{byte_val-1}.npy', traces)
                                exit()

        print(f"Traces for byte index {byte_idx} collected.")
        avgs = np.average(traces, axis=1)
        print(avgs)
        np.save(f'traces_byte_idx_{byte_idx}.npy', avgs)

def testFunctionality(port='COM10', baud=921600):
    with serial.Serial(port, baud) as ser:
        while ser.in_waiting > 0:
            print(ser.read_all())
            time.sleep(0.01)
        '''
        ser.write(b'e')
        ser.write(b's')
        time.sleep(0.1)
        while ser.in_waiting > 0:
            print(ser.read_all())
            time.sleep(0.01)
        ser.write(b'e')
        '''
        ser.write(b'p')
        time.sleep(.1)
        while ser.in_waiting > 0:
            print(ser.read_all())
            time.sleep(0.01)

def getSbox():
    t = [0]*256
    x = 1
    for i in range(256): 
        t[i] = x
        x ^= (x << 1) ^ ((x >> 7) * 0x11B)
    
    S = [0]*256
    S[0] = 0x63
    for i in range(255): 
        x = t[255 - i]
        x |= x << 8
        x ^= (x >> 4) ^ (x >> 5) ^ (x >> 6) ^ (x >> 7)
        S[t[i]] = (x ^ 0x63) & 0xFF

    return S

def getInvSbox(S):
    invS = np.zeros(256, dtype=np.uint8)
    for i in range(256):
        invS[S[i]] = i
    return invS
    
    

if __name__ == "__main__":
    main()