import serial
import struct
import matplotlib.pyplot as plt
import numpy as np
import time
import argparse
import random
import traceback
from tqdm import tqdm

PACKET_SIZE = 128
KEY_SIZE = 16

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
    parser.add_argument("target_byte_indices", nargs='*', type=int, default=[0], help='Indices of the target bytes to analyze (default: [0])')

    args = parser.parse_args()

    if args.test:
        testFunctionality()
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
        powerTraceAvgs(traces_per_byte, traces_per_plaintext, target_byte_indices, partial=args.partial)
    else:
        powerTraceTimeSeries(traces_per_byte, traces_per_plaintext, target_byte_indices, samples=args.samples, partial=args.partial)


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
def powerTraceTimeSeries(traces_per_byte, traces_per_plaintext, target_byte_indices, port='COM10', baud=921600, partial=-1, samples=1408):

    
    def gracefulErrorHandler(ser, traces, byte_idx, trace_idx):
        
        ser.read_all()
        traces[trace_idx,:] = 0
        np.save(f'temporal_traces_byte_idx_{byte_idx}_partial_{trace_idx-1}.npy', traces)
        if trace_idx > 0:
            ser.close()       
            powerTraceTimeSeries(traces_per_byte, traces_per_plaintext, target_byte_indices, port, baud, partial=trace_idx-1, samples=samples)
        
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

def testFunctionality():
    with serial.Serial('COM10',921600) as ser:
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
        ser.write(b'p_\0\2\0\0\0\0\0\2')
        time.sleep(0.1)
        while ser.in_waiting < 8:
            pass
        raw_data = ser.read_all()
        v = struct.unpack('<II', raw_data)
        print(f"Received trace: {v}, with average power {4095 - v[1]/v[0] if v[0] > 0 else 0}")

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