#!/usr/bin/env python3

import socket
import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt
import argparse

# Connection parameters
HOST = '10.42.0.199'
PORT = 5000
SAMPLE_RATE = 50  # Hz
NUM_SAMPLES = 1024

def test_fft():
    # Generate 20 Hz sine wave
    t = np.arange(NUM_SAMPLES) / SAMPLE_RATE
    freq = 20  # Hz
    amplitude = 1.0
    test_signal = amplitude * np.sin(2 * np.pi * freq * t)
    
    # Calculate FFT
    fft_result = fft(test_signal)
    freqs = fftfreq(NUM_SAMPLES, 1/SAMPLE_RATE)
    
    # Get positive frequencies only
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    fft_result = 2.0/NUM_SAMPLES * np.abs(fft_result[pos_mask])
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Time domain
    ax1.plot(t, test_signal)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Amplitude')
    ax1.set_title('20 Hz Test Signal')
    ax1.grid(True)
    
    # Frequency domain
    ax2.plot(freqs, fft_result)
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Magnitude')
    ax2.set_title('FFT of Test Signal')
    ax2.grid(True)
    ax2.set_xlim(0, 25)  # Limit x-axis to better see the peak
    
    plt.tight_layout()
    plt.show()

def collect_samples():
    x_data = []
    y_data = []
    z_data = []
    
    # Connect to TCP server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")
        print(f"Collecting {NUM_SAMPLES} samples...")
        
        while len(x_data) < NUM_SAMPLES:
            data = s.recv(1024).decode()
            samples = data.strip().split('\n')
            
            for sample in samples:
                try:
                    # Parse format "X:  0.123  Y:  0.456  Z:  0.789"
                    parts = sample.split()
                    if len(parts) >= 6:  # Make sure we have all components
                        x = float(parts[1])
                        y = float(parts[3])
                        z = float(parts[5]) - 1.0  # Subtract gravity offset
                        x_data.append(x)
                        y_data.append(y)
                        z_data.append(z)
                        print(f"Samples collected: {len(x_data)}\r", end='')
                        
                        if len(x_data) >= NUM_SAMPLES:
                            return np.array(x_data[:NUM_SAMPLES]), \
                                   np.array(y_data[:NUM_SAMPLES]), \
                                   np.array(z_data[:NUM_SAMPLES])
                except (ValueError, IndexError):
                    continue
    
    return np.array(x_data), np.array(y_data), np.array(z_data)

def calculate_fft(data):
    # Calculate FFT exactly like in test_fft
    fft_result = fft(data)
    freqs = fftfreq(NUM_SAMPLES, 1/SAMPLE_RATE)
    
    # Get positive frequencies only
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    fft_result = 2.0/NUM_SAMPLES * np.abs(fft_result[pos_mask])
    return freqs, fft_result

def plot_results(x_data, y_data, z_data):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Time domain plot
    time = np.arange(NUM_SAMPLES) / SAMPLE_RATE
    ax1.plot(time, x_data, label='X')
    ax1.plot(time, y_data, label='Y')
    ax1.plot(time, z_data, label='Z')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Acceleration (g)')
    ax1.set_title('Time Domain')
    ax1.legend()
    ax1.grid(True)
    
    # Frequency domain plot
    freqs, x_fft = calculate_fft(x_data)
    _, y_fft = calculate_fft(y_data)
    _, z_fft = calculate_fft(z_data)
    
    ax2.plot(freqs, x_fft, label='X')
    ax2.plot(freqs, y_fft, label='Y')
    ax2.plot(freqs, z_fft, label='Z')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylabel('Magnitude')
    ax2.set_title('Frequency Domain (FFT)')
    ax2.legend()
    ax2.grid(True)
    ax2.set_xlim(0, 25)  # Added to match test_fft
    
    plt.tight_layout()
    plt.show()

def read_from_file(filename):
    x_data = []
    y_data = []
    z_data = []
    
    with open(filename, 'r') as f:
        for line in f:
            try:
                # Parse format "X:  0.123  Y:  0.456  Z:  0.789"
                parts = line.split()
                if len(parts) >= 6:  # Make sure we have all components
                    x = float(parts[1])
                    y = float(parts[3])
                    z = float(parts[5]) - 1.0  # Subtract gravity offset
                    x_data.append(x)
                    y_data.append(y)
                    z_data.append(z)
                    print(f"Samples read: {len(x_data)}\r", end='')
                    
                    if len(x_data) >= NUM_SAMPLES:
                        print("\nReached 1024 samples, truncating remaining data")
                        return np.array(x_data[:NUM_SAMPLES]), \
                               np.array(y_data[:NUM_SAMPLES]), \
                               np.array(z_data[:NUM_SAMPLES])
            except (ValueError, IndexError):
                continue
    
    if len(x_data) < NUM_SAMPLES:
        print(f"\nWarning: Only {len(x_data)} samples found in file")
    
    return np.array(x_data), np.array(y_data), np.array(z_data)

def main():
    parser = argparse.ArgumentParser(description='IMU Data FFT Analysis')
    parser.add_argument('--file', '-f', help='Read data from file instead of TCP')
    args = parser.parse_args()

    try:
        # Run FFT test first
        # print("Running FFT test with 20 Hz sine wave...")
        # test_fft()
        
        if args.file:
            # Read from file
            print(f"\nReading data from file {args.file}...")
            x_data, y_data, z_data = read_from_file(args.file)
        else:
            # Collect IMU data via TCP
            print("\nCollecting IMU data...")
            x_data, y_data, z_data = collect_samples()
            
        print("\nData collection complete!")
        
        # Plot results
        plot_results(x_data, y_data, z_data)
        
    except ConnectionRefusedError:
        print("Error: Could not connect to the IMU server")
    except FileNotFoundError:
        print(f"Error: File {args.file} not found")
    except KeyboardInterrupt:
        print("\nData collection interrupted")
    except Exception as e:
        print(f"Error: {str(e)}")

if __name__ == "__main__":
    main()

# Required packages:
# numpy
# scipy
# matplotlib 