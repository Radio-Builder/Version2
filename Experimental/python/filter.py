import matplotlib.pyplot as plt
from scipy.signal import firwin, freqz
import numpy as np

# Bandpass FIR filter design using firwin
def bandpass_firwin(ntaps, lowcut, highcut, fs, window='hamming'):
    # Design the filter using firwin with pass_zero=False for band-pass
    taps = firwin(ntaps, [lowcut, highcut], fs=fs, pass_zero=False,
                  window=window, scale=True)  # Added scale=True for proper gain
    return taps


if __name__ == "__main__":
    # Sample rate and desired cutoff frequencies (in Hz)
    fs = 22050.0         # Sampling rate: 44.1 kHz
    lowcut = 690.0       # Lower cutoff frequency: 600 Hz
    highcut = 710.0      # Upper cutoff frequency: 800 Hz
    ntaps = 1024        # Number of filter taps

    # Design the filter
    taps_hamming = bandpass_firwin(ntaps, lowcut, highcut, fs=fs)

#    print(taps_hamming)
    
    with open("coeff.txt", "w") as f:
        f.write("float fir_coefficients[NUM_TAPS] = {\n")
        
        for i, tap in enumerate(taps_hamming):
            # Print 10 taps per line for better readability
            if i % 10 == 0 and i != 0:
                f.write("\n")
            f.write(f"    {tap:.8e}, ")
        
        f.write("\n};\n")

    print("Filter coefficients have been written to coeff.txt")

    # Plot the frequency responses of the filter in dB
    plt.figure(figsize=(12, 6))
    plt.clf()

    # Plot the frequency response of the filter
    w, h = freqz(taps_hamming, 1, worN=2000, fs=fs)
    
    # Convert magnitude response to dB
    h_dB = 20 * np.log10(abs(h))

    # Plotting the response in dB
    plt.plot(w, h_dB, label="Hamming window")

    # Set the frequency limits for the x-axis and amplitude for the y-axis
    plt.xlim(0, 5000.0)      # Display frequencies from 0 to 5 kHz
    plt.ylim(-100, 5)        # Set y-axis limits to show dB range
    plt.grid(True)

    # First plot the desired ideal response as a green(ish) rectangle
    rect = plt.Rectangle((lowcut, -100), highcut - lowcut, 105.0,
                         facecolor="#60ff60", alpha=0.2)
    plt.gca().add_patch(rect)

    plt.legend(shadow=True, framealpha=1)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Gain (dB)')
    plt.title(f'Frequency response of the band-pass FIR filter in dB with {ntaps} taps')

    plt.show()

