import pyaudio
import wave
import numpy as np

SOUND_SPEED = 340.0
MIC_DISTANCE_6 = 0.09218
MAX_TDOA_6 = MIC_DISTANCE_6 / float(SOUND_SPEED)

RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 8
RESPEAKER_WIDTH = 2

# run getDeviceInfo.py to get index
RESPEAKER_INDEX = 2  # refer to input device id
CHUNK = 1024
RECORD_SECONDS = 1
WAVE_OUTPUT_FILENAME_0 = "output_0.wav"
WAVE_OUTPUT_FILENAME_1 = "output_1.wav"
WAVE_OUTPUT_FILENAME_2 = "output_2.wav"
WAVE_OUTPUT_FILENAME_3 = "output_3.wav"
WAVE_OUTPUT_FILENAME_4 = "output_4.wav"
WAVE_OUTPUT_FILENAME_5 = "output_5.wav"
WAVE_OUTPUT_FILENAME_6 = "output_6.wav"
WAVE_OUTPUT_FILENAME_7 = "output_7.wav"

def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=1):
    '''
    This function computes the offset between the signal sig and the reference signal refsig
    using the Generalized Cross Correlation - Phase Transform (GCC-PHAT)method.
    '''
    
    sig = sig.reshape(-1)
    refsig = refsig.reshape(-1)

    # make sure the length for the FFT is larger or equal than len(sig) + len(refsig)
    n = sig.shape[0] + refsig.shape[0]
    
    print(sig.shape)

    # Generalized Cross Correlation Phase Transform
    SIG = np.fft.rfft(sig, n=n)
    REFSIG = np.fft.rfft(refsig, n=n)
    R = SIG * np.conj(REFSIG)

    cc = np.fft.irfft(R / np.abs(R), n=(interp * n))
    
    print(cc.shape)

    max_shift = int(interp * n / 2)
    if max_tau:
        max_shift = np.minimum(int(interp * fs * max_tau), max_shift)

    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
    
    print('argmax(np.abs(cc)): '+str(np.argmax(np.abs(cc))))

    # find max cross correlation index
    shift = np.argmax(np.abs(cc)) - max_shift
    
    print('shift: '+str(shift))

    tau = shift / float(interp * fs)

    return tau#, np.max(cc)

p = pyaudio.PyAudio()

#count = 3

#for j in range(0, count):

stream = p.open(
rate=RESPEAKER_RATE,
format=p.get_format_from_width(RESPEAKER_WIDTH),
channels=RESPEAKER_CHANNELS,
input=True,
input_device_index=RESPEAKER_INDEX,)

print("* recording")


frames_0 = []
frames_1 = [] 
frames_2 = [] 
frames_3 = [] 
frames_4 = [] 
frames_5 = [] 
frames_6 = [] 
frames_7 = []

frames0 = [] 
frames1 = [] 
frames2 = [] 
frames3 = [] 
frames4 = [] 
frames5 = [] 
frames6 = [] 
frames7 = []  

for i in range(0, int(RESPEAKER_RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    # extract channel 0 data from 8 channels, if you want to extract channel 1, please change to [1::8]
    a = np.fromstring(data,dtype=np.int16)[0::8]
    b = np.fromstring(data,dtype=np.int16)[1::8]
    c = np.fromstring(data,dtype=np.int16)[2::8]
    d = np.fromstring(data,dtype=np.int16)[3::8]
    e = np.fromstring(data,dtype=np.int16)[4::8]
    f = np.fromstring(data,dtype=np.int16)[5::8]
    g = np.fromstring(data,dtype=np.int16)[6::8]
    h = np.fromstring(data,dtype=np.int16)[7::8]
    
    # a[np.abs(a)<3277]=0
    # b[np.abs(b)<3277]=0
    # c[np.abs(c)<3277]=0
    # d[np.abs(d)<3277]=0
    # e[np.abs(e)<3277]=0
    # f[np.abs(f)<3277]=0

    frames_0.append(a)
    frames_1.append(b)
    frames_2.append(c)
    frames_3.append(d)
    frames_4.append(e)
    frames_5.append(f)
    frames_6.append(g)
    frames_7.append(h)
    
    frames0.append(a.tostring())
    frames1.append(b.tostring())
    frames2.append(c.tostring())
    frames3.append(d.tostring())
    frames4.append(e.tostring())
    frames5.append(f.tostring())
    frames6.append(g.tostring())
    frames7.append(h.tostring())

print("* done recording")

print('tau pair 1: '+str(gcc_phat(np.array(frames_0), np.array(frames_3),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)))
print('tau pair 2: '+str(gcc_phat(np.array(frames_1), np.array(frames_4),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)))
print('tau pair 3: '+str(gcc_phat(np.array(frames_2), np.array(frames_5),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)))

#p1 = gcc_phat(np.array(frames_0), np.array(frames_3),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)
# p2 = gcc_phat(np.array(frames_1), np.array(frames_4),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)
# p3 = gcc_phat(np.array(frames_2), np.array(frames_5),fs=RESPEAKER_RATE, max_tau=MAX_TDOA_6, interp=1)

# ind = np.argmax([p1, p2, p3])



stream.stop_stream()
stream.close()
p.terminate()


##

wf = wave.open(WAVE_OUTPUT_FILENAME_0, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames0))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_1, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames1))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_2, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames2))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_3, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames3))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_4, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames4))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_5, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames5))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_6, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames6))
wf.close()

wf = wave.open(WAVE_OUTPUT_FILENAME_7, 'wb')
wf.setnchannels(1)
wf.setsampwidth(p.get_sample_size(p.get_format_from_width(RESPEAKER_WIDTH)))
wf.setframerate(RESPEAKER_RATE)
wf.writeframes(b''.join(frames7))
wf.close()

