#!/usr/bin/env python3
import rospy

# the following line depends upon the
# type of message you are trying to publish
from std_msgs.msg import String
# from sound_pkg.msg import Sound
from std_msgs.msg import Int32

from tuning import Tuning
import usb.core
import usb.util
import time

import pyaudio
import wave
import numpy as np

RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6 
RESPEAKER_WIDTH = 2
RESPEAKER_INDEX = 0
CHUNK = 1024
RECORD_SECONDS = 1

def volume_level(sig):
    # Reshape array
    sig = sig.reshape(-1)
    
    # Convert elements to floats 
    sig_float = sig.astype(np.float32)
    sig = sig_float/2**15
    
    # Calculate energy of signal
    vol_sig = np.sum(np.square(sig))
    
    return vol_sig

def max_volume(vol1, vol2, vol3, vol4, vol5):
    return max(vol1, vol2, vol3, vol4, vol5)

def max_freq_component(sig):
    # Reshape array
    sig = sig.reshape(-1)
    
    sample_interval = 1/16000
    magnitudes = np.abs(np.fft.rfft(sig))
    frequencies = np.fft.fftfreq(len(sig), d=sample_interval)
    
    values = len(frequencies)//2
    frequencies = frequencies[:values]
    magnitudes = magnitudes[0:len(frequencies)]
    
    max_mag = np.argmax(magnitudes)
    max_freq = frequencies[max_mag]
    
    return max_freq

def sound_classification(freq):
    # fire_alarm: 6280-6290
    # baby_crying: changes too much
    # door_bell:  735 - 740 
    
    sound = "none"
    
    if freq > 6280 and freq < 6290:
        sound = "alarm"
    elif freq > 735 and freq < 740:
        sound = "door_bell"
    elif freq > 7390 and freq < 7420:
        sound = "phone_ring"
        
    return sound

p = pyaudio.PyAudio()

stream = p.open(
    rate=RESPEAKER_RATE,
    format=p.get_format_from_width(RESPEAKER_WIDTH),
    channels=RESPEAKER_CHANNELS,
    input=True,
    input_device_index=RESPEAKER_INDEX,)

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

_sound_type = ""
_volume = 0
_angle = 0

def publisher():
    pub = rospy.Publisher('sound', String, queue_size=10)
    pub_angle = rospy.Publisher('sound_angle', Int32, queue_size=10)
    rospy.init_node('pi', anonymous=True)
    rate = rospy.Rate(10)

    if dev:
        Mic_tuning = Tuning(dev)
        print(Mic_tuning.direction)
        
        while not rospy.is_shutdown():
            frames_0 = []
            frames_1 = [] 
            frames_2 = [] 
            frames_3 = [] 
            frames_4 = [] 

            for i in range(0, int(RESPEAKER_RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK, exception_on_overflow=False)
                a = np.fromstring(data,dtype=np.int16)[0::6]
                b = np.fromstring(data,dtype=np.int16)[1::6]
                c = np.fromstring(data,dtype=np.int16)[2::6]
                d = np.fromstring(data,dtype=np.int16)[3::6]
                e = np.fromstring(data,dtype=np.int16)[4::6]
                
                frames_0.append(a)
                frames_1.append(b)
                frames_2.append(c)
                frames_3.append(d)
                frames_4.append(e)
            
            v1 = volume_level(np.array(frames_0))
            v2 = volume_level(np.array(frames_1))
            v3 = volume_level(np.array(frames_2))
            v4 = volume_level(np.array(frames_3))
            v5 = volume_level(np.array(frames_4))
            
            freq = max_freq_component(np.array(frames_0))
            
            _sound_type = sound_classification(freq)
            _volume = max_volume(v1, v2, v3, v4, v5)
            _angle = Mic_tuning.direction
            # print("Volume: " + str(_volume), " DOA:" + str(_angle))
            
            msg = dict(sound_type = _sound_type, volume = _volume, angle = _angle)
            print(msg)
            msg = str(msg)
            print(msg)
            rospy.loginfo(msg)
            #print(type(_angle))
            if _sound_type == "alarm" or _sound_type == "door_bell" or _sound_type == "phone_ring": 
                pub.publish(msg)
                pub_angle.publish(_angle)
            rospy.loginfo(msg)


if __name__ == '__main__':
    # it is good practice to maintain
    # a 'try'-'except' clause
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
