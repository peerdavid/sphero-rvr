


#
# Create different sounds with a buzzer
#
# Credit for star wars song: https://gist.github.com/mandyRae/459ae289cdfcf6d98a6b

import time
import numpy as np
import RPi.GPIO as GPIO

sound_pin = 12
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(sound_pin, GPIO.OUT)
GPIO.output(sound_pin, 0)
tone1 = GPIO.PWM(sound_pin, 100)
tone1.start(50)

c = [32, 65, 131, 262, 523]
db= [34, 69, 139, 277, 554]
d = [36, 73, 147, 294, 587]
eb= [37, 78, 156, 311, 622]
e = [41, 82, 165, 330, 659]
f = [43, 87, 175, 349, 698]
gb= [46, 92, 185, 370, 740]
g = [49, 98, 196, 392, 784]
ab= [52, 104, 208, 415, 831]
a = [55, 110, 220, 440, 880]
bb= [58, 117, 223, 466, 932]
b = [61, 123, 246, 492, 984]

cmajor = [c, d, e, f, g, a, b]
aminor = [a, b, c, d, e, f, g]

o=2
starwars_notes = [c[1+o], g[1+o], f[1+o], e[1+o], d[1+o], c[2+o], g[1+o], f[1+o], e[1+o], d[1+o], c[2+o], g[1+o], 
              f[1+o], e[1+o], f[1+o], d[1+o]]
starwars_beats = [4,4,1,1,1,4,4,1,1,1,4,4,1,1,1,4]



def playScale(scale, pause):
    '''
    scale: scale name to be played
    pause: pause between each notes played
    
    This function plays the given scale in every available octave
    I used this to test what was audible on the buzzer
    '''
    for i in range(0, 5):
        for note in scale:
            tone1.ChangeFrequency(note[i])
            time.sleep(pause)
    tone1.stop()


def playSong(songnotes, songbeats, tempo):
    '''
    songnotes: list of the melodies notes
    songbeats: list of melodies beat times
    tempo: speed of song, this is not traditional tempo in bpm like on a metronome, 
        but more like a multiplier for whatever the notes are so a tempo value of 2 
        make it play twice as fast. Adjust this by ear.
        
    This function plays the melody, simply by iterating through the list. 
    '''
    tone1.ChangeDutyCycle(50)
    for i in range(0, len(songnotes)):
        tone1.ChangeFrequency(songnotes[i])
        time.sleep(songbeats[i]*tempo)
    tone1.ChangeDutyCycle(0)


def buzzPwm(duration, freq):
    tone1.ChangeDutyCycle(50)
    tone1.ChangeFrequency(freq)
    time.sleep(float(duration) / 1000)
    tone1.ChangeDutyCycle(0)


def main():
    #for i in range(1, 10000, 100):
    #    buzzPwm(20, i)
    pause = 0.05

    buzzPwm(100, 10000)
    time.sleep(pause)
    buzzPwm(100, 10000)
    time.sleep(pause)
    buzzPwm(100, 5000)
    time.sleep(pause)
    buzzPwm(100, 10000)
    time.sleep(2)
    # time.sleep(pause)
    # buzzPwm(500, 400)
    # time.sleep(pause)
    # buzzPwm(500, 800)
    # time.sleep(pause)
    # buzzPwm(500, 400)
    
    playSong(starwars_notes, starwars_beats, 0.2)
    

if __name__ == '__main__':
    main()

