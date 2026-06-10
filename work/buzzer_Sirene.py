#!/usr/bin/env python3
from gpiozero import TonalBuzzer
from time import sleep

# Initialize a TonalBuzzer connected to GPIO18 (BCM)
tb = TonalBuzzer(18) 

# Define a musical tune as a sequence of notes and durations.
POLICE =	[
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4]
]
MII = [
    # 1ère phrase : tut, tut, tut, tut, tut... tut tut tuuut !
    ["F#4", 0.25],[None, 0.25], ["A4", 0.25], ["C#5", 0.25], [None, 0.25],
    ["A4", 0.25],[None, 0.25],  ["F#4", 0.25], ["D4", 0.15],[None, 0.1],["D4", 0.15],
    [None, 0.1],["D4", 0.15],[None, 0.1],
    [None, 1],

    # 2ème phrase
    ["C#4", 0.25], ["D4", 0.25], ["F#4", 0.25], ["A4", 0.25],
    ["C#5", 0.125],[None, 0.125] ["A4", 0.2], ["F#4", 0.2], ["E5", 0.4],
    ["D#5", 0.6],
    [None, 0.2],

    # 3ème phrase (la grande descente)
    ["D4", 0.2], ["G#4", 0.2], ["C#5", 0.2], ["F#5", 0.2],
    ["C#5", 0.2], ["G#4", 0.2], ["C#5", 0.2], ["G4", 0.2],
    ["F#4", 0.2], ["C#5", 0.2], ["C5", 0.2], ["C5", 0.2],
    ["C5", 0.6],
    [None, 0.4]
]

def play(tune):
    """
    Play a musical tune using the buzzer.
    :param tune: List of tuples (note, duration), 
    where each tuple represents a note and its duration.
    """
    for note, duration in tune:
        print(note)  # Output the current note being played
        tb.play(note)  # Play the note on the buzzer
        sleep(float(duration))  # Delay for the duration of the note
    tb.stop()  # Stop playing after the tune is complete

if __name__ == "__main__":
    try:
        play(MII)  # Execute the play function to start playing the tune.

    except KeyboardInterrupt:
        # Handle KeyboardInterrupt for graceful termination
        pass
