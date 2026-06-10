#!/usr/bin/env python3
from gpiozero import TonalBuzzer
from time import sleep

# Initialize a TonalBuzzer connected to GPIO18 (BCM)
# tb = TonalBuzzer(18) 

# Define a musical tune as a sequence of notes and durations.
SONG =	[
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4]
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
        play(SONG)  # Execute the play function to start playing the tune.

    except KeyboardInterrupt:
        # Handle KeyboardInterrupt for graceful termination
        pass
