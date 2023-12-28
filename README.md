# Verilog Piano

A piano written in Verilog for Intel's DE1-SOC FPGA board.

The purpose of the project is to replicate the functionality of a standard type of music software used for recording and playing back music.

A user should be able to record a set of notes by pressing keys on the keyboard and play back multiple recordings simultaneously.

## Usage

If you would like to record a sound track, you must first select the switch you would like to use to access that part of the memory in the FPGA board. Once you have selected the switch, it must be turned on before pressing the record button. After pressing the record button, you have approximately 8 seconds to record a sequence of notes.

Similarly, if you would like to play back a sound track, you must first select the switch associated to the sound track that is stored in memory. Then after turning on that switch, you can press the playback button and the sound track will be played back in the same sequence as it was recorded. You can also play back both sound tracks simultaneously by turning on both switches and then pressing the playback button.

#### Toggles

- KEY[0] is for reset
- KEY[1] is for record
- KEY[2] is for playback
- SW[0] is used for accessing the first sound track
- SW[1] is used for accessing the second sound track

## Features

- A flashing LED meant to be used as a metronome
- Hex displays for showing the current note that is being played
