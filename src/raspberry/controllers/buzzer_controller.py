from machine import Pin, PWM
from parameters.parameters import params
import asyncio

# Notes definition (same as Arduino's frequency table)
NOTE_FREQ = {
    "REST": 0,
    "NOTE_B0": 31,
    "NOTE_C1": 33,
    "NOTE_CS1": 35,
    "NOTE_D1": 37,
    "NOTE_DS1": 39,
    "NOTE_E1": 41,
    "NOTE_F1": 44,
    "NOTE_FS1": 46,
    "NOTE_G1": 49,
    "NOTE_GS1": 52,
    "NOTE_A1": 55,
    "NOTE_AS1": 58,
    "NOTE_B1": 62,
    "NOTE_C2": 65,
    "NOTE_CS2": 69,
    "NOTE_D2": 73,
    "NOTE_DS2": 78,
    "NOTE_E2": 82,
    "NOTE_F2": 87,
    "NOTE_FS2": 93,
    "NOTE_G2": 98,
    "NOTE_GS2": 104,
    "NOTE_A2": 110,
    "NOTE_AS2": 117,
    "NOTE_B2": 123,
    "NOTE_C3": 131,
    "NOTE_CS3": 139,
    "NOTE_D3": 147,
    "NOTE_DS3": 156,
    "NOTE_E3": 165,
    "NOTE_F3": 175,
    "NOTE_FS3": 185,
    "NOTE_G3": 196,
    "NOTE_GS3": 208,
    "NOTE_A3": 220,
    "NOTE_AS3": 233,
    "NOTE_B3": 247,
    "NOTE_C4": 262,
    "NOTE_CS4": 277,
    "NOTE_D4": 294,
    "NOTE_DS4": 311,
    "NOTE_E4": 330,
    "NOTE_F4": 349,
    "NOTE_FS4": 370,
    "NOTE_G4": 392,
    "NOTE_GS4": 415,
    "NOTE_A4": 440,
    "NOTE_AS4": 466,
    "NOTE_B4": 494,
    "NOTE_C5": 523,
    "NOTE_CS5": 554,
    "NOTE_D5": 587,
    "NOTE_DS5": 622,
    "NOTE_E5": 659,
    "NOTE_F5": 698,
    "NOTE_FS5": 740,
    "NOTE_G5": 784,
    "NOTE_GS5": 831,
    "NOTE_A5": 880,
    "NOTE_AS5": 932,
    "NOTE_B5": 988,
    "NOTE_C6": 1047,
    "NOTE_CS6": 1109,
    "NOTE_D6": 1175,
    "NOTE_DS6": 1245,
    "NOTE_E6": 1319,
    "NOTE_F6": 1397,
    "NOTE_FS6": 1480,
    "NOTE_G6": 1568,
    "NOTE_GS6": 1661,
    "NOTE_A6": 1760,
    "NOTE_AS6": 1865,
    "NOTE_B6": 1976,
    "NOTE_C7": 2093,
    "NOTE_CS7": 2217,
    "NOTE_D7": 2349,
    "NOTE_DS7": 2489,
    "NOTE_E7": 2637,
    "NOTE_F7": 2794,
    "NOTE_FS7": 2960,
    "NOTE_G7": 3136,
    "NOTE_GS7": 3322,
    "NOTE_A7": 3520,
    "NOTE_AS7": 3729,
    "NOTE_B7": 3951,
    "NOTE_C8": 4186,
    "NOTE_CS8": 4435,
    "NOTE_D8": 4699,
    "NOTE_DS8": 4978,
}

# === Star Wars melody ===
star_wars_melody = [
    "NOTE_AS4",
    8,
    "NOTE_AS4",
    8,
    "NOTE_AS4",
    8,
    "NOTE_F5",
    2,
    "NOTE_C6",
    2,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F6",
    2,
    "NOTE_C6",
    4,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F6",
    2,
    "NOTE_C6",
    4,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_AS5",
    8,
    "NOTE_G5",
    2,
    "NOTE_C5",
    8,
    "NOTE_C5",
    8,
    "NOTE_C5",
    8,
    "NOTE_F5",
    2,
    "NOTE_C6",
    2,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F6",
    2,
    "NOTE_C6",
    4,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F6",
    2,
    "NOTE_C6",
    4,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_AS5",
    8,
    "NOTE_G5",
    2,
    "NOTE_C5",
    -8,
    "NOTE_C5",
    16,
    "NOTE_D5",
    -4,
    "NOTE_D5",
    8,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F5",
    8,
    "NOTE_F5",
    8,
    "NOTE_G5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    4,
    "NOTE_D5",
    8,
    "NOTE_E5",
    4,
    "NOTE_C5",
    -8,
    "NOTE_C5",
    16,
    "NOTE_D5",
    -4,
    "NOTE_D5",
    8,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F5",
    8,
    "NOTE_C6",
    -8,
    "NOTE_G5",
    16,
    "NOTE_G5",
    2,
    "REST",
    8,
    "NOTE_C5",
    8,
    "NOTE_D5",
    -4,
    "NOTE_D5",
    8,
    "NOTE_AS5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    8,
    "NOTE_F5",
    8,
    "NOTE_F5",
    8,
    "NOTE_G5",
    8,
    "NOTE_A5",
    8,
    "NOTE_G5",
    4,
    "NOTE_D5",
    8,
    "NOTE_E5",
    4,
    "NOTE_C6",
    -8,
    "NOTE_C6",
    16,
    "NOTE_F6",
    4,
    "NOTE_DS6",
    8,
    "NOTE_CS6",
    4,
    "NOTE_C6",
    8,
    "NOTE_AS5",
    4,
    "NOTE_GS5",
    8,
    "NOTE_G5",
    4,
    "NOTE_F5",
    8,
    "NOTE_C6",
    1,
]


class BuzzerController:
    def __init__(self, buzzer_pin=None):
        """Initialize the buzzer controller."""
        if buzzer_pin is None:
            buzzer_pin = params.data["BUZZER_CONFIG"]["pin"]

        self.pin = Pin(buzzer_pin)
        self.buzzer = PWM(self.pin)
        self.buzzer.duty_u16(0)

        config = params.data["BUZZER_CONFIG"]
        self.tempo = config.get("default_tempo", 1.0)
        self.volume = config.get("default_volume", 0.5)

        self.playing = False
        self.current_task = None

        # Play a simple melody on startup
        self.play_startup_melody()

    def play_startup_melody(self):
        # A simple melody like C5, E5, G5
        simple_melody = ["NOTE_C5", 8, "NOTE_E5", 8, "NOTE_G5", 4]
        self.play_melody_async(simple_melody, base_tempo=120)

    async def _play_tone_async(self, freq, duration_ms):
        if freq == 0:
            self.buzzer.duty_u16(0)
        else:
            self.buzzer.freq(freq)
            self.buzzer.duty_u16(int(65535 * self.volume))
        await asyncio.sleep_ms(int(duration_ms))
        self.buzzer.duty_u16(0)

    async def _play_melody_async(self, melody, base_tempo):
        self.playing = True
        wholenote = int((60000 * 4) / (base_tempo * self.tempo))

        for i in range(0, len(melody), 2):
            if not self.playing:
                break

            note, duration = melody[i], melody[i + 1]
            freq = NOTE_FREQ.get(note, 0)

            if duration > 0:
                note_duration = wholenote // duration
            else:
                note_duration = int(wholenote // abs(duration) * 1.5)

            play_time = int(note_duration * 0.9)
            await self._play_tone_async(freq, play_time)
            await asyncio.sleep_ms(note_duration - play_time)

        self.buzzer.duty_u16(0)
        self.playing = False
        self.current_task = None

    def play_melody_async(self, melody, base_tempo=108):
        if self.current_task is None or self.current_task.done():
            self.current_task = asyncio.create_task(
                self._play_melody_async(melody, base_tempo)
            )

    def stop(self):
        self.playing = False
        self.buzzer.duty_u16(0)
