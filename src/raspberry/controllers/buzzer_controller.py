from machine import Pin, PWM
import parameters.parameters as params
import asyncio


class BuzzerController:
    # Notes definition 
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
        "NOTE_AS4", 8, "NOTE_AS4", 8, "NOTE_AS4", 8, "NOTE_F5", 2, "NOTE_C6", 2,
        "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_G5", 8, "NOTE_F6", 2, "NOTE_C6", 4,
        "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_G5", 8, "NOTE_F6", 2, "NOTE_C6", 4,
        "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_AS5", 8, "NOTE_G5", 2, "NOTE_C5", 8,
        "NOTE_C5", 8, "NOTE_C5", 8, "NOTE_F5", 2, "NOTE_C6", 2, "NOTE_AS5", 8,
        "NOTE_A5", 8, "NOTE_G5", 8, "NOTE_F6", 2, "NOTE_C6", 4, "NOTE_AS5", 8,
        "NOTE_A5", 8, "NOTE_G5", 8, "NOTE_F6", 2, "NOTE_C6", 4, "NOTE_AS5", 8,
        "NOTE_A5", 8, "NOTE_AS5", 8, "NOTE_G5", 2, "NOTE_C5", -8, "NOTE_C5", 16,
        "NOTE_D5", -4, "NOTE_D5", 8, "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_G5", 8,
        "NOTE_F5", 8, "NOTE_F5", 8, "NOTE_G5", 8, "NOTE_A5", 8, "NOTE_G5", 4,
        "NOTE_D5", 8, "NOTE_E5", 4, "NOTE_C5", -8, "NOTE_C5", 16, "NOTE_D5", -4,
        "NOTE_D5", 8, "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_G5", 8, "NOTE_F5", 8,
        "NOTE_C6", -8, "NOTE_G5", 16, "NOTE_G5", 2, "REST", 8, "NOTE_C5", 8,
        "NOTE_D5", -4, "NOTE_D5", 8, "NOTE_AS5", 8, "NOTE_A5", 8, "NOTE_G5", 8,
        "NOTE_F5", 8, "NOTE_F5", 8, "NOTE_G5", 8, "NOTE_A5", 8, "NOTE_G5", 4,
        "NOTE_D5", 8, "NOTE_E5", 4, "NOTE_C6", -8, "NOTE_C6", 16, "NOTE_F6", 4,
        "NOTE_DS6", 8, "NOTE_CS6", 4, "NOTE_C6", 8, "NOTE_AS5", 4, "NOTE_GS5", 8,
        "NOTE_G5", 4, "NOTE_F5", 8, "NOTE_C6", 1,
    ]

    def __init__(self, buzzer_pin=None):
        """Initialize the buzzer controller."""
        if buzzer_pin is None:
            buzzer_pin = params.data["BUZZER_CONFIG"]["pin"]

        self.pin = Pin(buzzer_pin)
        self.buzzer = PWM(self.pin)
        self.buzzer.freq(1000)  # Initialize with 1kHz
        self.buzzer.duty_u16(0)  # Start with duty cycle at 0

        config = params.data["BUZZER_CONFIG"]
        self.tempo = config.get("default_tempo", 1.0)
        self.volume = config.get("default_volume", 1.0)

        self.playing = False
        self.current_task = None
        self.periodic_beeping = False

        # Play a short beep on startup and start periodic beeping
        self.start_periodic_beeping()

    async def _play_tone_async(self, freq, duration_ms):
        if freq == 0:
            self.buzzer.duty_u16(0)  # Rest
        else:
            self.buzzer.freq(freq)
            # Using 50% duty cycle (32767) works better for square wave output
            # We multiply by volume to allow volume control
            duty = int(32767 * self.volume)
            self.buzzer.duty_u16(duty)

        await asyncio.sleep_ms(int(duration_ms))

    async def _periodic_beep_task(self) -> None:
        """
        Asynchronous task that beeps once every 2 seconds.
        Plays a short beep, then waits for 2 seconds before repeating.
        Ensures the buzzer is turned off when the task is stopped.
        
        Args:
            None
        Returns:
            None
        """
        self.periodic_beeping = True
        while self.periodic_beeping:
            await self._play_tone_async(self.NOTE_FREQ["NOTE_C5"], 100)
            self.buzzer.duty_u16(0)
            await asyncio.sleep(2)
        self.buzzer.duty_u16(0)

    def start_periodic_beeping(self) -> asyncio.Task:
        """
        Start a periodic beep every 2 seconds.
        Cancels any existing periodic beep task before starting a new one.
        
        Args:
            None
        Returns:
            asyncio.Task: The task object for the periodic beeping.
        """
        if self.current_task is not None and not self.current_task.done():
            self.stop_periodic_beeping()
        self.current_task = asyncio.create_task(self._periodic_beep_task())
        return self.current_task

    def stop_periodic_beeping(self) -> asyncio.Task:
        """
        Stop the periodic beeping task.
        Sets the periodic_beeping flag to False and schedules the buzzer to stop.
        
        Args:
            None
        Returns:
            asyncio.Task: The task object for stopping the buzzer.
        """
        self.periodic_beeping = False
        return asyncio.create_task(self.stop())

    async def _play_melody_async(self, melody: list[str|int], base_tempo: int) -> None:
        """
        Play a melody asynchronously.
        
        Args:
            melody (list[str|int]): A list of notes and their durations.
            base_tempo (int): The base tempo for the melody.
        Returns:
            None
        """
        self.playing = True
        wholenote = int((60000 * 4) / (base_tempo * self.tempo))
        for i in range(0, len(melody), 2):
            if not self.playing:
                break
            note, duration = melody[i], melody[i + 1]
            freq = self.NOTE_FREQ.get(note, 0)
            if duration > 0:
                note_duration = wholenote // duration
            else:
                note_duration = int(wholenote // abs(duration) * 1.5)
            play_time = int(note_duration * 0.9)
            await self._play_tone_async(freq, play_time)
            self.buzzer.duty_u16(0)
            await asyncio.sleep_ms(note_duration - play_time)
        self.buzzer.duty_u16(0)
        self.playing = False
        self.current_task = None

    def play_melody_async(self, melody: list[str|int]=None, base_tempo: int=108) -> asyncio.Task:
        """
        Play a melody asynchronously. Wrapper for the _play_melody_async method.
        
        Args:
            melody (list[str|int], optional): A list of notes and their durations. Defaults to Star Wars melody.
            base_tempo (int, optional): The base tempo for the melody. Defaults to 108.
        Returns:
            asyncio.Task: The task object for the melody playback.
        """
        if melody is None:
            melody = self.star_wars_melody
        if self.current_task is None or self.current_task.done():
            self.current_task = asyncio.create_task(
                self._play_melody_async(melody, base_tempo)
            )
            return self.current_task
        return self.current_task

    async def stop(self) -> None:
        """
        Stops all melody playback and periodic beeping.
        
        Args:
            None
        Returns:
            None
        """
        self.playing = False
        self.periodic_beeping = False
        self.buzzer.duty_u16(0)
        if self.current_task and not self.current_task.done():
            try:
                self.current_task.cancel()
                await asyncio.sleep(0.1)
            except Exception:
                pass

    def set_tempo(self, tempo: float) -> bool:
        """
        Set the tempo multiplier for melody playback.
        
        Args:
            tempo (float): The tempo multiplier (1.0 is normal speed).
        Returns:
            bool: True if the tempo was set successfully, False otherwise.
        """
        self.tempo = float(tempo)
        return True

    def set_volume(self, volume: float) -> bool:
        """
        Set the volume level (0.0 to 1.0).
        
        Args:
            volume (float): The volume level (0.0 to 1.0).
        Returns:
            bool: True if the volume was set successfully, False otherwise.
        """
        self.volume = max(0.0, min(1.0, float(volume)))
        print(f"Volume set to {self.volume}")
        return True

    def play_star_wars_song(self) -> asyncio.Task:
        """
        Play the Star Wars theme song.
        
        Args:
            None
        Returns:
            asyncio.Task: The task object for the melody playback.
        """
        return self.play_melody_async(self.star_wars_melody, base_tempo=108)

    def play_random_sound(self, duration_seconds: int=5) -> asyncio.Task:
        """
        Play random R2D2-like beeps and boops.
        
        Args:
            duration_seconds (int, optional): Duration of the random sound in seconds. Defaults to 5.
        Returns:
            asyncio.Task: The task object for the melody playback.
        """
        import random
        r2d2_melody = []
        notes = [
            "NOTE_C6", "NOTE_D6", "NOTE_E6", "NOTE_F6", "NOTE_G6", "NOTE_A6", "NOTE_B6", "NOTE_C7",
        ]
        durations = [8, 16, 4, 2]
        num_notes = duration_seconds * 10
        for _ in range(int(num_notes)):
            note = random.choice(notes)
            duration = random.choice(durations)
            r2d2_melody.extend([note, duration])
            if random.random() < 0.2:
                r2d2_melody.extend(["REST", duration])
        return self.play_melody_async(r2d2_melody, base_tempo=180)
