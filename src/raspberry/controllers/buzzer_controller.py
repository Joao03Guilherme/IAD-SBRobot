from machine import Pin, PWM
import uasyncio as asyncio
import urandom
import time
import parameters.parameters as params

class BuzzerController:
    """Controller for playing sounds on a piezo buzzer using PWM.
    
    Features:
    - Star Wars Imperial March theme
    - R2D2 random beeping sounds
    - Non-blocking sound playback using asyncio
    """
    
    # Star Wars Imperial March notes and durations
    IMPERIAL_MARCH_NOTES = [
        392, 0, 392, 0, 392, 0, 311, 0, 466, 0, 392, 0, 311, 0, 466, 0, 392, 0,  # First phrase
        587, 0, 587, 0, 587, 0, 622, 0, 466, 0, 370, 0, 311, 0, 466, 0, 392, 0,  # Second phrase
        784, 0, 392, 0, 392, 0, 784, 0, 740, 0, 698, 0, 659, 0, 622, 0,          # Third phrase
        659, 0, 415, 0, 554, 0, 523, 0, 494, 0, 466, 0, 440, 0, 
        466, 0, 370, 0, 392, 0                                                    # Final notes
    ]
    
    IMPERIAL_MARCH_DURATIONS = [
        500, 50, 500, 50, 500, 50, 350, 50, 150, 50, 500, 50, 350, 50, 150, 50, 1000, 100,  # First phrase
        500, 50, 500, 50, 500, 50, 350, 50, 150, 50, 500, 50, 350, 50, 150, 50, 1000, 100,  # Second phrase
        500, 50, 350, 50, 150, 50, 500, 50, 250, 50, 250, 50, 250, 50, 250, 50,              # Third phrase
        300, 50, 150, 50, 150, 50, 250, 50, 200, 50, 150, 50, 300, 50, 
        150, 50, 350, 50, 1000, 50                                                           # Final notes
    ]
    
    def __init__(self, buzzer_pin=None):
        """Initialize the buzzer controller.
        
        Args:
            buzzer_pin: Pin number to use for buzzer. If None, uses pin from config.
        """
        if buzzer_pin is None:
            buzzer_pin = params.data["BUZZER_CONFIG"]["pin"]
            
        self.pin = Pin(buzzer_pin)
        self.buzzer = PWM(self.pin)
        self.buzzer.duty_u16(0)  # Start with buzzer off
        
        # Load configuration values
        buzzer_config = params.data["BUZZER_CONFIG"]
        self.tempo = buzzer_config.get("default_tempo", 1.0)  # Speed multiplier
        self.volume = buzzer_config.get("default_volume", 0.5)  # Volume level 
        self.r2d2_min_freq = buzzer_config.get("r2d2_min_freq", 1000)
        self.r2d2_max_freq = buzzer_config.get("r2d2_max_freq", 4000)
        self.r2d2_min_duration = buzzer_config.get("r2d2_min_duration", 40)
        self.r2d2_max_duration = buzzer_config.get("r2d2_max_duration", 300)
        
        # State tracking
        self.playing = False
        self.current_task = None
        
        # Initialize with a test beep
        self._play_beep()
        
    def _play_beep(self):
        """Play a quick startup beep."""
        self.buzzer.freq(params.data["BUZZER_CONFIG"]["frequency"])
        self.buzzer.duty_u16(10000)  # About 15% volume
        time.sleep_ms(100)
        self.buzzer.duty_u16(0)
        
    def set_tempo(self, tempo):
        """Set the tempo for Star Wars playback.
        
        Args:
            tempo: Speed multiplier (0.5 = half speed, 2.0 = double speed)
        """
        self.tempo = max(0.2, min(3.0, tempo))
        return self.tempo
        
    def set_volume(self, volume):
        """Set the volume for sound playback.
        
        Args:
            volume: Volume level from 0.0 (silent) to 1.0 (full volume)
        """
        self.volume = max(0.0, min(1.0, volume))
        return self.volume
    
    def play_tone(self, frequency, duration_ms):
        """Play a single tone (blocking).
        
        Args:
            frequency: Frequency in Hz
            duration_ms: Duration in milliseconds
        """
        if frequency > 0:
            self.buzzer.freq(frequency)
            volume = int(32767 * self.volume)
            self.buzzer.duty_u16(volume)
        else:
            self.buzzer.duty_u16(0)  # No sound for frequency=0 (rest)
            
        time.sleep_ms(duration_ms)
    
    async def play_tone_async(self, frequency, duration_ms):
        """Play a single tone asynchronously.
        
        Args:
            frequency: Frequency in Hz
            duration_ms: Duration in milliseconds
        """
        if frequency > 0:
            self.buzzer.freq(frequency)
            volume = int(32767 * self.volume)
            self.buzzer.duty_u16(volume)
        else:
            self.buzzer.duty_u16(0)  # No sound for frequency=0 (rest)
            
        await asyncio.sleep_ms(duration_ms)
    
    async def _play_star_wars_task(self):
        """Background task for playing the Star Wars Imperial March."""
        self.playing = True
        
        # Play the Imperial March
        for i in range(0, len(self.IMPERIAL_MARCH_NOTES)):
            # Check if we should stop early
            if not self.playing:
                break
                
            # Calculate tempo-adjusted duration
            duration = int(self.IMPERIAL_MARCH_DURATIONS[i] / self.tempo)
            await self.play_tone_async(self.IMPERIAL_MARCH_NOTES[i], duration)
        
        # Make sure we turn off the buzzer when done
        self.buzzer.duty_u16(0)
        self.playing = False
    
    async def _play_r2d2_sounds_task(self, duration_seconds=5):
        """Background task for playing random R2D2-like sounds.
        
        Args:
            duration_seconds: Total duration to generate sounds for
        """
        self.playing = True
        end_time = time.time() + duration_seconds
        
        while time.time() < end_time and self.playing:
            # Random chirp with random characteristics
            freq = urandom.randint(self.r2d2_min_freq, self.r2d2_max_freq)
            duration = urandom.randint(self.r2d2_min_duration, self.r2d2_max_duration)
            
            # Play the tone
            await self.play_tone_async(freq, duration)
            
            # Small gap between tones (sometimes)
            if urandom.randint(0, 10) > 7:
                await self.play_tone_async(0, urandom.randint(30, 100))
                
        # Make sure we turn off the buzzer when done
        self.buzzer.duty_u16(0)
        self.playing = False
    
    def play_star_wars_song(self):
        """Play the Star Wars Imperial March theme in the background.
        
        This method is non-blocking and will return immediately.
        """
        self.stop()  # Stop any currently playing sounds
        self.current_task = asyncio.create_task(self._play_star_wars_task())
        return True
    
    async def play_star_wars_song_async(self):
        """Play the Star Wars Imperial March theme asynchronously.
        
        This method returns a task that can be awaited to play the entire song.
        """
        self.stop()  # Stop any currently playing sounds
        return await self._play_star_wars_task()
    
    def play_random_sound(self, duration_seconds=5):
        """Play random R2D2-like sounds in the background.
        
        Args:
            duration_seconds: Duration in seconds to generate sounds for
            
        This method is non-blocking and will return immediately.
        """
        self.stop()  # Stop any currently playing sounds
        self.current_task = asyncio.create_task(
            self._play_r2d2_sounds_task(duration_seconds)
        )
        return True
    
    async def play_random_sound_async(self, duration_seconds=5):
        """Play random R2D2-like sounds asynchronously.
        
        Args:
            duration_seconds: Duration in seconds to generate sounds for
            
        This method returns a task that can be awaited to play all sounds.
        """
        self.stop()  # Stop any currently playing sounds
        return await self._play_r2d2_sounds_task(duration_seconds)
    
    def stop(self):
        """Stop all sounds being played."""
        self.playing = False
        self.buzzer.duty_u16(0)
        
        # Cancel any running task
        if self.current_task is not None:
            try:
                self.current_task.cancel()
            except Exception as e:
                print(f"Error canceling sound task: {e}")
        
        return True

    def is_playing(self):
        """Check if a sound is currently being played."""
        return self.playing

# Example usage (if run standalone)
if __name__ == "__main__":
    async def test_buzzer():
        buzzer = BuzzerController()
        
        print("Playing Star Wars song...")
        buzzer.play_star_wars_song()
        await asyncio.sleep(5)  # Let it play for 5 seconds
        
        print("Stopping...")
        buzzer.stop()
        await asyncio.sleep(1)
        
        print("Playing R2D2 sounds...")
        buzzer.play_random_sound(duration_seconds=3)
        await asyncio.sleep(3)
        
        print("Done!")
        buzzer.stop()
    
    # Run the test
    asyncio.run(test_buzzer())