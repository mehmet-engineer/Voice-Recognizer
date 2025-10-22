import numpy as np
import scipy.io.wavfile
import sounddevice as sd

class VoiceRecognizer:
    def __init__(self, sample_rate: int):
        self.sample_rate = sample_rate
        self.channels = 1
        self.dtype = 'int16'
        self.recording = []
        self.stream = None
        self._is_recording = False

    def _callback(self, indata, frames, time, status):
        self.recording.append(indata.copy())

    def start_recording(self):
        self.recording = []
        self.stream = sd.InputStream(samplerate=self.sample_rate,
                                     channels=self.channels,
                                     dtype=self.dtype,
                                     callback=self._callback)
        self.stream.start()
        self._is_recording = True

    def stop_recording(self):
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None
            self._is_recording = False

    def save(self, filename: str):
        if self.recording:
            audio_data = np.concatenate(self.recording, axis=0)
            scipy.io.wavfile.write(filename, self.sample_rate, audio_data)
            
    def record_for_duration(self, duration_sec: int, filename: str):
        self.start_recording()
        sd.sleep(duration_sec * 1000)
        self.stop_recording()
        self.save(filename)

    def is_recording(self) -> bool:
        return self._is_recording
