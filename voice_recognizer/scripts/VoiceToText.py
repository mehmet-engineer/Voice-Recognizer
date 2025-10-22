import whisper

class VoiceToText:
    def __init__(self, model: str, auto_detect: bool, lang: str):
        self.model = whisper.load_model(model)
        self.auto_detect = auto_detect
        self.lang = lang
        
    def convert_to_text(self, audio_file) -> str:
        if self.auto_detect:
            result = self.model.transcribe(audio_file, language=None)
        else:
            result = self.model.transcribe(audio_file, language=self.lang)

        return result['text']