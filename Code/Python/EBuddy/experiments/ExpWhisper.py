# pip install openai-whisper
# pip install pyaudio

# Wave example
import whisper

# Load the model
# model = whisper.load_model("base")

# Transcribe an audio file
# result = model.transcribe(r"D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\Data\Input\waves\audio_tiny8.wav")
#
# # Print the transcribed text
# print(result["text"])


# Online example
import whisper
import pyaudio
import wave

# Load Whisper model
model = whisper.load_model("base")

# Set up audio stream
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=1024)

print("Listening...")

try:
    while True:
        data = stream.read(1024)
        # Save audio data to a temporary file
        with wave.open("temp.wav", "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(data)

        # Transcribe audio
        result = model.transcribe("temp.wav")
        print(result["text"])

except KeyboardInterrupt:
    print("Stopped listening.")
    stream.stop_stream()
    stream.close()
    p.terminate()