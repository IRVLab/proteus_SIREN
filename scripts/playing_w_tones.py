from tones import SINE_WAVE, SAWTOOTH_WAVE, SQUARE_WAVE, TRIANGLE_WAVE
from tones.mixer import Mixer

from pydub import AudioSegment
from pydub.playback import play

mixer = Mixer(44100,0.5)

mixer.create_track(0, SQUARE_WAVE, vibrato_frequency=0, vibrato_variance=0, attack=0.001, decay=0.01)

mixer.add_note(0, note="c", octave=4, duration=1.0)
mixer.add_note(0, note="d", octave=4, duration=1.0)
mixer.add_note(0, note="e", octave=4, duration=1.0)
mixer.add_note(0, note="f", octave=4, duration=1.0)
mixer.add_note(0, note="g", octave=4, duration=1.0)
mixer.add_note(0, note="a", octave=4, duration=1.0)
mixer.add_note(0, note="b", octave=4, duration=1.0)

bstr = mixer.sample_data()

a = AudioSegment.from_ogg("../audio_files/duck_test.ogg")
a._data = bstr
play(a)