from voxpopuli import Voice

test_sentence = "The birch canoe slid on the smooth planks. Glue the sheet to the dark blue background. It's easy to tell the depth of a well. These days a chicken leg is a rare dish. Rice is often served in round bowls."

en = Voice(lang="en", speed=150, volume=1, voice_id=1)
us1 = Voice(lang="us", speed=150, volume=1, voice_id=1)
us2 = Voice(lang="us", speed=150, volume=1, voice_id=2)
us3 = Voice(lang="us", speed=150, volume=1, voice_id=3)

with open("/home/michael/Desktop/siren_wavs/en_voice.wav", 'wb') as fstream:
    fstream.write(en.to_audio(test_sentence))

with open("/home/michael/Desktop/siren_wavs/us1_voice.wav", 'wb') as fstream:
    fstream.write(us1.to_audio(test_sentence))

with open("/home/michael/Desktop/siren_wavs/us2_voice.wav", 'wb') as fstream:
    fstream.write(us2.to_audio(test_sentence))

with open("/home/michael/Desktop/siren_wavs/us3_voice.wav", 'wb') as fstream:
    fstream.write(us3.to_audio(test_sentence))

