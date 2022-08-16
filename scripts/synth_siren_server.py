#!/usr/bin/python3

import rospy
from rosnode import get_node_names
from rospy import service
import rospkg
rospack = rospkg.RosPack()

from tones import SINE_WAVE, SAWTOOTH_WAVE, SQUARE_WAVE, TRIANGLE_WAVE
from tones.mixer import Mixer

from pydub import AudioSegment
import pydub.audio_segment
from pydub.playback import play

import sys
from os.path import exists
import xml.etree.ElementTree as ET
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity
from proteus.soneme import Soneme, SNode, SNodeTone
from proteus.siren import SirenConfig
from proteus.tone import Tone, VariableTone, RunTone

rospy.init_node('ogg_siren_server', argv=None, anonymous=True)
siren_config = None

# Farms out the execution of the soneme to the appropriate function
def service_cb(req, soneme):
    rospy.logdebug('Service callback for soneme %s'%(soneme.id))
    if soneme.call_type == 'trigger':
        return execute_trigger(req, soneme)
    elif soneme.call_type == 'directional':
        return execute_directional(req, soneme)
    elif soneme.call_type == 'target':
        return execute_target(req, soneme)
    elif soneme.call_type == 'quantity':
        return execute_quantity(req, soneme)
    else:
        return False

def execute_trigger(req, soneme):
    mixer = Mixer(44100,0.5)
    tracks = dict()

    for sn in soneme.snodes:
        d = sn.duration
        if len(sn.tones)> 0:
            for t in sn.tones:
                track_id = t.track_id

                # Only create the track if it hasn't been created before.
                if track_id not in tracks.keys():
                    idx = len(tracks.keys())
                    tracks[track_id] = idx

                    tdef = siren_config.synth_tracks[track_id]

                    if tdef.wave_type == "sine":
                        wave_type = SINE_WAVE
                    elif tdef.wave_type == "saw":
                        wave_type = SAWTOOTH_WAVE
                    elif tdef.wave_type == "square":
                        wave_type = SQUARE_WAVE
                    elif tdef.wave_type == "triangle":
                        wave_type = TRIANGLE_WAVE

                    mixer.create_track(idx,wave_type, vibrato_frequency=tdef.vibrato, vibrato_variance=tdef.vibrato_variance, attack=tdef.attack, decay=tdef.decay)

                mixer.add_note(tracks[track_id], note=t.note, octave=t.octave, endnote=t.end_note, duration=d.seconds)
        else:
            mixer.add_silence(0, duration=d.seconds)

    # We need an audio segment, so load in the duck, then replace the data with our mixer's data.
    dir = siren_config.clip_location
    duck_fn = dir + '/duck_test.ogg'

    a = AudioSegment.from_ogg(duck_fn)
    a._data = mixer.sample_data()
    play(a)

    return True
            
    
def execute_directional(req, soneme):
    mixer = Mixer(44100,0.5)
    tracks = dict()

    for sn in soneme.snodes:
        d = sn.duration
        if len(sn.tones)> 0:
            for t in sn.tones:
                track_id = t.track_id

                # Only create the track if it hasn't been created before.
                if track_id not in tracks.keys():
                    idx = len(tracks.keys())
                    tracks[track_id] = idx

                    tdef = siren_config.synth_tracks[track_id]

                    if tdef.wave_type == "sine":
                        wave_type = SINE_WAVE
                    elif tdef.wave_type == "saw":
                        wave_type = SAWTOOTH_WAVE
                    elif tdef.wave_type == "square":
                        wave_type = SQUARE_WAVE
                    elif tdef.wave_type == "triangle":
                        wave_type = TRIANGLE_WAVE

                    mixer.create_track(idx,wave_type, vibrato_frequency=tdef.vibrato, vibrato_variance=tdef.vibrato_variance, attack=tdef.attack, decay=tdef.decay)

                if type(t) == Tone:
                    mixer.add_note(tracks[track_id], note=t.note, octave=t.octave, endnote=t.end_note, duration=d.seconds)
                elif type(t) == VariableTone:
                    if t.parameter == 'y-val':
                        value = req.transform.rotation.y
                    elif t.parameter == 'z-val':
                        value = req.transform.rotation.z
                    else:
                        print("No idea how to handle this.")

                    if value == 0:
                        mixer.add_note(tracks[track_id], note=t.options[1][0], octave=t.options[1][1], duration=d.seconds)
                    elif value > 0:
                        mixer.add_note(tracks[track_id], note=t.options[2][0], octave=t.options[2][1], duration=d.seconds)
                    elif value < 0:
                        mixer.add_note(tracks[track_id], note=t.options[0][0], octave=t.options[0][1], duration=d.seconds)
                    
        else:
            mixer.add_silence(0, duration=d.seconds)

    # We need an audio segment, so load in the duck, then replace the data with our mixer's data.
    dir = siren_config.clip_location
    duck_fn = dir + '/duck_test.ogg'

    a = AudioSegment.from_ogg(duck_fn)
    a._data = mixer.sample_data()
    play(a)

    return True

def execute_target(req, soneme):
    pass

def execute_quantity(req, soneme):
    mixer = Mixer(44100,0.5)
    tracks = dict()

    for sn in soneme.snodes:
        d = sn.duration
        if len(sn.tones)> 0:
            for t in sn.tones:
                track_id = t.track_id

                # Only create the track if it hasn't been created before.
                if track_id not in tracks.keys():
                    idx = len(tracks.keys())
                    tracks[track_id] = idx

                    tdef = siren_config.synth_tracks[track_id]

                    if tdef.wave_type == "sine":
                        wave_type = SINE_WAVE
                    elif tdef.wave_type == "saw":
                        wave_type = SAWTOOTH_WAVE
                    elif tdef.wave_type == "square":
                        wave_type = SQUARE_WAVE
                    elif tdef.wave_type == "triangle":
                        wave_type = TRIANGLE_WAVE

                    mixer.create_track(idx,wave_type, vibrato_frequency=tdef.vibrato, vibrato_variance=tdef.vibrato_variance, attack=tdef.attack, decay=tdef.decay)

                if type(t) == Tone:
                    mixer.add_note(tracks[track_id], note=t.note, octave=t.octave, endnote=t.end_note, duration=d.seconds)
                elif type(t) == RunTone:
                    tone_values = t.get_dyn_notes(req.quantity, sn.duration.seconds, sn.quantity.min_amount, sn.quantity.max_amount)
                    # We get note, octave, and duration. If note/octave are None, simply add silence.
                    for n, o, d in tone_values:
                        if n:
                            mixer.add_note(tracks[track_id], note=n, octave=o, duration=d)
                        else:
                            mixer.add_silence(tracks[track_id], duration=d)

        else:
            mixer.add_silence(0, duration=d.seconds)

    # We need an audio segment, so load in the duck, then replace the data with our mixer's data.
    dir = siren_config.clip_location
    duck_fn = dir + '/duck_test.ogg'

    a = AudioSegment.from_ogg(duck_fn)
    a._data = mixer.sample_data()
    play(a)

    return True

if __name__ == '__main__':
    rospy.loginfo('Initializing the SIREN server')

    #Check if PROTEUS language server is up
    rospy.loginfo('Checking SIREN language server...')
    lang_server_active = False
    nodes = get_node_names()
    rospy.logdebug(nodes)
    for n in nodes:
        if n.split('/')[-1] == 'proteus_language_server':
            lang_server_active = True
            break
    if not lang_server_active:
        rospy.logerr("This SIREN implementation requires the PROTEUS language server to be active.")
        sys.exit(1)
    else:
        rospy.loginfo('PROTEUS language server OK!')


     # Find soneme language definition file
    rospy.loginfo("Loading vector information...")
    siren_info = rospy.get_param('vectors/out/TonalSiren')
    siren_def_file = siren_info['definition_file']

    # Find symbol definitions
    rospy.loginfo("Loading symbol information...")
    symbols = rospy.get_param('symbols/out')
    
    # Process soneme definition file into soneme objects
    rospy.loginfo("Loading soneme definitions from symbol definition file.")
    sonemes = dict()

    #Load XML file
    tree = ET.parse(siren_def_file)
    root = tree.getroot()    

    for item in root:
        if item.tag == 'sonemes':
            for sdef in item:
                s = Soneme()
                s.parse_from_xml(sdef)
                sonemes[s.id] = s
        elif item.tag == 'siren-config':
            #Special case for parsing the meta information.
            siren_config = SirenConfig()
            siren_config.parse_from_xml(item)

    # Check for symbol matchup.
    for sym in symbols:
        for key,s in sonemes.items():
            if sym == key:
                rospy.loginfo("Found match beteween symbol %s and soneme %s, associating data."%(sym, key))
                rospy.logdebug("Call type: %s"%(symbols.get(sym).get('call_type')))
                s.set_call_type(symbols.get(sym).get('call_type'))
                break

    # Setup service calls
    for key, soneme in sonemes.items():
        service_class = None
        if soneme.call_type == 'trigger':
            service_class = SymbolTrigger
        elif soneme.call_type == 'directional':
            service_class = SymbolDirectional
        elif soneme.call_type == 'target':
            service_class = SymbolTarget
        elif soneme.call_type == 'quantity':
            service_class = SymbolQuantity
        else:
            rospy.logwarn("Unexpected call type {} for soneme {}".format(soneme.call_type, soneme.id))

        service_name = 'siren/synth/'+ soneme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for soneme %s at service endpoint: %s'%(soneme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, soneme=soneme: service_cb(req, soneme))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass