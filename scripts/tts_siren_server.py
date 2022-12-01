#!/usr/bin/python3
from email.policy import default
import rospy
from rosnode import get_node_names
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

from voxpopuli import Voice
from pydub import AudioSegment
from pydub.playback import play

import sys
import xml.etree.ElementTree as ET
from proteus_msgs.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity
from proteus.soneme import Soneme, SNode, SNodeClip, SNodeSpeech
from proteus.siren import SirenConfig

rospy.init_node('ogg_siren_server', argv=None, anonymous=True)
siren_config = None
default_voice = None

# Fit transform into text description of cardinal direction
def cardinalize(transform):
    q = transform.rotation
    # rpy = euler_from_quaternion([q.x, q.y, q.z, q.w]) #We only actually need pitch and yaw, roll is ignored here.
    rpy = [q.x, q.y, q.z] # THIS IS A DIRTY HACK

    ret = "" # Return string.

    # Pitch handling
    if rpy[1] > 0:
        ret+= " up"
    elif rpy[1] < 0:
        ret+= " down"

    # Add a conjunction if there's pitch involved.
    if rpy[1] != 0 and rpy[2] != 0:
        ret+= " and "

    # Yaw handling
    if rpy[2] > 0:
        ret+= "left"
    elif rpy[2] < 0:
        ret += "right"

    return ret

# Fit quantity into available bins.
def bin_quant(quantity, bins):
    return min(bins, key=lambda x:abs(x-quantity))

def text_cb(msg):
    global default_voice

    rospy.loginfo(f"Saying received data {msg.data}")
    # HACK
    fname = siren_config.clip_location + '/tts/last_dynamic.wav'
    with open(fname, 'wb') as fstream:
        fstream.write(default_voice.to_audio(msg.data))

        a = AudioSegment.from_wav(fname)
        play(a)


# Farms out the execution of the soneme to the appropriate function
def service_cb(req, soneme):

    # # HACK prime the audio system with some silence.
    silence = AudioSegment.silent(duration=500)
    play(silence)

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
    for s in soneme.snodes:
        for speech in s.speeches:
            voice = Voice(lang=siren_config.voice_language, speed=int((siren_config.voice_wpm * speech.speed)), volume=(siren_config.volume * speech.volume), voice_id= siren_config.voice_id)
            # voice.say(speech.text)

            #HACK 
            fname = siren_config.clip_location + '/tts/' + soneme.name + '.wav'
            with open(fname, 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.text))

            a = AudioSegment.from_wav(fname)
            play(a)


            if siren_config.save_wavs:
                with open(siren_config.clip_location + '/tts/' + soneme.name + '.wav', 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.text))

    return True

def execute_directional(req, soneme):
    for s in soneme.snodes:
        for speech in s.speeches:
            voice = Voice(lang=siren_config.voice_language, speed=int((siren_config.voice_wpm * speech.speed)), volume=(siren_config.volume * speech.volume), voice_id= siren_config.voice_id)
            direction = cardinalize(req.transform)
            # voice.say(speech.get_dyn_text(direction))

            #HACK 
            fname = siren_config.clip_location + '/tts/' + soneme.name + '.wav'
            with open(fname, 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.get_dyn_text(direction)))

            a = AudioSegment.from_wav(fname)
            play(a)

            if siren_config.save_wavs:
                with open(siren_config.clip_location + '/tts/' + soneme.name + '_' + direction + '.wav', 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.get_dyn_text(direction)))

    return True

def execute_target(req, soneme):
    pass

def execute_quantity(req, soneme):
    for s in soneme.snodes:
        for speech in s.speeches:
            voice = Voice(lang=siren_config.voice_language, speed=int((siren_config.voice_wpm * speech.speed)), volume=(siren_config.volume * speech.volume), voice_id= siren_config.voice_id)
            quantity = str(int(req.quantity * 100)) + " percent"
            # voice.say(speech.get_dyn_text(quantity))

            #HACK 
            fname = siren_config.clip_location + '/tts/' + soneme.name + '.wav'
            with open(fname, 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.get_dyn_text(quantity)))

            a = AudioSegment.from_wav(fname)
            play(a)

            if siren_config.save_wavs:
                with open(siren_config.clip_location + '/tts/' + soneme.name + '_' + quantity + '.wav', 'wb') as fstream:
                    fstream.write(voice.to_audio(speech.get_dyn_text(quantity)))

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
    siren_info = rospy.get_param('vectors/out/TTSSiren')
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
        for key in sonemes:
            s = sonemes[key]
            if sym == s.id:
                rospy.loginfo("Found match beteween symbol %s and soneme %s, associating data."%(sym, s.id))
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

        service_name = 'siren/tts/'+ soneme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for soneme %s at service endpoint: %s'%(soneme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, soneme=soneme: service_cb(req, soneme))

    # If a dynamic input has been defined, we need to prepare a topic subscriber with callbacks.
    if siren_config.dynamic_input:
        rospy.loginfo("Dynamic input detected!")

        rospy.loginfo("Creating default voice for dynamic input")
        default_voice = Voice(lang=siren_config.voice_language, speed=int((siren_config.voice_wpm)), volume=(siren_config.volume ), voice_id= siren_config.voice_id)
        default_voice.say("hello")

        topic_name = 'siren/tts/' + siren_config.dynamic_input.topic
        if siren_config.dynamic_input.type.lower() == "string":
            from std_msgs.msg import String
            rospy.loginfo(f"Creating dynamic input topic at {topic_name}.")
            rospy.Subscriber(topic_name, String, text_cb)
        else:
            rospy.logerr("Dynamic input with unimplemented topic type sent.")


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass