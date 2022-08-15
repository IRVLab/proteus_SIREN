#!/usr/bin/python3
import rospy
from rosnode import get_node_names
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from pydub import AudioSegment
from pydub.playback import play as pydub_play

import sys
import xml.etree.ElementTree as ET
from proteus.srv import SymbolTrigger, SymbolDirectional, SymbolTarget, SymbolQuantity
from proteus.soneme import Soneme, SNode, SNodeClip, SNodeSpeech
from proteus.siren import SirenConfig

rospy.init_node('ogg_siren_server', argv=None, anonymous=True)
siren_config = None

def change_volume(audio_seg, volume=50):
    return audio_seg

def change_speed(audio_seg, speed=1.0):
    return audio_seg._spawn(audio_seg.raw_data, overrides={"frame_rate": int(audio_seg.frame_rate * speed)})

# Fit transform into text description of cardinal direction
def cardinalize(transform):
    q = transform.rotation
    rpy = euler_from_quaternion([q.x, q.y, q.z, q.w]) #We only actually need pitch and yaw, roll is ignored here.

    ret = "" # Return string.

    # Pitch handling
    if rpy[1] > 0:
        ret+= "up"
    elif rpy[1] < 0:
        ret+= "down"

    # Don't need to do this for a search string.
    # # Add a conjunction if there's pitch involved.
    # if rpy[1] != 0 and rpy[2] != 0:
    #     ret+= " and "

    # Yaw handling
    if rpy[2] > 0:
        ret+= "left"
    elif rpy[2] < 0:
        ret += "right"

    return ret

# Fit quantity into available bins.
def bin_quant(quantity, bins):
    return min(bins, key=lambda x:abs(int(x)-quantity))


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
    dir = siren_config.clip_location
    for s in soneme.snodes:
        for audio in s.audios:
            clip = AudioSegment.from_wav(dir + '/' + audio.filename)
            clip = change_speed(clip, audio.speed)
            clip = change_volume(clip, audio.volume)
            pydub_play(clip)

    return True

def execute_directional(req, soneme):
    dir = siren_config.clip_location
    cardinal_str = cardinalize(req.transform) # Fit transform into text description.

    for s in soneme.snodes:
        for audio in s.audios:
            clip = AudioSegment.from_wav(dir + '/' + audio.get_dyn_fname(cardinal_str))
            clip = change_speed(clip, audio.speed)
            clip = change_volume(clip, audio.volume)
            pydub_play(clip)

    return True

def execute_target(req, soneme):
    pass

def execute_quantity(req, soneme):
    dir = siren_config.clip_location
    for s in soneme.snodes:
        for audio in s.audios:
            val = bin_quant(req.quantity * 100, audio.options) #Fit battery value to closest available value.
            clip = AudioSegment.from_wav(dir + '/' + audio.get_dyn_fname(val))
            clip = change_speed(clip, audio.speed)
            clip = change_volume(clip, audio.volume)
            pydub_play(clip)

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
    siren_info = rospy.get_param('vectors/out/ClipSIREN')
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
        for key,soneme in sonemes.items():
            if sym == key:
                rospy.loginfo("Found match beteween symbol %s and soneme %s, associating data."%(sym, key))
                rospy.logdebug("Call type: %s"%(symbols.get(key).get('call_type')))
                soneme.set_call_type(symbols.get(sym).get('call_type'))
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

        service_name = 'siren/clip/'+ soneme.name.replace(' ', '_')

        rospy.loginfo('Advertising a service for soneme %s at service endpoint: %s'%(soneme.id, service_name))
        rospy.Service(service_name, service_class, lambda req, soneme=soneme: service_cb(req, soneme))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

else:
    pass