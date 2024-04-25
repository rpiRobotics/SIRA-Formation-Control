import socket
import os

sira_name = socket.gethostname()
sira_follower = 'sirar'
sira_leader = 'sirab'
sira_leader_frame = 'ridgeBframe_from_marker_right'
sira_follower_frame = 'ridgeRframe'
if sira_name == 'sirab-T15' or os.environ['SIRAB_OR_SIRAR'] == 'sirab':
    sira_follower = 'sirab'
    sira_leader = 'sirar'
    sira_leader_frame = 'ridgeRframe_from_marker_left'
    sira_follower_frame = 'ridgeBframe'
