import time
import requests
import json

# Setting
camera_uri = 'http://192.168.1.1:80/'

# Check Battery
state = json.loads(requests.post(camera_uri + 'osc/state').text)
battery = state["state"]["batteryLevel"]
print "RICOH THETA V Power: " + str(battery * 100) + "%"

# Change Mode to Video
command = {"name": "camera.setOptions", "parameters": {
    "options": {
        "captureMode": "image"
        }
    }
}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})

# Take Picture
print "5"
time.sleep(2)
print "3"
time.sleep(1)
print "2"
time.sleep(1)
print "1"
time.sleep(1)
print "Click"
command = {"name": "camera.takePicture"}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})

# Wait Saving
command = {"stateFingerprint": "FIG_0000"}
prev_finger = json.loads(requests.post(camera_uri + 'osc/checkForUpdates', data=json.dumps(command), headers={'Content-Type': 'application/json'}).text)["stateFingerprint"]
command = {"stateFingerprint": prev_finger}
now_finger = prev_finger
while now_finger == prev_finger:
    print "Wait ..."
    time.sleep(1)
    now_finger = json.loads(requests.post(camera_uri + 'osc/checkForUpdates', data=json.dumps(command), headers={'Content-Type': 'application/json'}).text)["stateFingerprint"]

# Save
print "Save File"
state = json.loads(requests.post(camera_uri + 'osc/state').text)
file_uri = state["state"]["_latestFileUrl"]
save_file = requests.get(file_uri)
try:
    with open(now_finger + ".JPG", "wb") as out:
        out.write(save_file.content)
except Exception as err:
    print "%s" % (err)

# Delete
print "Delete File"
command = {"name": "camera.delete", "parameters": {
    "fileUrls": [file_uri]
    }
}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})

