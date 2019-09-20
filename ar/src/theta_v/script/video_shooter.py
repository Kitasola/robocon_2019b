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
        "captureMode": "video"
        }
    }
}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})

# Start Shooting Video
command = {"name": "camera.startCapture"}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})
print "Satrt"

time.sleep(2)

# Stop Shooting Video
command = {"name": "camera.stopCapture"}
result = requests.post(camera_uri + 'osc/commands/execute', data=json.dumps(command), headers={'Content-Type': 'application/json'})
print "Finish"

# Save
command = {"stateFingerprint": "FIG_0000"}
now_finger = json.loads(requests.post(camera_uri + 'osc/checkForUpdates', data=json.dumps(command), headers={'Content-Type': 'application/json'}).text)["stateFingerprint"]
state = json.loads(requests.post(camera_uri + 'osc/state').text)
file_uri = state["state"]["_latestFileUrl"]
save_file = requests.get(file_uri)
try:
    with open(now_finger, "wb") as out:
        out.write(save_file.content)
except Exception as err:
    print "%s" % (err)
