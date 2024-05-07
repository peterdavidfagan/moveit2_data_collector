import json
import subprocess

if __name__=="__main__":
    # open camera config yaml

    with open("../config/camera_params.json", "r") as f:
        camera_params = json.load(f)

    service_name = camera_params["camera_pose_service"]
    service_type = "zed_interfaces/srv/SetPose"
    pos = str(camera_params["extrinsic_params"]["translation"])
    orient = str(camera_params["extrinsic_params"]["rpy"])
    data = "'{pos: " + pos + ", orient:" + orient + "}'"

    # set zed camera pose
    result = subprocess.Popen(["ros2", "service", "call", f"{service_name}", f"{service_type}", data])
    print(result.stdout)