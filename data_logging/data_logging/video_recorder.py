import os
import signal
import subprocess
from pathlib import Path


class MP4Recorder:
    def __init__(self, topic: str, output: Path, codec: str = "h265", bitrate: int = 4000000):
        # Codec verification
        codec = codec.lower()
        if codec not in ["h264", "h265", "mjpeg"]:
            print(f"Codec argument incorrect, defaulting to h265")
            codec = "h265"

        self.process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ros_deep_learning",
                "video_output.ros2.launch",
                f"topic:={topic}",
                f"output:=file://{output}",
                f"output_codec:={codec}",
                f"output_bitrate:={bitrate}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid,
        )

    def terminate(self):
        # NOTE process .terminate and .kill aren't working as intended
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)


if __name__ == "__main__":
    mp4 = MP4Recorder(topic="/test", output=Path("/over/here"))
