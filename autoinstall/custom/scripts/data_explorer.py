import cv2
import csv
from datetime import datetime
from pathlib import Path
import argparse


class DataExplorer:
    def __init__(self, video: str):
        self.video = Path(video)
        self.cap = cv2.VideoCapture(str(self.video))
        self.duration = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.frame_idx = 0
        self.prev_idx = -1
        self.playing = False
        self.log_name = None
        self.file_names = []
        self.cam = 0
        self.enhance = 0

        print(f"Data Explorer - Controls\n========================")
        print(
            "w/s:\tplay / pause\na/d:\tprev frame / next frame \n0-4:\tSwitch cameras \nt:\tlog timestamp \nesc:\texit"
        )

    def update_frame_idx(self, value: int):
        self.frame_idx = value
        cv2.setTrackbarPos("Frame #", "window", self.frame_idx)
        self.update_file_name()

    def save_file_names(self):
        if self.file_names:
            now = datetime.now().strftime("%y%m%d_%H%M%S")
            filename = f"{now}_timestamps.csv"

            with open(filename, "w", newline="") as ts_file:
                writer = csv.writer(ts_file)
                writer.writerow(self.file_names)

            print(f"Saved file names to {filename}")

    def update_file_name(self):
        idx = self.frame_idx // 60
        if idx != self.prev_idx:
            self.log_name = f"{self.video.stem}_{idx}.mcap"
            self.prev_idx = idx
            print(self.log_name)

    def next_frame(self):
        self.frame_idx = min(self.duration, self.frame_idx + 1)

    def prev_frame(self):
        self.frame_idx = max(0, self.frame_idx - 1)

    def control(self, key):
        # General
        if key == ord("w"):
            self.playing = True
        if key == ord("s") or key == ord("d") or key == ord("a"):
            self.playing = False
        if key == ord("d"):
            self.next_frame()
            self.update_file_name()
        if key == ord("a"):
            self.prev_frame()
            self.update_file_name()

        # Switch cameras
        if key == ord("0") or key == ord("1") or key == ord("2") or key == ord("3") or key == ord("4"):
            self.cam = int(chr(key))

        # Enhance
        if key == ord("e"):
            self.enhance = not self.enhance

        # Save current file name
        if key == ord("t"):
            if self.log_name:
                print(f"Logging {self.log_name}")
                self.file_names.append(self.log_name)

    def start(self):
        # Setup UI
        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("Frame #", "window", 0, self.duration, self.update_frame_idx)

        while self.cap.isOpened():
            # Update UI
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.frame_idx)
            cv2.setTrackbarPos("Frame #", "window", self.frame_idx)

            ret, frame = self.cap.read()
            if ret == True:
                # Switch cameras
                if self.cam == 1:
                    frame = frame[:240, :320]  # Top left
                elif self.cam == 2:
                    frame = frame[:240, 320:]  # Top right
                elif self.cam == 3:
                    frame = frame[240:480, :320]  # Bottom left
                elif self.cam == 4:
                    frame = frame[240:480, 320:]  # Bottom right

                if self.enhance:
                    frame = cv2.resize(frame, dsize=None, fx=2, fy=2)

                cv2.imshow("window", frame)
                key = cv2.waitKey(1)
                self.control(key)

                # Exit
                if key == 27:
                    break

                if self.playing:
                    self.next_frame()
                    self.update_file_name()
            else:
                break

        self.save_file_names()
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Data explorer")
    parser.add_argument("mp4", type=str, help="Path to .mp4 file")
    args = parser.parse_args()

    explorer = DataExplorer(args.mp4)
    explorer.start()
