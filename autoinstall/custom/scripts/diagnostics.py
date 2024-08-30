from datetime import datetime
from email.mime.text import MIMEText
import shutil
import smtplib
import subprocess
from threading import Lock, Thread
import time

import requests

import schedule


class Monitor:
    def __init__(
        self, sms_number: str, sms_user: str, sms_pw: str, email_sender: str, email_recipient: str, email_pw: str
    ):
        self.lock = Lock()
        self.alerts = []

        # Details for notifications
        self.sms_number = sms_number
        self.sms_user = sms_user
        self.sms_pw = sms_pw

        self.email_sender = email_sender
        self.email_recipient = email_recipient
        self.email_pw = email_pw

        # Disable SSL warning
        requests.urllib3.disable_warnings(requests.packages.urllib3.exceptions.InsecureRequestWarning)

        # Run an initial check and then schedule checks every hour after
        self.monitor()
        schedule.every().hour.do(self.monitor)

    def monitor(self):
        """Run all system and device checks."""
        threads = []

        # Check networked devices
        threads.append(Thread(target=self.check_networked_device, kwargs={"ip": "cam-near.lan"}))
        threads.append(Thread(target=self.check_networked_device, kwargs={"ip": "cam-far.lan"}))
        threads.append(Thread(target=self.check_networked_device, kwargs={"ip": "cam-thermal.lan"}))

        # Check topic hz
        threads.append(Thread(target=self.check_hz, kwargs={"topic": "/camera_far/resized/detection", "hz": 10}))
        threads.append(Thread(target=self.check_hz, kwargs={"topic": "/camera_near/resized/detection", "hz": 10}))
        threads.append(Thread(target=self.check_hz, kwargs={"topic": "/camera_thermal/detection", "hz": 10}))
        threads.append(
            Thread(target=self.check_hz, kwargs={"topic": "/camera_near/zoomed/resized/detection", "hz": 10})
        )

        # Check disk usage
        threads.append(Thread(target=self.check_disk, kwargs={"disk": "/mnt/emmc"}))
        threads.append(Thread(target=self.check_disk, kwargs={"disk": "/mnt/ssd"}))

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Send current alerts
        self.send_alert()

    def start(self):
        """Maintain a while loop that runs scheduled tasks every second."""
        while True:
            schedule.run_pending()
            time.sleep(1)

    def log(self, msg: str):
        """Timestamp and log and message to the alerts system."""
        timestamp = datetime.now().strftime("%y%m%d-%H%M%S")
        with self.lock:
            self.alerts.append(f"[{timestamp}] {msg}")

    def check_disk(self, disk: str, threshold: float = 0.85):
        """Send an alert if the disk usage is above the specified threshold."""
        total, used, _ = shutil.disk_usage(disk)
        usage_pc = used / total
        if usage_pc > threshold:
            self.log(f"Disk {disk} at {usage_pc*100:.0f}% capacity [{used/1e9:.0f}GB/{total/1e9:.0f}GB]")

    def check_hz(self, topic: str, hz: float, buffer: float = 0.1):
        """Send an alert if the topic is not running at the specified Hz."""
        try:
            subprocess.run(["/opt/ros/iron/install/bin/ros2", "topic", "hz", topic], timeout=5, capture_output=True)
        except subprocess.TimeoutExpired as e:
            if (e.output is not None) and ("average rate" in str(e.output, "utf-8")):
                output = e.output.decode("utf-8")
                observed_hz = float(output.splitlines()[0].split(" ")[-1])
                if observed_hz < (hz - buffer):
                    self.log(f"Topic {topic} rate at {observed_hz:.1f}Hz, slower than expected {hz:.1f}Hz")
            else:
                self.log(f"Topic {topic} not published")

    def check_networked_device(self, ip: str):
        """Send an alert if the networked device is not accessible."""
        response = subprocess.run(["ping", "-c1", ip], stdout=subprocess.DEVNULL).returncode
        if response != 0:
            print(f"{ip} is down")
            self.log(f"Device at {ip} not found")

    def send_alert(self):
        if self.alerts:
            msg = "\n".join(self.alerts)
            print(msg)
            self.send_email(msg)
            self.send_sms(msg)
            self.alerts.clear()

    def send_email(self, msg: str):
        email = MIMEText(msg)
        email["Subject"] = "[ERROR] LAARMA"
        email["From"] = self.email_sender
        email["To"] = self.email_recipient

        try:
            with smtplib.SMTP(host="smtp-mail.outlook.com", port=587) as smtp:
                smtp.starttls()
                smtp.login(self.email_sender, self.email_pw)
                smtp.sendmail(self.email_sender, self.email_recipient, email.as_string())
        except Exception as e:
            self.send_sms("Email service is down")

    def send_sms(self, msg: str):
        ip = "https://192.168.1.1"
        url = f"{ip}/cgi-bin/sms_send?username={self.sms_user}&password={self.sms_pw}&number={self.sms_number}&text={msg}"

        try:
            requests.get(url, verify=False)
        except requests.exceptions.RequestException as e:
            self.send_email("SMS service is down")


if __name__ == "__main__":
    print(f"Starting diagnostics monitor")

    # Enter your details below to receive notifications
    # mobile notifications
    sms_number = "61#########"
    sms_user = "sms"
    sms_pw = "abcdefgh"
    # email notifications
    email_sender = "company_smtp@outlook.com"
    email_recipient = "user@company.com"
    email_pw = "abcdefgh"

    monitor = Monitor(sms_number, sms_user, sms_pw, email_sender, email_recipient, email_pw)
    monitor.start()
