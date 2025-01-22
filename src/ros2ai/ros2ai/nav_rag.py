#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ros2ai_msgs.srv import Nav2Gpt

import base64
from io import BytesIO

import sounddevice as sd
import scipy.io.wavfile as wav
import whisper
import time
import json
import requests
import openai

import os
from dotenv import load_dotenv

# Muat file .env
load_dotenv(dotenv_path="/home/badri/nav2_gpt/.env")

# Ambil API Key dari environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

class NavGpt(Node):
    def __init__(self):
        super().__init__("nav_gpt")
        self.bridge = CvBridge()
        self.cli = self.create_client(Nav2Gpt, 'goToPose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("connected to goToPose server")
        self.req = Nav2Gpt.Request()

    def record_audio(self, filename, duration, fs=44100):
        print("Recording...")
        audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
        sd.wait()  # Wait until recording is finished
        print("Recording finished.")
        wav.write(filename, fs, audio)

    def execute_command(self, command):
        service = command["service"]
        args = command["args"]
        if service == "/goToPose":
            x = args["x"]
            y = args["y"]
            theta = args["theta"]
            print(f"Executing goToPose with x={x}, y={y}, theta={theta}")
            self.req.x = float(x)
            self.req.y = float(y)
            self.req.theta = float(theta)
            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            print(self.future.result())
        elif service == "/wait":
            print("Executing wait")
            time.sleep(5)
        else:
            print(f"Unknown service: {service}")

    def invoke_gpt4(self, prompt):
        # Example call to your separate agent
        url = "http://localhost:8080/agent/"
        payload = {"text": prompt}
        response = requests.post(url, json=payload)
        response_json = response.json()
        return response_json["generated"]["command"]["parameters"]["target"]

    def speechtotext(self, path):
        with open(path, "rb") as audio_file:
            # For English translation:
            translation = openai.Audio.translate("whisper-1", audio_file)
            # Or for transcription, use:
            # translation = openai.Audio.transcribe("whisper-1", audio_file)
        return translation["text"]

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavGpt()
        input("Press Enter to start recording...")
        filename = "recorded_audio.wav"
        duration = 10
        node.record_audio(filename, duration)

        print("Transcribing...")
        result = node.speechtotext(filename)
        print("Transcription:", result)

        # Example usage of custom prompt
        gpt_output = node.invoke_gpt4(result)
        print("GPT output:", gpt_output)

        # Hard-coded example of mapping response to a command
        if gpt_output == "lecturer_room":
            commands_str = '[{"args":{"x":-2.36,"y":0.76,"theta":90},"service":"/goToPose"}]'
            commands = json.loads(commands_str)
            for command in commands:
                node.execute_command(command)

    except Exception as e:
        print(f"Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
