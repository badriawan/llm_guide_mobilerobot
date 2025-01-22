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
#import whisper

import time
import json

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
        sd.wait()
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
        response = openai.ChatCompletion.create(
            model="gpt-4-0613",
            # model="gpt-4o",
            messages=[{"role": "user", "content": prompt}],
            temperature=0
        )
        return response.choices[0].message.content
    
    def speechtotext(self, path):
        with open(path, "rb") as audio_file:
            translation = openai.Audio.translate("whisper-1", audio_file)
        return translation

def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavGpt()
        input("Press Enter to start recording...")
        filename = "recorded_audio.wav"
        duration = 10
        node.record_audio(filename, duration)

        print("Transcribing...")
        #model = whisper.load_model("turbo")
        #result = model.transcribe(filename)
        result = node.speechtotext(filename)
        print("Transcription:", result["text"])

        prompt = """
       Use this JSON schema to achieve the user's goals:\n\
                {
        "$schema": "http://json-schema.org/draft-04/schema#",
        "type": "object",
        "properties": {
            "service": {
                "type": "string",
                "default": "/goToPose"
            },
            "args": {
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "min": -10,
                        "max": 10
                    },
                    "y": {
                        "type": "number",
                        "min": -10,
                        "max": 10
                    },
                    "theta": {
                        "type": "number",
                        "min": -3.14,
                        "max": 3.14
                    }
                },
                "required": [
                    "x",
                    "y",
                    "theta"
                ]
            }
        },
        "required": [
            "service",
            "args"
        ]
    }
    \n\
                Respond as a list of JSON objects.\
                Do not include explanations or conversation in the response
                
                
    "role": "user",
                    "content": f"\
        remember these the coordinates of the kitchen is x: -4, y: 4, theta: 0
        the coordinates of the bedroom is x: 3, y: 4, theta: 0
        the coordinates of the living room is x:-0.5, y:4.5, theta:0
        the coordinates of the study room is x: 8.00, y:-6.35, theta:0
        the coordinates of the dining room is x:6.9, y:3.28, theta:0 
        the coordinates of the toilet is x: -4.5, y: -2, theta:0
        """

        # Append user’s transcription to your prompt
        prompt += result["text"]
        prompt += """\nRespond only with the output in the exact format..."""

        gpt_output = node.invoke_gpt4(prompt)
        print(gpt_output)

        commands = json.loads(gpt_output)
        for command in commands:
            node.execute_command(command)

    except Exception as e:
        print(f"Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()