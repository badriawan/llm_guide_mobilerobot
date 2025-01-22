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

import openai
import os
from dotenv import load_dotenv
# from openai import OpenAI

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
            # model="gpt-4-0613",
            # # model="gpt-4o",
            # messages=[{"role": "user", "content": prompt}],
            # temperature=0
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": [
                        {
                            "text": "remember these the coordinates \n\n"
                                    "the living room is x:-0.5, y:4.5, theta:90\n"
                                    "the study room is x: 8.00, y:-6.35, theta:0\n"
                                    "the bedroom is is x: 3, y: 4,theta:-90\n"
                                    "the dining room is x:6.9, y:3.28, theta:0  \n"
                                    "the kitchen is x: -4, y: 4, theta: 0 \n"
                                    "the toilet is x: -4.5, y: -2, theta:0 \n\n"
                                    "Respond only with the output in the exact format specified in the system prompt, with no explanation or conversation.",
                            "type": "text"
                        }
                    ]
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ]
            ,
            response_format={
                "type": "json_schema",
                "json_schema": {
                    "name": "goToPose",
                    "schema": {
                        "type": "object",
                        "required": [
                            "service",
                            "args"
                        ],
                        "properties": {
                            "args": {
                                "type": "object",
                                "required": [
                                    "x",
                                    "y",
                                    "theta"
                                ],
                                "properties": {
                                    "x": {
                                        "max": 10,
                                        "min": -10,
                                        "type": "number",
                                        "description": "X-coordinate, must be in the range 0 to 10."
                                    },
                                    "y": {
                                        "max": 10,
                                        "min": -10,
                                        "type": "number",
                                        "description": "Y-coordinate, must be in the range 0 to 10."
                                    },
                                    "theta": {
                                        "max": 180,
                                        "min": -180,
                                        "type": "number",
                                        "description": "Orientation angle in radians, must be in the range -3.14 to 3.14."
                                    }
                                },
                                "description": "Arguments for the pose action including position and angle.",
                                "additionalProperties": False
                            },
                            "service": {
                                "enum": [
                                    "/goToPose"
                                ],
                                "type": "string",
                                "description": "The service endpoint for the pose action."
                            }
                        },
                        "additionalProperties": False
                    },
                    "strict": True
                }
            },
            temperature=0.5,
            max_tokens=256,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0
        )
        return response.choices[0].message.content

    def speechtotext(self, path):
        audio_file = open(path, "rb")
        translation = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file
        )
        return translation.text


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavGpt()
        input("Press Enter to start recording...")
        filename = "recorded_audio.wav"
        duration = 10
        node.record_audio(filename, duration)

        print("Transcribing...")
        # model = whisper.load_model("turbo")
        result = node.speechtotext(filename)
        print("Transcription:", result)

        prompt = result
        gpt_output = node.invoke_gpt4(prompt)
        print(gpt_output)
        commands_string = "["+gpt_output+"]"
        commands = json.loads(commands_string)
        
        # tes = '[{"args":{"x":4,"y":4,"theta":180},"service":"/goToPose"}]'
        # commands = json.loads(tes)
        
        
        for command in commands:
            node.execute_command(command)
            
            
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()