#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from blindassistant.srv import VLQuery, VLQueryResponse
from llama_cpp import Llama
from llama_cpp.llama_chat_format import MoondreamChatHandler, Llama3VisionAlphaChatHandler, format_qwen
import cv2
import base64


bridge = CvBridge()


# Initialize the vision-language chat handler with the clip model
clip_model_path = "src/blindassistant/src/mmproj-Qwen2-VL-2B-Instruct-f16.gguf"
chat_handler = Llama3VisionAlphaChatHandler(clip_model_path=clip_model_path)

# Load the Qwen2-VL model with the chat handler
llm = Llama(
    model_path="src/blindassistant/src/Qwen2-VL-2B-Instruct-Q4_0.gguf",
    chat_handler=chat_handler,
    n_gpu_layers=32,
    verbose=True
)


def handle_vl_query(req):
    try:
        rospy.loginfo("[VL Server] Received image and prompt.")


        # Convert ROS Image to OpenCV RGB
        cv_image = bridge.imgmsg_to_cv2(req.image, desired_encoding="rgb8")


        # Encode image to base64 PNG
        _, buffer = cv2.imencode('.png', cv_image)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        image_url = f"data:image/png;base64,{image_base64}"


        # Build messages array with a system message + user image+text
        messages = [
            {"role": "system", "content": "You are an assistant who perfectly describes images."},
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": image_url}},
                    {"type": "text", "text": req.prompt.strip() or "Describe this image."}
                ]
            }
        ]


        rospy.loginfo(f"[VL Server] Prompt: {req.prompt.strip()}")
        rospy.loginfo("[VL Server] Calling model...")


        response = llm.create_chat_completion(
            messages=messages,
            temperature=0.2,
        )


        result = response["choices"][0]["message"]["content"]
        rospy.loginfo("[VL Server] Model responded.")
        return VLQueryResponse(result)


    except Exception as e:
        rospy.logerr(f"[VL Server] Exception: {e}")
        return VLQueryResponse("Error processing image.")


def qwen2_vl_server():
    rospy.init_node('qwen2_vl_node')
    rospy.loginfo("[VL Server] Starting Qwen2-VL server...")
    service = rospy.Service('vl_query', VLQuery, handle_vl_query)
    rospy.loginfo("[VL Server] Service 'vl_query' is ready.")
    rospy.spin()


if __name__ == "__main__":
    qwen2_vl_server()





