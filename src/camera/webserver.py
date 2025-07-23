from flask import Flask, Response
import cv2
import os

app = Flask(__name__)

# Automatically find ZED2's index
def find_zed_camera_index(keyword="ZED"):
    for i in range(5):
        path = f"/sys/class/video4linux/video{i}/name"
        if os.path.exists(path):
            with open(path, 'r') as f:
                name = f.read().strip()
                if keyword.lower() in name.lower():
                    print(f"Found ZED2 camera at index {i} ({name})")
                    return i
    print("Not found ZED2! Defaulting to index 1")
    return 1

index = find_zed_camera_index(keyword="ZED")

# Open the camera (defaulting to /dev/video1 if ZED not found)
cap = cv2.VideoCapture(index)

if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera at index {index}")

# MJPEG stream generator with half-frame crop and resize
def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            print("Failed to capture frame.")
            break
        else:
            height, width, _ = frame.shape

            frame = frame[:, :width // 2]  # crop nửa trái

            # ✅ Resize ra khung rộng hơn như 960x540 hoặc 1280x720
            frame = cv2.resize(frame, (1280, 720))  # tùy bạn chọn

            # Lưu ý: 1280x720 là 16:9, sẽ fill được màn hình tốt hơn


            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # MJPEG stream
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Video stream route           
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# HTML display route
@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>ZED2 STREAM</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            html, body {
                margin: 0;
                padding: 0;
                background-color: #000;
                height: 100%;
                width: 100%;
                overflow: hidden;
                display: flex;
                flex-direction: column;
                align-items: center;
                justify-content: flex-start;
            }
            h1 {
                color: white;
                font-family: sans-serif;
                margin: 10px;
            }
            .video-container {
                flex: 1;
                display: flex;
                justify-content: center;
                align-items: center;
                width: 100%;
                height: 100%;
            }
            img {
                max-width: 100%;
                max-height: 100%;
                width: auto;
                height: auto;
                object-fit: contain;
                display: block;
            }
        </style>
    </head>
    <body>
        <h1>ZED2 STREAM</h1>
        <div class="video-container">
            <img src="/video_feed" alt="ZED2 Video Stream">
        </div>
    </body>
    </html>
    """

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
