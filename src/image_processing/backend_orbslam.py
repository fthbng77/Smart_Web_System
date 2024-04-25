from flask import Flask
import subprocess

app = Flask(__name__)

@app.route('/start-ai-model', methods=['POST'])
def start_ai_model():
    try:
        # ROS düğümünü başlatmak için kullanılacak komut
        command = "roslaunch gokmen.launch"
        subprocess.Popen(command, shell=True)
        return "ORB_SLAM3 başlatıldı!"
    except Exception as e:
        return str(e)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)