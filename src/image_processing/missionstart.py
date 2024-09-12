from flask import Flask, jsonify
import subprocess
import os

app = Flask(__name__)

@app.route('/run-drone', methods=['GET'])
def run_drone():
    try:
        # Python betiğinin bulunduğu tam yolu belirtiyoruz
        script_path = os.path.join(os.getcwd(), 'karekok.py')
        
        # Python betiğini çalıştırıyoruz
        result = subprocess.run(['python3', script_path], capture_output=True, text=True)
        if result.returncode != 0:
            app.logger.error(f'Python betiği hata verdi: {result.stderr}')
            return jsonify({'error': result.stderr}), 500
        app.logger.info(f'Python betiği başarılı şekilde çalıştı: {result.stdout}')
        return jsonify({'output': result.stdout}), 200
    except Exception as e:
        app.logger.error(f'Beklenmeyen bir hata oluştu: {str(e)}')
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)

