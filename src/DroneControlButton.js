import React, { useState } from 'react';

function DroneControlButton() {
  const [isRunning, setIsRunning] = useState(false);
  const [errorMessage, setErrorMessage] = useState(null);  // Hata mesajını ekledik

  const runDroneScript = () => {
    setIsRunning(true);
    setErrorMessage(null);  // Hata mesajını temizliyoruz

    // Flask backend'e istek gönderiyoruz
    fetch('http://localhost:5000/run-drone')
      .then(response => {
        if (!response.ok) {
          return response.json().then(data => {  // Hata mesajını göster
            throw new Error(data.error || 'Bilinmeyen bir hata oluştu');
          });
        }
        return response.json();
      })
      .then(data => {
        console.log('Python betiği çıktısı:', data);
        setIsRunning(false);  // Buton tekrar aktif hale gelir
      })
      .catch(error => {
        console.error('Hata oluştu:', error.message);
        setErrorMessage(error.message);  // Hata mesajını ekranda göster
        setIsRunning(false);
      });
  };

  return (
    <div style={{
      position: 'fixed',        // Sabit pozisyon
      top: '90px',              // Sol üst köşeye taşı
      left: '20px',             // Sol üst köşeye taşı
      width: '60px',            // Buton genişliği
      height: '60px',           // Buton yüksekliği
    }}>
      <button onClick={runDroneScript} disabled={isRunning} style={{
        width: '100%',
        height: '100%',
        fontSize: '12px',        // Buton metninin boyutu
        cursor: 'pointer',
        backgroundColor: '#4CAF50',  // Arka plan rengi
        color: 'white',              // Yazı rengi
        border: 'none',              // Kenarlık
        borderRadius: '5px',         // Köşeleri yuvarla
        textAlign: 'center',
        lineHeight: '60px'           // Yazıyı butonun ortasında tut
      }}>
        {isRunning ? 'Running...' : 'çalıştır'}
      </button>

      {errorMessage && (  // Eğer bir hata mesajı varsa göster
        <div style={{ color: 'red', marginTop: '10px' }}>
          <strong>Hata:</strong> {errorMessage}
        </div>
      )}
    </div>
  );
}

export default DroneControlButton;

