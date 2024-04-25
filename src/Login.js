import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { Link, useNavigate } from 'react-router-dom';
import { useAuth } from './Root';

function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');

  const auth = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    if (auth.isLoggedIn) {
      navigate('/app');
    }
  }, [auth.isLoggedIn, navigate]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError(''); // Mevcut hataları temizle

    // URL doğru yapılandırıldı mı kontrol edin
    const serverUrl = 'http://0.0.0.0:3000/auth/login';

    try {
      const response = await axios.post(serverUrl, { email, password }, {
        headers: {
          'Content-Type': 'application/json'
        },
        withCredentials: true // CORS ve çerezler için önemli
      });

      if (response.data.token) {
        localStorage.setItem('authToken', response.data.token);
        auth.setUser(response.data.user); // Kullanıcı durumunu güncelle
        navigate('/app');
      } else {
        setError('Login failed. Please check your credentials.');
      }
    } catch (err) {
      // Hata mesajını daha anlaşılır hale getir
      if (err.response) {
        // Sunucu tarafından döndürülen hata mesajı
        setError(err.response.data.message || 'Login failed. Please check your credentials.');
      } else if (err.request) {
        // İstek yapıldı ancak yanıt alınamadı
        setError('The server did not respond. Please check your network connection and the server status.');
      } else {
        // İstek yapılamadan bir hata oluştu
        setError('An error occurred while setting up the request. Please try again.');
      }
    }
  };

  return (
    <div className="login-container">
      <div className="logo-container">
        <img src="/gokmen512.png" alt="Logo" className="logo" />
      </div>
      <div className="form-container">
        <form onSubmit={handleSubmit} className="login-form">
          <label>Email:
            <input 
              type="email" 
              value={email} 
              onChange={(e) => setEmail(e.target.value)} 
              required 
            />
          </label>
          <label>Password:
            <input 
              type="password" 
              value={password} 
              onChange={(e) => setPassword(e.target.value)} 
              required 
            />
          </label>
          <button type="submit">Login</button>
          {error && <div className="error-message">{error}</div>}
        </form>
        <div className="signup-link">
          <p>Dont have an account? <Link to="/signup">Sign Up</Link></p>
        </div>
      </div>
    </div>
  );
}

export default Login;
